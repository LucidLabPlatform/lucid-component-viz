"""
Arena Projection Overlay - Interactive Calibration

Controls:
  - Click and drag the arena to move it
  - Scroll wheel to resize the arena
  - Arrow keys for fine position adjustment (1px)
  - Shift + Arrow keys for fine size adjustment (1px)
  - P: Print current size and origin to console
  - F: Toggle fullscreen
  - I: Toggle info overlay
  - ESC: Quit

MQTT:
  Subscribes to lucid/agents/+/components/ros_bridge/telemetry/aruco_confirmed
  Payload: {"marker_id": int, "x": float, "y": float, "z": float}
  Corners update from "?" to their marker ID + position when data arrives.
"""

import pygame
import sys
import os
import json
import math
import threading

try:
    import paho.mqtt.client as mqtt
    HAS_MQTT = True
except ImportError:
    HAS_MQTT = False

# ── Display ──────────────────────────────────────────────────────────────────
WINDOW_WIDTH = 1920
WINDOW_HEIGHT = 1080

# ── Calibrated arena (764px = 3m, 509px = 2m) ───────────────────────────────
arena_w = 764
arena_h = 534
arena_x = 570
arena_y = 0

ARENA_METERS_W = 3.0
ARENA_METERS_H = 2.0

# ── Visual ───────────────────────────────────────────────────────────────────
BORDER_THICKNESS = 4
BORDER_COLOR = (255, 255, 255)
BG_COLOR = (0, 0, 0)
GUIDE_COLOR = (40, 40, 40)
CORNER_UNKNOWN_COLOR = (100, 100, 100)
CORNER_KNOWN_COLOR = (0, 200, 100)
CORNER_RADIUS = 84
CORNER_FONT_SIZE = 14

RESIZE_STEP = 10
FINE_STEP = 1

# ── MQTT (configurable via env vars) ─────────────────────────────────────────
MQTT_BROKER = os.getenv("MQTT_HOST", "localhost")
MQTT_PORT = int(os.getenv("MQTT_PORT", "1883"))
MQTT_USERNAME = os.getenv("AGENT_USERNAME", "")
MQTT_PASSWORD = os.getenv("AGENT_PASSWORD", "")
CORNER_TOPIC = "lucid/agents/+/components/ros_bridge/telemetry/aruco_confirmed"

# ── Corner state ─────────────────────────────────────────────────────────────
# Four arena corners: top-left, top-right, bottom-right, bottom-left
# Each starts as unknown ("?") and gets filled by MQTT aruco_confirmed messages.
# "position" is the label position (which arena corner to draw at).
# "real" is the real-world (x, y) from MQTT once known.
corners = [
    {"label": "TL", "known": False, "marker_id": None, "real_x": None, "real_y": None},
    {"label": "TR", "known": False, "marker_id": None, "real_x": None, "real_y": None},
    {"label": "BR", "known": False, "marker_id": None, "real_x": None, "real_y": None},
    {"label": "BL", "known": False, "marker_id": None, "real_x": None, "real_y": None},
]
corners_lock = threading.Lock()


def corner_screen_positions():
    """Return (x, y) screen positions for each corner: TL, TR, BR, BL.
    Positions are at the exact arena corners so quarter circles sit flush."""
    return [
        (arena_x,           arena_y),
        (arena_x + arena_w, arena_y),
        (arena_x + arena_w, arena_y + arena_h),
        (arena_x,           arena_y + arena_h),
    ]


def assign_corner(marker_id, x, y):
    """Assign an aruco marker to the nearest unoccupied arena corner based on real-world position."""
    with corners_lock:
        # Skip if this marker is already assigned
        for c in corners:
            if c["known"] and c["marker_id"] == marker_id:
                # Update position
                c["real_x"] = x
                c["real_y"] = y
                return

        # Map real-world coords to a corner quadrant
        # Arena center in real-world: (ARENA_METERS_W/2, ARENA_METERS_H/2)
        # But we don't know the arena origin in real-world coords, so use
        # relative position: left/right and top/bottom halves.
        # For now, assign to first free slot in order of arrival.
        for c in corners:
            if not c["known"]:
                c["known"] = True
                c["marker_id"] = marker_id
                c["real_x"] = x
                c["real_y"] = y
                return


# ── MQTT callbacks ───────────────────────────────────────────────────────────
def on_connect(client, userdata, flags, reason_code, properties=None):
    print(f"[MQTT] Connected (rc={reason_code}), subscribing to corner topic")
    client.subscribe(CORNER_TOPIC, qos=0)


def on_message(client, userdata, msg):
    try:
        payload = json.loads(msg.payload)
        marker_id = payload.get("marker_id")
        x = payload.get("x")
        y = payload.get("y")
        if marker_id is not None and x is not None and y is not None:
            assign_corner(int(marker_id), float(x), float(y))
            print(f"[MQTT] Corner marker {marker_id} at ({x:.3f}, {y:.3f})")
    except (json.JSONDecodeError, TypeError) as e:
        print(f"[MQTT] Bad payload: {e}")


def start_mqtt():
    """Start MQTT client in a background thread. Fails silently if broker unavailable."""
    if not HAS_MQTT:
        print("[MQTT] paho-mqtt not installed — corners will stay as '?'")
        return None
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="arena-overlay")
    client.on_connect = on_connect
    client.on_message = on_message
    if MQTT_USERNAME:
        client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
    try:
        client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
    except Exception as e:
        print(f"[MQTT] Could not connect to {MQTT_BROKER}:{MQTT_PORT} — {e}")
        print("[MQTT] Corners will stay as '?' until broker is available")
        return None
    client.loop_start()
    return client


# ── Main ─────────────────────────────────────────────────────────────────────
def main():
    global arena_w, arena_h, arena_x, arena_y

    pygame.init()
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT), pygame.FULLSCREEN)
    pygame.display.set_caption("Arena Calibration")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("monospace", 16)
    corner_font = pygame.font.SysFont("monospace", CORNER_FONT_SIZE, bold=True)

    mqtt_client = start_mqtt()

    dragging = False
    drag_offset_x = 0
    drag_offset_y = 0
    fullscreen = True
    show_info = False

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.KEYDOWN:
                mods = pygame.key.get_mods()

                if event.key == pygame.K_ESCAPE:
                    running = False

                elif event.key == pygame.K_p:
                    print(f"\n=== ARENA CALIBRATION ===")
                    print(f"  Origin (top-left): ({arena_x}, {arena_y})")
                    print(f"  Size: {arena_w} x {arena_h} px")
                    print(f"  Center: ({arena_x + arena_w // 2}, {arena_y + arena_h // 2})")
                    print(f"  Px per meter: {arena_w / ARENA_METERS_W:.1f}")
                    with corners_lock:
                        for i, c in enumerate(corners):
                            if c["known"]:
                                print(f"  Corner {c['label']}: marker {c['marker_id']} "
                                      f"({c['real_x']:.3f}, {c['real_y']:.3f})")
                            else:
                                print(f"  Corner {c['label']}: ?")
                    print(f"=========================\n")

                elif event.key == pygame.K_f:
                    fullscreen = not fullscreen
                    if fullscreen:
                        screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT), pygame.FULLSCREEN)
                    else:
                        screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))

                elif event.key == pygame.K_i:
                    show_info = not show_info

                elif event.key == pygame.K_UP:
                    if mods & pygame.KMOD_SHIFT:
                        arena_h = max(10, arena_h - FINE_STEP)
                    else:
                        arena_y -= FINE_STEP
                elif event.key == pygame.K_DOWN:
                    if mods & pygame.KMOD_SHIFT:
                        arena_h += FINE_STEP
                    else:
                        arena_y += FINE_STEP
                elif event.key == pygame.K_LEFT:
                    if mods & pygame.KMOD_SHIFT:
                        arena_w = max(10, arena_w - FINE_STEP)
                    else:
                        arena_x -= FINE_STEP
                elif event.key == pygame.K_RIGHT:
                    if mods & pygame.KMOD_SHIFT:
                        arena_w += FINE_STEP
                    else:
                        arena_x += FINE_STEP

            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    mx, my = event.pos
                    rect = pygame.Rect(arena_x, arena_y, arena_w, arena_h)
                    if rect.collidepoint(mx, my):
                        dragging = True
                        drag_offset_x = mx - arena_x
                        drag_offset_y = my - arena_y
                elif event.button == 4:  # scroll up
                    arena_w += RESIZE_STEP
                    arena_h = int(arena_w * (ARENA_METERS_H / ARENA_METERS_W))
                elif event.button == 5:  # scroll down
                    arena_w = max(10, arena_w - RESIZE_STEP)
                    arena_h = int(arena_w * (ARENA_METERS_H / ARENA_METERS_W))

            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:
                    dragging = False

            elif event.type == pygame.MOUSEMOTION:
                if dragging:
                    mx, my = event.pos
                    arena_x = mx - drag_offset_x
                    arena_y = my - drag_offset_y

        # ── Draw ─────────────────────────────────────────────────────────────
        screen.fill(BG_COLOR)

        # Crosshair at window center
        pygame.draw.line(screen, GUIDE_COLOR, (WINDOW_WIDTH // 2, 0), (WINDOW_WIDTH // 2, WINDOW_HEIGHT))
        pygame.draw.line(screen, GUIDE_COLOR, (0, WINDOW_HEIGHT // 2), (WINDOW_WIDTH, WINDOW_HEIGHT // 2))

        # Arena rectangle
        arena_rect = pygame.Rect(arena_x, arena_y, arena_w, arena_h)
        pygame.draw.rect(screen, BORDER_COLOR, arena_rect)
        pygame.draw.rect(screen, (255, 0, 0), arena_rect, BORDER_THICKNESS)

        # ── Corner markers ───────────────────────────────────────────────────
        positions = corner_screen_positions()
        with corners_lock:
            for i, (cx, cy) in enumerate(positions):
                c = corners[i]
                if c["known"]:
                    color = CORNER_KNOWN_COLOR
                    text = f"#{c['marker_id']}"
                else:
                    color = CORNER_UNKNOWN_COLOR
                    text = "?"

                # Quarter circle tucked into each corner
                # Arc rect is centered on the corner point; only a 90 slice is drawn
                r = CORNER_RADIUS
                size = r * 2
                arc_rect = pygame.Rect(cx - r, cy - r, size, size)

                if i == 0:    # TL — arc curves into arena (down-right)
                    pygame.draw.arc(screen, color, arc_rect, 3 * math.pi / 2, 2 * math.pi, 2)
                    label_off = (cx + r // 2, cy + r // 2)
                elif i == 1:  # TR — arc curves into arena (down-left)
                    pygame.draw.arc(screen, color, arc_rect, math.pi, 3 * math.pi / 2, 2)
                    label_off = (cx - r // 2, cy + r // 2)
                elif i == 2:  # BR — arc curves into arena (up-left)
                    pygame.draw.arc(screen, color, arc_rect, math.pi / 2, math.pi, 2)
                    label_off = (cx - r // 2, cy - r // 2)
                else:         # BL — arc curves into arena (up-right)
                    pygame.draw.arc(screen, color, arc_rect, 0, math.pi / 2, 2)
                    label_off = (cx + r // 2, cy - r // 2)

                # Label inside the quarter circle
                label_surf = corner_font.render(text, True, color)
                label_rect = label_surf.get_rect(center=label_off)
                screen.blit(label_surf, label_rect)

        # ── Info overlay ─────────────────────────────────────────────────────
        if show_info:
            with corners_lock:
                known_count = sum(1 for c in corners if c["known"])
            lines = [
                f"Origin: ({arena_x}, {arena_y})  Size: {arena_w}x{arena_h}  Corners: {known_count}/4",
                f"Drag to move | Scroll to resize | Arrows: fine move | Shift+Arrows: fine size",
                f"P: print | F: fullscreen | I: toggle info | ESC: quit",
            ]
            for i, line in enumerate(lines):
                text = font.render(line, True, (100, 100, 100))
                screen.blit(text, (10, 10 + i * 20))

        pygame.display.flip()
        clock.tick(60)

    if mqtt_client:
        mqtt_client.loop_stop()
        mqtt_client.disconnect()
    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()
