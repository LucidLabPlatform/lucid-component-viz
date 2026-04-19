"""
Arena Projection Overlay - Interactive Calibration + Foraging Visualization

Data is received from the VizComponent via stdin (JSON lines).
Each line: {"topic": "<full topic>", "payload": {...}}

Controls:
  - Click and drag the arena to move it
  - Scroll wheel to resize the arena
  - Arrow keys for fine position adjustment (1px)
  - Shift + Arrow keys for fine size adjustment (1px)
  - P: Print current calibration + robot/puck state to console
  - F: Toggle fullscreen
  - I: Toggle info overlay
  - ESC: Quit
"""

import pygame
import sys
import os
import json
import math
import threading

# ── Display ──────────────────────────────────────────────────────────────────
WINDOW_WIDTH = 1920
WINDOW_HEIGHT = 1080

# ── Calibrated arena (764px = 3m, 534px = 2m) ───────────────────────────────
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
CORNER_RADIUS = 84
CORNER_FONT_SIZE = 14

RESIZE_STEP = 10
FINE_STEP = 1

# Puck/marker color mapping: 1=red, 2=green, 3=blue
COLOR_MAP = {
    1: (255, 50, 50),
    2: (50, 255, 50),
    3: (50, 100, 255),
}
PUCK_DRAW_RADIUS = 8
ROBOT_SIZE = 15
ROBOT_COLOR = (255, 255, 0)

# ── Corner state ─────────────────────────────────────────────────────────────
corners = [
    {"label": "TL", "known": False, "marker_id": None, "real_x": None, "real_y": None},
    {"label": "TR", "known": False, "marker_id": None, "real_x": None, "real_y": None},
    {"label": "BR", "known": False, "marker_id": None, "real_x": None, "real_y": None},
    {"label": "BL", "known": False, "marker_id": None, "real_x": None, "real_y": None},
]
corners_lock = threading.Lock()

# ── Puck state ───────────────────────────────────────────────────────────────
pucks = []
pucks_lock = threading.Lock()

# ── Robot pose state ─────────────────────────────────────────────────────────
robot_pose = {"position": None, "orientation": None}
robot_lock = threading.Lock()


# ── Coordinate mapping ───────────────────────────────────────────────────────
def map_to_screen(mx, my):
    """Convert map coordinates (meters) to screen pixels.
    Map origin (0,0) = bottom-left of arena."""
    px_per_m_x = arena_w / ARENA_METERS_W
    px_per_m_y = arena_h / ARENA_METERS_H
    sx = arena_x + mx * px_per_m_x
    sy = arena_y + arena_h - my * px_per_m_y  # Y inverted for pygame
    return int(sx), int(sy)


def optitrack_to_map(ox, oy, oz):
    """Convert OptiTrack coordinates to map coordinates.
    Adjust axes here if robot appears in the wrong place.
    Default: OptiTrack X → map X, OptiTrack Z → map Y."""
    return ox, oz


def quaternion_to_yaw(qx, qy, qz, qw):
    """Extract yaw angle (radians) from quaternion."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def corner_screen_positions():
    """Return (x, y) screen positions for each corner: TL, TR, BR, BL."""
    return [
        (arena_x,           arena_y),
        (arena_x + arena_w, arena_y),
        (arena_x + arena_w, arena_y + arena_h),
        (arena_x,           arena_y + arena_h),
    ]


def assign_corner(marker_id, x, y):
    """Assign an aruco marker to the nearest unoccupied arena corner."""
    with corners_lock:
        for c in corners:
            if c["known"] and c["marker_id"] == marker_id:
                c["real_x"] = x
                c["real_y"] = y
                return
        for c in corners:
            if not c["known"]:
                c["known"] = True
                c["marker_id"] = marker_id
                c["real_x"] = x
                c["real_y"] = y
                return


# ── Message handlers ─────────────────────────────────────────────────────────
def _handle_aruco_confirmed(payload):
    marker_id = payload.get("marker_id")
    x = payload.get("x")
    y = payload.get("y")
    if marker_id is not None and x is not None and y is not None:
        assign_corner(int(marker_id), float(x), float(y))
        print(f"[arena] Corner marker {marker_id} at ({x:.3f}, {y:.3f})")


def _handle_puck_registry(payload):
    value = payload.get("value", payload)
    puck_list = value.get("pucks", [])
    with pucks_lock:
        pucks.clear()
        pucks.extend(puck_list)


def _handle_aruco_registry(payload):
    value = payload.get("value", payload)
    marker_list = value.get("markers", [])
    for m in marker_list:
        assign_corner(int(m["marker_id"]), float(m["x"]), float(m["y"]))


def _handle_robot_pose(payload):
    value = payload.get("value", payload)
    pose = value.get("pose", {})
    pos = pose.get("position")
    ori = pose.get("orientation")
    if pos and ori:
        with robot_lock:
            robot_pose["position"] = pos
            robot_pose["orientation"] = ori


# ── Stdin reader ─────────────────────────────────────────────────────────────
def start_stdin_reader():
    """Read JSON lines from stdin and dispatch to state handlers.

    Each line written by VizComponent: {"topic": "<topic>", "payload": {...}}
    Runs in a daemon thread so it exits when the main process exits.
    """
    def _read():
        for raw in sys.stdin:
            raw = raw.strip()
            if not raw:
                continue
            try:
                msg = json.loads(raw)
                topic = msg.get("topic", "")
                payload = msg.get("payload", {})
                if topic.endswith("/telemetry/aruco_confirmed"):
                    _handle_aruco_confirmed(payload)
                elif topic.endswith("/telemetry/puck_registry"):
                    _handle_puck_registry(payload)
                elif topic.endswith("/telemetry/aruco_registry"):
                    _handle_aruco_registry(payload)
                elif topic.endswith("/telemetry/robot_pose"):
                    _handle_robot_pose(payload)
            except Exception as e:
                print(f"[arena] Bad stdin message: {e}")

    t = threading.Thread(target=_read, name="StdinReader", daemon=True)
    t.start()
    return t


# ── Main ─────────────────────────────────────────────────────────────────────
def main():
    global arena_w, arena_h, arena_x, arena_y

    start_stdin_reader()

    pygame.init()
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT), pygame.FULLSCREEN)
    pygame.display.set_caption("Arena Calibration")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("monospace", 16)
    corner_font = pygame.font.SysFont("monospace", CORNER_FONT_SIZE, bold=True)

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
                    with robot_lock:
                        if robot_pose["position"]:
                            p = robot_pose["position"]
                            o = robot_pose["orientation"]
                            print(f"  Robot pos: ({p['x']:.3f}, {p['y']:.3f}, {p['z']:.3f})")
                            print(f"  Robot ori: ({o['x']:.3f}, {o['y']:.3f}, {o['z']:.3f}, {o['w']:.3f})")
                        else:
                            print(f"  Robot: no data")
                    with pucks_lock:
                        print(f"  Pucks: {len(pucks)} total")
                        for pk in pucks:
                            print(f"    id={pk['id']} color={pk['color']} status={pk['status']} "
                                  f"({pk['x']:.3f}, {pk['y']:.3f})")
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
                    color = COLOR_MAP.get(c["marker_id"], (0, 200, 100))
                    text = f"#{c['marker_id']}"
                else:
                    color = CORNER_UNKNOWN_COLOR
                    text = "?"

                r = CORNER_RADIUS
                size = r * 2
                arc_rect = pygame.Rect(cx - r, cy - r, size, size)

                if i == 0:    # TL
                    pygame.draw.arc(screen, color, arc_rect, 3 * math.pi / 2, 2 * math.pi, 2)
                    label_off = (cx + r // 2, cy + r // 2)
                elif i == 1:  # TR
                    pygame.draw.arc(screen, color, arc_rect, math.pi, 3 * math.pi / 2, 2)
                    label_off = (cx - r // 2, cy + r // 2)
                elif i == 2:  # BR
                    pygame.draw.arc(screen, color, arc_rect, math.pi / 2, math.pi, 2)
                    label_off = (cx - r // 2, cy - r // 2)
                else:         # BL
                    pygame.draw.arc(screen, color, arc_rect, 0, math.pi / 2, 2)
                    label_off = (cx + r // 2, cy - r // 2)

                label_surf = corner_font.render(text, True, color)
                label_rect = label_surf.get_rect(center=label_off)
                screen.blit(label_surf, label_rect)

        # ── Pucks ────────────────────────────────────────────────────────────
        with pucks_lock:
            for p in pucks:
                color = COLOR_MAP.get(p.get("color"), (200, 200, 200))
                sx, sy = map_to_screen(p.get("x", 0), p.get("y", 0))
                pygame.draw.circle(screen, color, (sx, sy), PUCK_DRAW_RADIUS)
                if p.get("status") == 1:  # placed at home
                    pygame.draw.circle(screen, (255, 255, 255), (sx, sy), PUCK_DRAW_RADIUS + 3, 2)

        # ── Robot ────────────────────────────────────────────────────────────
        with robot_lock:
            pos = robot_pose["position"]
            ori = robot_pose["orientation"]
        if pos is not None and ori is not None:
            mx, my = optitrack_to_map(pos["x"], pos["y"], pos["z"])
            sx, sy = map_to_screen(mx, my)
            yaw = quaternion_to_yaw(ori["x"], ori["y"], ori["z"], ori["w"])
            s = ROBOT_SIZE
            tip = (sx + int(s * math.cos(yaw)), sy - int(s * math.sin(yaw)))
            left = (sx + int(s * 0.6 * math.cos(yaw + 2.5)), sy - int(s * 0.6 * math.sin(yaw + 2.5)))
            right = (sx + int(s * 0.6 * math.cos(yaw - 2.5)), sy - int(s * 0.6 * math.sin(yaw - 2.5)))
            pygame.draw.polygon(screen, ROBOT_COLOR, [tip, left, right])

        # ── Info overlay ─────────────────────────────────────────────────────
        if show_info:
            with corners_lock:
                known_count = sum(1 for c in corners if c["known"])
            with pucks_lock:
                puck_count = len(pucks)
                placed_count = sum(1 for p in pucks if p.get("status") == 1)
            with robot_lock:
                robot_str = "connected" if robot_pose["position"] else "no data"
            lines = [
                f"Origin: ({arena_x}, {arena_y})  Size: {arena_w}x{arena_h}  Corners: {known_count}/4",
                f"Pucks: {puck_count} ({placed_count} placed)  Robot: {robot_str}",
                f"Drag to move | Scroll to resize | Arrows: fine move | Shift+Arrows: fine size",
                f"P: print | F: fullscreen | I: toggle info | ESC: quit",
            ]
            for i, line in enumerate(lines):
                text = font.render(line, True, (100, 100, 100))
                screen.blit(text, (10, 10 + i * 20))

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()
