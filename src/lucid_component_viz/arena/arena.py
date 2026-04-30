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
  - C: Clear the odom path trail
  - ESC: Quit
"""

import pygame
import sys
import os
import json
import math
import threading
from collections import deque

# ── Display ──────────────────────────────────────────────────────────────────
WINDOW_WIDTH = 1920
WINDOW_HEIGHT = 1080

# ── Calibrated arena (534px = 2.1m x 2.1m) ─────────────────────────────────
arena_w = 534
arena_h = 534
arena_x = 595
arena_y = 0

ARENA_METERS_W = 2.1
ARENA_METERS_H = 2.1

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
    1: (255, 0, 0),
    2: (0, 255, 0),
    3: (0, 0, 255),
}
PUCK_DRAW_RADIUS = 8
ROBOT_SIZE = 15
ROBOT_COLOR_OPTITRACK = (255, 255, 0)   # yellow
ROBOT_COLOR_ODOM      = (0, 200, 255)   # cyan

# Lidar scan rendering
SCAN_POINT_RADIUS = 2
SCAN_POINT_COLOR = (255, 0, 255)        # magenta
SCAN_DOWNSAMPLE = 2                     # draw every Nth range (1 = all)

# Odom path trail (breadcrumbs of where the robot has been)
PATH_COLOR = (0, 120, 180)              # dim cyan, related to odom robot colour
PATH_THICKNESS = 2
PATH_MAX_POINTS = 2000                  # ~ several minutes of motion
PATH_MIN_STEP_M = 0.01                  # only append if robot moved >= 1 cm

# ── Coordinate frame calibration ─────────────────────────────────────────────
# Arena (0,0) is defined as the robot start pose, coincident with odom/map (0,0).
# The drawn arena rectangle's BL corner sits at arena (BL_X, BL_Y) — i.e. the robot
# starts 0.20 m right of the left wall and 0.22 m above the bottom wall.
BL_X = -0.20
BL_Y = -0.22

# OptiTrack world-frame position of arena (0,0) (= robot start)
OPTITRACK_ORIGIN_X = 2.992316246032715
OPTITRACK_ORIGIN_Y = 1.827694296836853

# Angle odom's +X axis makes with arena's +X axis at odom reset.
# Robot faces TR corner at start.
ODOM_YAW_OFFSET = 0.7552700000000000

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
seen_puck_ids: set[int] = set()

# ── ArUco state ──────────────────────────────────────────────────────────────
seen_marker_ids: set[int] = set()

# ── Robot pose state ─────────────────────────────────────────────────────────
robot_pose_optitrack = {"position": None, "orientation": None}
robot_pose_odom      = {"position": None, "orientation": None}
robot_lock = threading.Lock()

# ── Lidar scan state ─────────────────────────────────────────────────────────
# Stored in arena frame (already transformed at receive time using latest
# robot_pose_odom). Each entry is (ax, ay) in metres.
scan_points: list[tuple[float, float]] = []
scan_lock = threading.Lock()

# ── Odom path state ──────────────────────────────────────────────────────────
# Trail of robot positions in arena frame, derived from /odom.
robot_path_odom: deque[tuple[float, float]] = deque(maxlen=PATH_MAX_POINTS)
path_lock = threading.Lock()


# ── Coordinate mapping ───────────────────────────────────────────────────────
def map_to_screen(mx, my):
    """Convert arena coordinates (meters) to screen pixels.
    Arena (0,0) = robot start; the drawn rectangle's BL corner sits at arena (BL_X, BL_Y)."""
    px_per_m_x = arena_w / ARENA_METERS_W
    px_per_m_y = arena_h / ARENA_METERS_H
    sx = arena_x + (mx - BL_X) * px_per_m_x
    sy = arena_y + arena_h - (my - BL_Y) * px_per_m_y  # Y inverted for pygame
    return int(sx), int(sy)


def optitrack_to_arena(ox, oy):
    """OptiTrack world frame → arena frame (metres, origin = robot start).
    OptiTrack: left=+X, top→bottom=+Y. Arena: right=+X, bottom→top=+Y.
    Both axes are flipped: X negated, Y negated."""
    return OPTITRACK_ORIGIN_X - ox, OPTITRACK_ORIGIN_Y - oy


def odom_to_arena(ox, oy):
    """Odom frame → arena frame. Origins coincide (both at robot start), so pure rotation.
    Odom yaw=0 points in the ODOM_YAW_OFFSET direction in arena frame."""
    c = math.cos(ODOM_YAW_OFFSET)
    s = math.sin(ODOM_YAW_OFFSET)
    return ox * c - oy * s, ox * s + oy * c


def map_to_arena(mx, my):
    """ROS map frame → arena frame. Map shares origin/rotation with odom, so same transform."""
    c = math.cos(ODOM_YAW_OFFSET)
    s = math.sin(ODOM_YAW_OFFSET)
    return mx * c - my * s, mx * s + my * c


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


def corner_slot_for(x, y):
    """Slot index (0=TL, 1=TR, 2=BR, 3=BL) by quadrant relative to arena centre."""
    cx = BL_X + ARENA_METERS_W / 2
    cy = BL_Y + ARENA_METERS_H / 2
    right = x >= cx
    top   = y >= cy
    if top and not right:    return 0
    if top and right:        return 1
    if not top and right:    return 2
    return 3


def assign_corner(marker_id, x, y):
    """Place a marker into the corner slot whose quadrant contains (x, y).
    If the same marker_id is already in another slot, vacate it.
    If the target slot already holds a different marker, the newer marker wins."""
    slot = corner_slot_for(x, y)
    with corners_lock:
        for i, c in enumerate(corners):
            if c["known"] and c["marker_id"] == marker_id and i != slot:
                c["known"] = False
                c["marker_id"] = None
                c["real_x"] = None
                c["real_y"] = None
        c = corners[slot]
        if c["known"] and c["marker_id"] != marker_id:
            print(f"[arena] WARN: corner {c['label']} contested between markers "
                  f"{c['marker_id']} and {marker_id}; keeping newer ({marker_id})")
        c["known"] = True
        c["marker_id"] = marker_id
        c["real_x"] = x
        c["real_y"] = y


# ── Message handlers ─────────────────────────────────────────────────────────
def _handle_aruco_registry(payload):
    """ArucoRegistry: markers[] of {id, marker_id, status, x, y, z}.

    Calls assign_corner for every marker (idempotent — known marker_ids just
    update position). Logs only the first time we see each marker_id.
    """
    value = payload.get("value", payload)
    markers = value.get("markers", []) or []
    for m in markers:
        marker_id = m.get("marker_id")
        x = m.get("x")
        y = m.get("y")
        if marker_id is None or x is None or y is None:
            continue
        ax, ay = map_to_arena(float(x), float(y))
        mid = int(marker_id)
        assign_corner(mid, ax, ay)
        if mid not in seen_marker_ids:
            seen_marker_ids.add(mid)
            print(f"[arena] NEW corner marker {mid} at map ({x:.3f}, {y:.3f}) → arena ({ax:.3f}, {ay:.3f})")


def _handle_puck_registry(payload):
    """PuckRegistry: pucks[] of {id, color, status, x, y, z}.

    Replaces puck list wholesale (semantic: registry IS the full list).
    Logs newly-seen puck ids and any transition to/from empty so we can see
    when the upstream publisher transiently clears the registry.
    """
    value = payload.get("value", payload)
    puck_list = value.get("pucks", []) or []
    new_ids = []
    for p in puck_list:
        pid = p.get("id")
        if pid is None:
            continue
        pid = int(pid)
        if pid not in seen_puck_ids:
            seen_puck_ids.add(pid)
            new_ids.append((pid, p.get("color"), p.get("x"), p.get("y")))
    with pucks_lock:
        prev_count = len(pucks)
        pucks.clear()
        pucks.extend(puck_list)
        new_count = len(pucks)
    if prev_count != new_count and (prev_count == 0 or new_count == 0):
        print(f"[arena] puck_registry: {prev_count} -> {new_count}")
    for pid, color, px, py in new_ids:
        try:
            print(f"[arena] NEW puck id={pid} color={color} at ({float(px):.3f}, {float(py):.3f})")
        except (TypeError, ValueError):
            print(f"[arena] NEW puck id={pid} color={color}")


def _handle_robot_pose_optitrack(payload):
    value = payload.get("value", payload)
    pose = value.get("pose", {})
    pos = pose.get("position")
    ori = pose.get("orientation")
    if pos and ori:
        with robot_lock:
            robot_pose_optitrack["position"] = pos
            robot_pose_optitrack["orientation"] = ori


def _handle_robot_pose_odom(payload):
    value = payload.get("value", payload)
    pose = value.get("pose", {})
    pos = pose.get("position")
    ori = pose.get("orientation")
    if not (pos and ori):
        return
    with robot_lock:
        robot_pose_odom["position"] = pos
        robot_pose_odom["orientation"] = ori

    # Append the new position to the path trail (in arena frame).
    try:
        ax, ay = odom_to_arena(float(pos["x"]), float(pos["y"]))
    except (TypeError, ValueError, KeyError):
        return
    with path_lock:
        if not robot_path_odom:
            robot_path_odom.append((ax, ay))
        else:
            lx, ly = robot_path_odom[-1]
            if math.hypot(ax - lx, ay - ly) >= PATH_MIN_STEP_M:
                robot_path_odom.append((ax, ay))


_scan_msg_count = 0


def _handle_scan(payload):
    """sensor_msgs/LaserScan → list of arena-frame points.

    Transforms each valid range to arena frame using the latest odom pose
    (laser frame is approximated as base_link — laser offset is small on the
    ROSbot). If we have no robot pose yet, drops the scan.
    """
    global _scan_msg_count
    value = payload.get("value", payload)
    angle_min = value.get("angle_min")
    angle_inc = value.get("angle_increment")
    range_min = value.get("range_min", 0.0) or 0.0
    range_max = value.get("range_max", 0.0) or 0.0
    ranges = value.get("ranges") or []
    if angle_min is None or angle_inc is None or not ranges:
        return

    with robot_lock:
        pos = robot_pose_odom["position"]
        ori = robot_pose_odom["orientation"]

    if not pos or not ori:
        # No pose yet — can't place the scan in the arena. Drop quietly.
        return

    rx_odom = float(pos["x"])
    ry_odom = float(pos["y"])
    yaw_odom = quaternion_to_yaw(ori["x"], ori["y"], ori["z"], ori["w"])

    # Robot heading in arena frame: rotate odom yaw by ODOM_YAW_OFFSET.
    yaw_arena = yaw_odom + ODOM_YAW_OFFSET

    # Robot position in arena frame.
    rx_arena, ry_arena = odom_to_arena(rx_odom, ry_odom)

    points: list[tuple[float, float]] = []
    for i in range(0, len(ranges), SCAN_DOWNSAMPLE):
        r = ranges[i]
        if r is None:
            continue
        try:
            r = float(r)
        except (TypeError, ValueError):
            continue
        if not math.isfinite(r):
            continue
        if r < range_min or (range_max > 0.0 and r > range_max):
            continue
        theta = angle_min + i * angle_inc
        # Laser-frame point rotated into arena frame and offset by robot pos.
        a = theta + yaw_arena
        ax = rx_arena + r * math.cos(a)
        ay = ry_arena + r * math.sin(a)
        points.append((ax, ay))

    with scan_lock:
        scan_points.clear()
        scan_points.extend(points)

    _scan_msg_count += 1
    if _scan_msg_count == 1 or _scan_msg_count % 30 == 0:
        print(f"[arena] scan #{_scan_msg_count}: {len(ranges)} raw → {len(points)} drawn")


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
                if topic.endswith("/aruco_registry"):
                    _handle_aruco_registry(payload)
                elif topic.endswith("/puck_registry"):
                    _handle_puck_registry(payload)
                elif topic.endswith("/robot_pose_optitrack"):
                    _handle_robot_pose_optitrack(payload)
                elif topic.endswith("/robot_pose_odom"):
                    _handle_robot_pose_odom(payload)
                elif topic.endswith("/scan"):
                    _handle_scan(payload)
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
                        if robot_pose_optitrack["position"]:
                            p = robot_pose_optitrack["position"]
                            o = robot_pose_optitrack["orientation"]
                            ax, ay = optitrack_to_arena(p["x"], p["y"])
                            print(f"  Robot (OptiTrack) raw: ({p['x']:.3f}, {p['y']:.3f}) → arena ({ax:.3f}, {ay:.3f})")
                            print(f"  Robot (OptiTrack) ori: ({o['x']:.3f}, {o['y']:.3f}, {o['z']:.3f}, {o['w']:.3f})")
                        else:
                            print(f"  Robot (OptiTrack): no data")
                        if robot_pose_odom["position"]:
                            p = robot_pose_odom["position"]
                            o = robot_pose_odom["orientation"]
                            ax, ay = odom_to_arena(p["x"], p["y"])
                            print(f"  Robot (Odom) raw: ({p['x']:.3f}, {p['y']:.3f}) → arena ({ax:.3f}, {ay:.3f})")
                            print(f"  Robot (Odom) ori: ({o['x']:.3f}, {o['y']:.3f}, {o['z']:.3f}, {o['w']:.3f})")
                        else:
                            print(f"  Robot (Odom): no data")
                    with pucks_lock:
                        print(f"  Pucks: {len(pucks)} total")
                        for pk in pucks:
                            print(f"    id={pk['id']} color={pk['color']} status={pk['status']} "
                                  f"({pk['x']:.3f}, {pk['y']:.3f})")
                    with scan_lock:
                        print(f"  Scan points (drawn): {len(scan_points)}")
                    with path_lock:
                        print(f"  Odom path: {len(robot_path_odom)} points")
                    print(f"=========================\n")

                elif event.key == pygame.K_f:
                    fullscreen = not fullscreen
                    if fullscreen:
                        screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT), pygame.FULLSCREEN)
                    else:
                        screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))

                elif event.key == pygame.K_i:
                    show_info = not show_info

                elif event.key == pygame.K_c:
                    with path_lock:
                        robot_path_odom.clear()
                    print("[arena] Cleared odom path trail")

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

        # ── Arena origin TF axes ─────────────────────────────────────────────
        # Draw +X (red, right) and +Y (green, up) axes at arena (0,0) = BL corner
        _tf_len = 40  # pixels
        _ox, _oy = map_to_screen(0, 0)
        pygame.draw.line(screen, (255, 50, 50),  (_ox, _oy), (_ox + _tf_len, _oy), 2)  # +X right
        pygame.draw.line(screen, (50, 255, 50),  (_ox, _oy), (_ox, _oy - _tf_len), 2)  # +Y up
        _tf_font = pygame.font.SysFont("monospace", 11)
        screen.blit(_tf_font.render("+X", True, (255, 50, 50)),  (_ox + _tf_len + 2, _oy - 8))
        screen.blit(_tf_font.render("+Y", True, (50, 255, 50)),  (_ox + 2, _oy - _tf_len - 12))
        screen.blit(_tf_font.render("0", True, (180, 180, 180)), (_ox + 3, _oy + 3))

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

        # ── Odom path trail ──────────────────────────────────────────────────
        # Drawn first so pucks / scan / robot render on top of it.
        with path_lock:
            path_snapshot = list(robot_path_odom)
        if len(path_snapshot) >= 2:
            screen_path = [map_to_screen(ax, ay) for ax, ay in path_snapshot]
            pygame.draw.lines(screen, PATH_COLOR, False, screen_path, PATH_THICKNESS)

        # ── Pucks ────────────────────────────────────────────────────────────
        with pucks_lock:
            for p in pucks:
                color = COLOR_MAP.get(p.get("color"), (200, 200, 200))
                ax, ay = map_to_arena(p.get("x", 0), p.get("y", 0))
                sx, sy = map_to_screen(ax, ay)
                pygame.draw.circle(screen, color, (sx, sy), PUCK_DRAW_RADIUS)
                if p.get("status") == 1:  # placed at home
                    pygame.draw.circle(screen, (255, 255, 255), (sx, sy), PUCK_DRAW_RADIUS + 3, 2)

        # ── Lidar scan ───────────────────────────────────────────────────────
        # Drawn under the robot so the robot icon stays readable.
        with scan_lock:
            for ax, ay in scan_points:
                sx, sy = map_to_screen(ax, ay)
                pygame.draw.circle(screen, SCAN_POINT_COLOR, (sx, sy), SCAN_POINT_RADIUS)

        # ── Robot ────────────────────────────────────────────────────────────
        with robot_lock:
            ot_pos = robot_pose_optitrack["position"]
            ot_ori = robot_pose_optitrack["orientation"]
            od_pos = robot_pose_odom["position"]
            od_ori = robot_pose_odom["orientation"]

        def draw_robot(ax, ay, ori, color, yaw_offset=0.0, negate_yaw=False):
            sx, sy = map_to_screen(ax, ay)
            yaw = quaternion_to_yaw(ori["x"], ori["y"], ori["z"], ori["w"])
            if negate_yaw:
                yaw = -yaw
            yaw += yaw_offset
            s = ROBOT_SIZE
            tip   = (sx + int(s * math.cos(yaw)),             sy - int(s * math.sin(yaw)))
            left  = (sx + int(s * 0.6 * math.cos(yaw + 2.5)), sy - int(s * 0.6 * math.sin(yaw + 2.5)))
            right = (sx + int(s * 0.6 * math.cos(yaw - 2.5)), sy - int(s * 0.6 * math.sin(yaw - 2.5)))
            pygame.draw.polygon(screen, color, [tip, left, right])

        if ot_pos and ot_ori:
            ax, ay = optitrack_to_arena(ot_pos["x"], ot_pos["y"])
            draw_robot(ax, ay, ot_ori, ROBOT_COLOR_OPTITRACK, yaw_offset=0.0, negate_yaw=True)
        if od_pos and od_ori:
            ax, ay = odom_to_arena(od_pos["x"], od_pos["y"])
            draw_robot(ax, ay, od_ori, ROBOT_COLOR_ODOM, yaw_offset=ODOM_YAW_OFFSET)

        # ── Info overlay ─────────────────────────────────────────────────────
        if show_info:
            with corners_lock:
                known_count = sum(1 for c in corners if c["known"])
            with pucks_lock:
                puck_count = len(pucks)
                placed_count = sum(1 for p in pucks if p.get("status") == 1)
            with robot_lock:
                ot_str = "ok" if robot_pose_optitrack["position"] else "--"
                od_str = "ok" if robot_pose_odom["position"] else "--"
            with scan_lock:
                scan_count = len(scan_points)
            with path_lock:
                path_count = len(robot_path_odom)
            lines = [
                f"Origin: ({arena_x}, {arena_y})  Size: {arena_w}x{arena_h}  Corners: {known_count}/4",
                f"Pucks: {puck_count} ({placed_count} placed)  Scan: {scan_count}  Path: {path_count}  OptiTrack: {ot_str}  Odom: {od_str}",
                f"Drag to move | Scroll to resize | Arrows: fine move | Shift+Arrows: fine size",
                f"P: print | F: fullscreen | I: toggle info | C: clear path | ESC: quit",
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
