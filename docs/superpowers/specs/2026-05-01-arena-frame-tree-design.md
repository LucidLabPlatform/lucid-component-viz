# Arena frame tree: decouple arena placement from robot start

**Status:** approved
**Author:** Farah
**Date:** 2026-05-01

## Motivation

The arena overlay (`lucid_component_viz/arena/arena.py`) currently couples the
arena's coordinate origin to the robot's reset-to-start pose. Module-level
constants `OPTITRACK_ORIGIN_X/Y`, `ODOM_YAW_OFFSET`, and `BL_X/Y` must be
hand-edited in source whenever the physical arena is moved.

Goals:

1. Let the arena be repositioned by editing config, not source.
2. Decouple arena placement from the robot's starting pose so each can be set
   independently.
3. Reuse the `goal_*` template params (already used by `reset_to_start`) as the
   single source of truth for the robot's initial OptiTrack pose.
4. Fail loud with a clear error if the values aren't provided — no silent
   fallback to stale hardcoded calibration.

## Frame structure

```
world (OptiTrack)              ← fixed, never moves
├── arena                      ← anchored to world via one corner
└── robot tree (= odom frame)  ← anchored to world via robot goal pose
    ├── pucks
    ├── aruco markers
    └── laser scan
```

### World

OptiTrack world frame. Treated as fixed; no configuration.

### Arena frame

Child of world.

- **Position in world:** set via one anchor corner. The user picks
  `arena_anchor_corner` ∈ `{TL, TR, BR, BL}` and provides its
  `(optitrack_x, optitrack_y)`.
- **Orientation in world (fixed):** arena `+X` = OptiTrack `-X`, arena `+Y` =
  OptiTrack `-Y`. Equivalent to a 180° rotation, matching the existing
  `optitrack_to_arena` convention.
- **Internal convention:** arena `(0, 0)` = BL corner of the rectangle, always,
  regardless of which corner the user anchors. The other three corners are
  derived from arena size (`ARENA_METERS_W` × `ARENA_METERS_H`) and the anchor.

#### Deriving BL OptiTrack position from anchor

Given anchor corner OptiTrack position `(ax, ay)` and arena size `(W, H)`:

| anchor | BL OptiTrack X | BL OptiTrack Y |
|--------|----------------|----------------|
| BL     | `ax`           | `ay`           |
| BR     | `ax + W`       | `ay`           |
| TR     | `ax + W`       | `ay + H`       |
| TL     | `ax`           | `ay + H`       |

(Each row computed by translating from the anchor toward BL in arena frame and
mapping back through the fixed `arena_x = -optitrack_x` axis flip.)

The `optitrack_to_arena` transform becomes:

```python
def optitrack_to_arena(ox, oy):
    return BL_OPTITRACK_X - ox, BL_OPTITRACK_Y - oy
```

### Robot tree (= odom frame)

Child of world. Sibling of arena.

- **Position in world:** `(goal_x, goal_y)`.
- **Orientation in world:** yaw derived from quaternion
  `(goal_qx, goal_qy, goal_qz, goal_qw)` via the existing `quaternion_to_yaw`
  helper.
- **Robot's pose inside this tree:** read directly from `/odom`. Odom is reset
  to 0 at experiment start with `+X` = robot's heading at start, which matches
  the robot tree origin/orientation by construction.

Pucks, ArUco markers, and the lidar scan all arrive in the robot tree (in code
today they come through ROS map frame, which equals odom for our pipeline).

## Rendering chains

| Source                               | Transform chain               |
|--------------------------------------|-------------------------------|
| Arena rectangle                      | drawn directly in arena       |
| Robot's OptiTrack-pose telemetry     | world → arena                 |
| Robot's odom pose                    | robot tree → world → arena    |
| Pucks, ArUco markers                 | robot tree → world → arena    |
| Lidar scan                           | robot tree → world → arena    |

`world → arena`:

```
arena_x = BL_OPTITRACK_X - world_x
arena_y = BL_OPTITRACK_Y - world_y
```

`robot tree → world` is a 2D rigid transform with translation
`(goal_x, goal_y)` and rotation = `goal_yaw`.

## Configuration

Nine fields, in two independent groups. Each group is resolved
**all-or-nothing**: if any field in a group is missing, that layer is skipped
for the whole group.

### Arena group (3 fields)

| Field                 | Type                              | Meaning                                  |
|-----------------------|-----------------------------------|------------------------------------------|
| `arena_anchor_corner` | `str` ∈ `{"TL","TR","BR","BL"}`   | Which corner is the anchor               |
| `arena_anchor_x`      | `float`                           | Anchor's OptiTrack X                     |
| `arena_anchor_y`      | `float`                           | Anchor's OptiTrack Y                     |

### Robot group (6 fields)

| Field      | Type    | Meaning                          |
|------------|---------|----------------------------------|
| `goal_x`   | `float` | Robot start X (OptiTrack)        |
| `goal_y`   | `float` | Robot start Y (OptiTrack)        |
| `goal_qx`  | `float` | Robot start orientation qx       |
| `goal_qy`  | `float` | Robot start orientation qy       |
| `goal_qz`  | `float` | Robot start orientation qz       |
| `goal_qw`  | `float` | Robot start orientation qw       |

Same names as the existing `foraging-reset` / `foraging-experiment` template
params.

### Layered resolution

Per group, in order:

1. `start_arena` payload — one-shot override for this run.
2. viz component cfg — persistent, settable via `cfg/set`, retained on MQTT.
3. **Fail** if neither layer provides all fields in the group. The
   `start_arena` command returns
   `evt/start_arena/result` with `ok=false` and an error naming which group
   and which fields are missing.

The two groups are resolved independently: it is valid for the arena group to
come from cfg while the robot group comes from the `start_arena` payload.

## Wiring

### `arena.py`

- Read 9 env vars at startup:
  `LUCID_ARENA_ANCHOR_CORNER`, `LUCID_ARENA_ANCHOR_X`, `LUCID_ARENA_ANCHOR_Y`,
  `LUCID_ARENA_GOAL_X`, `LUCID_ARENA_GOAL_Y`,
  `LUCID_ARENA_GOAL_QX/QY/QZ/QW`.
- Validate: all 9 present, anchor corner is one of the 4 labels, all numerics
  parse as floats. On any failure → log a clear error naming the missing or
  invalid var and `sys.exit(1)`.
- Compute `BL_OPTITRACK_X/Y` from anchor corner + size.
- Compute `GOAL_YAW = quaternion_to_yaw(goal_qx, goal_qy, goal_qz, goal_qw)`.
- Replace transforms:
  - `optitrack_to_arena(ox, oy)` returns `(BL_OPTITRACK_X - ox,
    BL_OPTITRACK_Y - oy)`.
  - `odom_to_arena(ox, oy)` becomes the composition of `odom → world` (rotate
    by `goal_yaw`, translate by `(goal_x, goal_y)`) followed by
    `world → arena` (the 180° flip via `optitrack_to_arena`). In code:

    ```python
    def odom_to_arena(ox, oy):
        c, s = math.cos(GOAL_YAW), math.sin(GOAL_YAW)
        world_x = GOAL_X + ox * c - oy * s
        world_y = GOAL_Y + ox * s + oy * c
        return optitrack_to_arena(world_x, world_y)
    ```

    Note: the old `odom_to_arena` was a pure rotation by `goal_yaw + π`
    because the arena origin coincided with the robot start. In the new model
    the two origins are independent, so we need both the rotation and a
    translation. When `(BL_OPTITRACK_X, BL_OPTITRACK_Y) == (goal_x, goal_y)`
    the new formula collapses to the old one (rotation by `goal_yaw + π`
    around the origin), confirming consistency.

  - `map_to_arena` becomes an alias of `odom_to_arena` (map shares
    origin/orientation with odom in our pipeline).
- `map_to_screen` uses arena `(0, 0)` = BL convention; the existing offset
  math drops the `BL_X/Y` constants and treats arena `(0, 0)` directly as the
  drawn rectangle's BL corner in screen pixels.
- Remove module-level `OPTITRACK_ORIGIN_X/Y`, `ODOM_YAW_OFFSET`, `BL_X/Y`.

### viz component (`component.py`)

- Add 9 `Optional` cfg fields with the names above. Default `None`.
- Extend `get_cfg_payload()` to return them.
- Extend `on_cmd_cfg_set` to parse and persist them. `arena_anchor_corner` is
  validated against `{"TL","TR","BR","BL"}`; numerics coerced to `float`.
- `on_cmd_start_arena` parses optional 9 fields from payload.
- `_start_arena(overrides)` resolves each group:
  - arena group: payload (if all 3 present) → cfg (if all 3 present) → fail.
  - robot group: payload (if all 6 present) → cfg (if all 6 present) → fail.
  - On failure, log a clear error and return `False`. Caller publishes
    `evt/start_arena/result` with `ok=false` and the error string.
- Sets the 9 env vars on the subprocess `env` before `Popen`.
- Update `schema()` to include the 9 fields under `cfg` and under
  `cmd/start-arena`.

### Templates

Param threading top-down (each layer adds the 9 fields to its `parameters`
block and threads them via `template_params`):

- `foraging-experiment.json` already has `goal_*`. Add `arena_anchor_corner`,
  `arena_anchor_x`, `arena_anchor_y`. Thread all 9 into `foraging-setup`.
- `foraging-setup.json`: add the 9 params; thread into `arena-startup`.
- `arena-startup.json`: add the 9 params; thread into `arena-td-launch`.
- `arena-td-launch.json`: add the 9 params; pass in the `start_arena` step's
  `params`.
- `foraging-reset.json`: no change (already has `goal_*` for the
  reset-to-start launch; doesn't talk to viz).

Defaults for the 9 template params should match the values currently in
arena.py source so the existing experiments keep working unchanged.

## Tests

### arena.py

- All 9 env vars present and valid → arena starts; `BL_OPTITRACK_X/Y`,
  `GOAL_YAW` match expected values for each anchor choice.
- Any env var missing → exits non-zero with an error naming the missing var.
- Invalid `LUCID_ARENA_ANCHOR_CORNER` (e.g. `"middle"`) → exits non-zero.
- Non-float numeric env var → exits non-zero.
- Math: anchor BL with size 2.1×2.1 at OptiTrack (3.0, 1.83) yields BL
  OptiTrack (3.0, 1.83). Anchor TR at OptiTrack (3.0, 1.83) yields BL
  OptiTrack (5.1, 3.93). (Spot-check the table.)

### viz component

- cfg group all-or-nothing: cfg with `arena_anchor_x` set but not
  `arena_anchor_y` → arena group treated as missing.
- payload > cfg per group: payload with all 3 arena fields and cfg with all 3
  arena fields → payload values used.
- Cross-group independence: payload has full robot group, cfg has full arena
  group → starts successfully using payload for robot group and cfg for arena
  group.
- Failure: neither layer complete → `start_arena` returns `ok=false` with a
  clear error string naming the incomplete group(s) and missing fields.
- `_start_arena()` sets the 9 env vars on the subprocess.
- Schema includes the 9 fields under `cfg` and `cmd/start-arena`.

### Templates

- Each template's `parameters` block lists the 9 fields with the documented
  defaults.
- Each template threads the 9 fields into its child template/command via
  `template_params` or `params`.

## Out of scope

- Live recalibration of a running arena. Recalibration requires
  `start_arena` (re)issue. Reasoned acceptable in earlier discussion.
- Changing the arena's orientation: arena `+X` / `+Y` direction in OptiTrack
  is fixed at `-X` / `-Y`. If the arena is rotated 90° in the lab, that's a
  separate change.
- Pixel-space projector calibration (`arena_x/y/w/h` keyboard tuning) is
  unchanged.
- ArUco corner markers continue to drive the visual corner indicators (they
  do not feed into the arena pose itself).
