"""Arena configuration: env-var parsing + frame transforms.

Pure functions. No pygame, MQTT, or subprocess dependencies — kept here so the
math is unit-testable independently of the pygame overlay.

Frame structure:
  world (OptiTrack)              ← fixed
  ├── arena                      ← anchored to world via one corner (axes
  │                                 fixed: arena +X = OptiTrack -X,
  │                                        arena +Y = OptiTrack -Y)
  └── robot tree (= odom frame)  ← anchored to world via robot goal pose
                                    (translation = (goal_x, goal_y),
                                     rotation = yaw of goal quaternion)

See docs/superpowers/specs/2026-05-01-arena-frame-tree-design.md.
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass


VALID_CORNERS = ("TL", "TR", "BR", "BL")


class ConfigError(Exception):
    """Raised when env-var configuration is missing or invalid."""


@dataclass(frozen=True)
class ArenaConfig:
    anchor_corner: str  # one of VALID_CORNERS
    anchor_x: float     # OptiTrack X of the anchor corner
    anchor_y: float     # OptiTrack Y of the anchor corner


@dataclass(frozen=True)
class RobotConfig:
    goal_x: float
    goal_y: float
    goal_qx: float
    goal_qy: float
    goal_qz: float
    goal_qw: float

    @property
    def goal_yaw(self) -> float:
        return quaternion_to_yaw(self.goal_qx, self.goal_qy, self.goal_qz, self.goal_qw)


# ── Frame math ───────────────────────────────────────────────────────────────

def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Yaw (radians) extracted from a quaternion."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def compute_bl_optitrack(
    anchor_corner: str,
    anchor_x: float,
    anchor_y: float,
    arena_w_m: float,
    arena_h_m: float,
) -> tuple[float, float]:
    """OptiTrack (x, y) of the arena's BL corner.

    Derived from whichever corner the user anchored. Axes fixed at arena +X =
    OptiTrack -X, arena +Y = OptiTrack -Y, so going BL→<other corner> in arena
    flips sign in OptiTrack.
    """
    if anchor_corner == "BL":
        return (anchor_x, anchor_y)
    if anchor_corner == "BR":
        return (anchor_x + arena_w_m, anchor_y)
    if anchor_corner == "TR":
        return (anchor_x + arena_w_m, anchor_y + arena_h_m)
    if anchor_corner == "TL":
        return (anchor_x, anchor_y + arena_h_m)
    raise ValueError(
        f"anchor_corner must be one of {VALID_CORNERS}, got {anchor_corner!r}"
    )


def optitrack_to_arena(
    bl_x: float, bl_y: float, ox: float, oy: float
) -> tuple[float, float]:
    """OptiTrack (ox, oy) → arena coordinates.

    Arena (0, 0) sits at the BL corner of the rectangle. Both axes flipped.
    """
    return (bl_x - ox, bl_y - oy)


def odom_to_arena(
    bl_x: float,
    bl_y: float,
    goal_x: float,
    goal_y: float,
    goal_yaw: float,
    ox: float,
    oy: float,
) -> tuple[float, float]:
    """Robot-tree (odom) point → arena coordinates.

    Composition: odom → world (rotate by goal_yaw, translate by goal pose),
    then world → arena (axis-flipped translation via optitrack_to_arena).
    """
    c = math.cos(goal_yaw)
    s = math.sin(goal_yaw)
    world_x = goal_x + ox * c - oy * s
    world_y = goal_y + ox * s + oy * c
    return optitrack_to_arena(bl_x, bl_y, world_x, world_y)


# ── Env-var parsing ──────────────────────────────────────────────────────────

ENV_ANCHOR_CORNER = "LUCID_ARENA_ANCHOR_CORNER"
ENV_ANCHOR_X = "LUCID_ARENA_ANCHOR_X"
ENV_ANCHOR_Y = "LUCID_ARENA_ANCHOR_Y"
ENV_GOAL_X = "LUCID_ARENA_GOAL_X"
ENV_GOAL_Y = "LUCID_ARENA_GOAL_Y"
ENV_GOAL_QX = "LUCID_ARENA_GOAL_QX"
ENV_GOAL_QY = "LUCID_ARENA_GOAL_QY"
ENV_GOAL_QZ = "LUCID_ARENA_GOAL_QZ"
ENV_GOAL_QW = "LUCID_ARENA_GOAL_QW"

ALL_ENV_VARS = (
    ENV_ANCHOR_CORNER, ENV_ANCHOR_X, ENV_ANCHOR_Y,
    ENV_GOAL_X, ENV_GOAL_Y,
    ENV_GOAL_QX, ENV_GOAL_QY, ENV_GOAL_QZ, ENV_GOAL_QW,
)


def _require_env(name: str) -> str:
    val = os.environ.get(name)
    if val is None or val == "":
        raise ConfigError(f"missing required env var {name}")
    return val


def _require_float(name: str) -> float:
    raw = _require_env(name)
    try:
        return float(raw)
    except ValueError as e:
        raise ConfigError(f"{name}={raw!r} is not a valid float") from e


def parse_env_config() -> tuple[ArenaConfig, RobotConfig]:
    """Read the 9 LUCID_ARENA_* env vars and validate.

    Raises ConfigError on any missing or invalid value, naming the offending
    variable in the message. All-or-nothing: any failure raises before
    anything is returned.
    """
    corner = _require_env(ENV_ANCHOR_CORNER)
    if corner not in VALID_CORNERS:
        raise ConfigError(
            f"{ENV_ANCHOR_CORNER}={corner!r} must be one of {VALID_CORNERS} "
            f"(anchor_corner)"
        )
    arena = ArenaConfig(
        anchor_corner=corner,
        anchor_x=_require_float(ENV_ANCHOR_X),
        anchor_y=_require_float(ENV_ANCHOR_Y),
    )
    robot = RobotConfig(
        goal_x=_require_float(ENV_GOAL_X),
        goal_y=_require_float(ENV_GOAL_Y),
        goal_qx=_require_float(ENV_GOAL_QX),
        goal_qy=_require_float(ENV_GOAL_QY),
        goal_qz=_require_float(ENV_GOAL_QZ),
        goal_qw=_require_float(ENV_GOAL_QW),
    )
    return arena, robot
