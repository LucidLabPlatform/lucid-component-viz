"""Unit tests for arena.config — frame math + env-var parsing.

Pure functions; no pygame, no MQTT, no subprocess.
"""
from __future__ import annotations

import math
import os

import pytest

from lucid_component_viz.arena import config as cfg


# ── quaternion_to_yaw ────────────────────────────────────────────────────────

def test_quaternion_to_yaw_identity():
    # Identity quaternion → yaw = 0
    assert cfg.quaternion_to_yaw(0.0, 0.0, 0.0, 1.0) == pytest.approx(0.0, abs=1e-9)


def test_quaternion_to_yaw_default_pose():
    # The previous hardcoded default quaternion should yield yaw such that
    # yaw + π ≈ 0.7553 (the previous ODOM_YAW_OFFSET).
    yaw = cfg.quaternion_to_yaw(
        -0.00494308490306139,
        -0.011712746694684029,
        -0.9295409917831421,
        0.3684997856616974,
    )
    assert (yaw + math.pi) == pytest.approx(0.7553, abs=1e-3)


# ── compute_bl_optitrack ─────────────────────────────────────────────────────

W = 2.1
H = 2.1


def test_compute_bl_anchor_bl():
    assert cfg.compute_bl_optitrack("BL", 3.0, 1.83, W, H) == (3.0, 1.83)


def test_compute_bl_anchor_br():
    assert cfg.compute_bl_optitrack("BR", 3.0, 1.83, W, H) == (3.0 + W, 1.83)


def test_compute_bl_anchor_tr():
    assert cfg.compute_bl_optitrack("TR", 3.0, 1.83, W, H) == (3.0 + W, 1.83 + H)


def test_compute_bl_anchor_tl():
    assert cfg.compute_bl_optitrack("TL", 3.0, 1.83, W, H) == (3.0, 1.83 + H)


def test_compute_bl_invalid_corner_raises():
    with pytest.raises(ValueError, match="anchor_corner"):
        cfg.compute_bl_optitrack("middle", 0.0, 0.0, W, H)


# ── optitrack_to_arena ───────────────────────────────────────────────────────

def test_optitrack_to_arena_at_bl():
    # OptiTrack point equal to BL OptiTrack → arena origin.
    assert cfg.optitrack_to_arena(3.0, 1.83, ox=3.0, oy=1.83) == (0.0, 0.0)


def test_optitrack_to_arena_axis_flip():
    # Going +X in OptiTrack → -X in arena.
    bl_x, bl_y = 3.0, 1.83
    ax, ay = cfg.optitrack_to_arena(bl_x, bl_y, ox=4.0, oy=1.83)
    assert ax == pytest.approx(-1.0)
    assert ay == pytest.approx(0.0)


# ── odom_to_arena ────────────────────────────────────────────────────────────

def test_odom_to_arena_robot_at_origin_collapses_to_pure_rotation():
    # When BL OptiTrack == goal, origins coincide; result = rotation by yaw + π.
    bl = (3.0, 1.83)
    goal = (3.0, 1.83)
    yaw = 0.5
    ox, oy = 1.0, 0.0
    ax, ay = cfg.odom_to_arena(bl[0], bl[1], goal[0], goal[1], yaw, ox, oy)
    expected_yaw = yaw + math.pi
    assert ax == pytest.approx(math.cos(expected_yaw), abs=1e-9)
    assert ay == pytest.approx(math.sin(expected_yaw), abs=1e-9)


def test_odom_to_arena_zero_pose_at_goal():
    # Robot at odom (0, 0) → arena = goal-in-arena offset.
    bl_x, bl_y = 5.0, 4.0
    goal_x, goal_y = 3.0, 1.83
    yaw = 0.5
    ax, ay = cfg.odom_to_arena(bl_x, bl_y, goal_x, goal_y, yaw, 0.0, 0.0)
    # goal in arena = optitrack_to_arena(BL, goal)
    expected = cfg.optitrack_to_arena(bl_x, bl_y, goal_x, goal_y)
    assert (ax, ay) == pytest.approx(expected)


# ── parse_env_config ─────────────────────────────────────────────────────────

ALL_ENV = {
    "LUCID_ARENA_ANCHOR_CORNER": "BL",
    "LUCID_ARENA_ANCHOR_X": "3.0",
    "LUCID_ARENA_ANCHOR_Y": "1.83",
    "LUCID_ARENA_GOAL_X": "3.0",
    "LUCID_ARENA_GOAL_Y": "1.83",
    "LUCID_ARENA_GOAL_QX": "0.0",
    "LUCID_ARENA_GOAL_QY": "0.0",
    "LUCID_ARENA_GOAL_QZ": "0.0",
    "LUCID_ARENA_GOAL_QW": "1.0",
}


def test_parse_env_config_full(monkeypatch):
    for k, v in ALL_ENV.items():
        monkeypatch.setenv(k, v)
    arena, robot = cfg.parse_env_config()
    assert arena.anchor_corner == "BL"
    assert arena.anchor_x == 3.0
    assert arena.anchor_y == 1.83
    assert robot.goal_x == 3.0
    assert robot.goal_qw == 1.0


@pytest.mark.parametrize("missing", list(ALL_ENV.keys()))
def test_parse_env_config_missing_var_raises(monkeypatch, missing):
    for k, v in ALL_ENV.items():
        monkeypatch.setenv(k, v)
    monkeypatch.delenv(missing, raising=False)
    with pytest.raises(cfg.ConfigError, match=missing):
        cfg.parse_env_config()


def test_parse_env_config_invalid_corner(monkeypatch):
    for k, v in ALL_ENV.items():
        monkeypatch.setenv(k, v)
    monkeypatch.setenv("LUCID_ARENA_ANCHOR_CORNER", "middle")
    with pytest.raises(cfg.ConfigError, match="anchor_corner"):
        cfg.parse_env_config()


def test_parse_env_config_non_numeric(monkeypatch):
    for k, v in ALL_ENV.items():
        monkeypatch.setenv(k, v)
    monkeypatch.setenv("LUCID_ARENA_GOAL_X", "not-a-float")
    with pytest.raises(cfg.ConfigError, match="LUCID_ARENA_GOAL_X"):
        cfg.parse_env_config()
