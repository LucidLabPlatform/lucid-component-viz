"""Contract and lifecycle tests for VizComponent."""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Any
from unittest.mock import MagicMock, patch

import pytest

from lucid_component_base import ComponentContext
from lucid_component_viz.component import VizComponent


# ── Test helpers ─────────────────────────────────────────────────────────────

@dataclass
class RecordingMqtt:
    """Captures all MQTT publish calls."""
    messages: list[dict[str, Any]] = field(default_factory=list)

    def publish(self, topic: str, payload: Any, *, qos: int = 0, retain: bool = False) -> MagicMock:
        self.messages.append({"topic": topic, "payload": payload, "qos": qos, "retain": retain})
        info = MagicMock()
        info.rc = 0
        return info

    def subscribe(self, topic: str, callback: Any, *, qos: int = 0) -> None:
        pass

    def unsubscribe(self, topic: str) -> None:
        pass

    def topics(self) -> list[str]:
        return [m["topic"] for m in self.messages]

    def by_suffix(self, suffix: str) -> list[dict[str, Any]]:
        return [m for m in self.messages if m["topic"].endswith(suffix)]


@pytest.fixture
def mqtt():
    return RecordingMqtt()


@pytest.fixture
def ctx(mqtt):
    return ComponentContext(
        agent_id="test-agent",
        base_topic="lucid/agents/test-agent",
        component_id="viz",
        mqtt=mqtt,
        config={"touchdesigner_app": "/Applications/TouchDesigner.app"},
    )


@pytest.fixture
def component(ctx):
    return VizComponent(ctx)


# ── Identity tests ───────────────────────────────────────────────────────────

def test_component_id(component):
    assert component.component_id == "viz"


def test_capabilities(component):
    caps = component.capabilities()
    assert "start-arena" in caps
    assert "stop-arena" in caps
    assert "start-touchdesigner" in caps
    assert "stop-touchdesigner" in caps
    assert "restart" in caps


def test_metadata_includes_capabilities(component):
    meta = component.metadata()
    assert "capabilities" in meta
    assert "start-arena" in meta["capabilities"]


# ── State tests ──────────────────────────────────────────────────────────────

@patch("lucid_component_viz.component._find_pid", return_value=None)
def test_state_payload_structure(mock_pgrep, component):
    state = component.get_state_payload()
    assert "arena" in state
    assert "touchdesigner" in state
    assert "running" in state["arena"]
    assert "pid" in state["arena"]
    assert "running" in state["touchdesigner"]
    assert "pid" in state["touchdesigner"]


@patch("lucid_component_viz.component._find_pid", return_value=None)
def test_state_arena_not_running_initially(mock_pgrep, component):
    state = component.get_state_payload()
    assert state["arena"]["running"] is False
    assert state["arena"]["pid"] is None


# ── Lifecycle tests ──────────────────────────────────────────────────────────

@patch("lucid_component_viz.component._find_pid", return_value=None)
@patch("lucid_component_viz.component.subprocess.Popen")
def test_start_publishes_retained(mock_popen, mock_pgrep, component, mqtt):
    mock_proc = MagicMock()
    mock_proc.poll.return_value = None
    mock_proc.pid = 12345
    mock_popen.return_value = mock_proc

    component.start()

    suffixes = [t.split("/")[-1] for t in mqtt.topics()]
    assert "metadata" in suffixes
    assert "status" in suffixes
    assert "state" in suffixes

    component.stop()


@patch("lucid_component_viz.component._find_pid", return_value=None)
@patch("lucid_component_viz.component.subprocess.Popen")
def test_start_stop_idempotent(mock_popen, mock_pgrep, component):
    mock_proc = MagicMock()
    mock_proc.poll.return_value = None
    mock_proc.pid = 12345
    mock_proc.wait.return_value = 0
    mock_popen.return_value = mock_proc

    component.start()
    component.start()  # idempotent
    component.stop()
    component.stop()  # idempotent


# ── Command handler tests ───────────────────────────────────────────────────

# (start_arena happy-path coverage lives in test_start_arena_sets_env_from_payload
# below, which also asserts the calibration env vars reach the subprocess.)


@patch("lucid_component_viz.component._find_pid", return_value=None)
def test_cmd_stop_arena_when_not_running(mock_pgrep, component, mqtt):
    payload = json.dumps({"request_id": "r2"})
    component.on_cmd_stop_arena(payload)

    results = mqtt.by_suffix("evt/stop_arena/result")
    assert len(results) == 1
    result_payload = json.loads(results[0]["payload"])
    assert result_payload["ok"] is True


@patch("lucid_component_viz.component._find_pid", return_value=None)
@patch("lucid_component_viz.component.subprocess.run")
def test_cmd_start_touchdesigner(mock_run, mock_pgrep, component, mqtt):
    mock_run.return_value = MagicMock(returncode=0)

    payload = json.dumps({"request_id": "r3"})
    component.on_cmd_start_touchdesigner(payload)

    results = mqtt.by_suffix("evt/start_touchdesigner/result")
    assert len(results) == 1
    result_payload = json.loads(results[0]["payload"])
    assert result_payload["ok"] is True


@patch("lucid_component_viz.component._find_pid", return_value=None)
def test_cmd_stop_touchdesigner_when_not_running(mock_pgrep, component, mqtt):
    payload = json.dumps({"request_id": "r4"})
    component.on_cmd_stop_touchdesigner(payload)

    results = mqtt.by_suffix("evt/stop_touchdesigner/result")
    assert len(results) == 1
    result_payload = json.loads(results[0]["payload"])
    assert result_payload["ok"] is True


@patch("lucid_component_viz.component._find_pid", return_value=None)
@patch("lucid_component_viz.component.subprocess.Popen")
def test_cmd_restart(mock_popen, mock_pgrep, component, mqtt):
    mock_proc = MagicMock()
    mock_proc.poll.return_value = None
    mock_proc.pid = 55
    mock_popen.return_value = mock_proc

    payload = json.dumps({"request_id": "r5"})
    component.on_cmd_restart(payload)

    results = mqtt.by_suffix("evt/restart/result")
    assert len(results) == 1
    result_payload = json.loads(results[0]["payload"])
    assert result_payload["ok"] is True


@patch("lucid_component_viz.component._find_pid", return_value=None)
def test_cmd_ping(mock_pgrep, component, mqtt):
    payload = json.dumps({"request_id": "r6"})
    component.on_cmd_ping(payload)

    results = mqtt.by_suffix("evt/ping/result")
    assert len(results) == 1
    result_payload = json.loads(results[0]["payload"])
    assert result_payload["ok"] is True
    assert result_payload["request_id"] == "r6"


@patch("lucid_component_viz.component._find_pid", return_value=None)
def test_cmd_ping_empty_payload(mock_pgrep, component, mqtt):
    component.on_cmd_ping("")

    results = mqtt.by_suffix("evt/ping/result")
    assert len(results) == 1
    result_payload = json.loads(results[0]["payload"])
    assert result_payload["ok"] is True


@patch("lucid_component_viz.component._find_pid", return_value=None)
def test_cmd_ping_malformed_json(mock_pgrep, component, mqtt):
    component.on_cmd_ping("{bad json")

    results = mqtt.by_suffix("evt/ping/result")
    assert len(results) == 1
    result_payload = json.loads(results[0]["payload"])
    assert result_payload["ok"] is True
    assert result_payload["request_id"] == ""


# ── Config tests ─────────────────────────────────────────────────────────────

@patch("lucid_component_viz.component._find_pid", return_value=None)
def test_cfg_payload(mock_pgrep, component):
    cfg = component.get_cfg_payload()
    assert cfg["touchdesigner_app"] == "/Applications/TouchDesigner.app"


@patch("lucid_component_viz.component._find_pid", return_value=None)
def test_cmd_cfg_set_touchdesigner_app(mock_pgrep, component, mqtt):
    payload = json.dumps({
        "request_id": "rc1",
        "set": {"touchdesigner_app": "/Applications/TD2.app"},
    })
    component.on_cmd_cfg_set(payload)

    results = mqtt.by_suffix("evt/cfg/set/result")
    assert len(results) == 1
    result_payload = json.loads(results[0]["payload"])
    assert result_payload["ok"] is True
    assert result_payload["applied"]["touchdesigner_app"] == "/Applications/TD2.app"

    assert component._touchdesigner_app == "/Applications/TD2.app"


# ── Arena/robot config field tests ───────────────────────────────────────────

ARENA_FIELDS = ("arena_anchor_corner", "arena_anchor_x", "arena_anchor_y")
ROBOT_FIELDS = ("goal_x", "goal_y", "goal_qx", "goal_qy", "goal_qz", "goal_qw")
ARENA_VALUES = {
    "arena_anchor_corner": "BL",
    "arena_anchor_x": 3.0,
    "arena_anchor_y": 1.83,
}
ROBOT_VALUES = {
    "goal_x": 3.0,
    "goal_y": 1.83,
    "goal_qx": 0.0,
    "goal_qy": 0.0,
    "goal_qz": 0.0,
    "goal_qw": 1.0,
}
ALL_GOAL_VALUES = {**ARENA_VALUES, **ROBOT_VALUES}

ARENA_ENV_NAMES = {
    "arena_anchor_corner": "LUCID_ARENA_ANCHOR_CORNER",
    "arena_anchor_x": "LUCID_ARENA_ANCHOR_X",
    "arena_anchor_y": "LUCID_ARENA_ANCHOR_Y",
    "goal_x": "LUCID_ARENA_GOAL_X",
    "goal_y": "LUCID_ARENA_GOAL_Y",
    "goal_qx": "LUCID_ARENA_GOAL_QX",
    "goal_qy": "LUCID_ARENA_GOAL_QY",
    "goal_qz": "LUCID_ARENA_GOAL_QZ",
    "goal_qw": "LUCID_ARENA_GOAL_QW",
}


@patch("lucid_component_viz.component._find_pid", return_value=None)
def test_cfg_payload_includes_arena_robot_fields(mock_pgrep, component):
    cfg = component.get_cfg_payload()
    for field in (*ARENA_FIELDS, *ROBOT_FIELDS):
        assert field in cfg, f"missing cfg field: {field}"
        assert cfg[field] is None  # default unset


@patch("lucid_component_viz.component._find_pid", return_value=None)
def test_cmd_cfg_set_arena_robot_fields(mock_pgrep, component, mqtt):
    payload = json.dumps({"request_id": "rc-a", "set": ALL_GOAL_VALUES})
    component.on_cmd_cfg_set(payload)
    results = mqtt.by_suffix("evt/cfg/set/result")
    result_payload = json.loads(results[-1]["payload"])
    assert result_payload["ok"] is True
    cfg = component.get_cfg_payload()
    for k, v in ALL_GOAL_VALUES.items():
        assert cfg[k] == v


@patch("lucid_component_viz.component._find_pid", return_value=None)
def test_cmd_cfg_set_invalid_corner_rejected(mock_pgrep, component, mqtt):
    payload = json.dumps({
        "request_id": "rc-b",
        "set": {"arena_anchor_corner": "middle"},
    })
    component.on_cmd_cfg_set(payload)
    result_payload = json.loads(mqtt.by_suffix("evt/cfg/set/result")[-1]["payload"])
    assert result_payload["ok"] is False
    assert "anchor_corner" in (result_payload.get("error") or "")
    cfg = component.get_cfg_payload()
    assert cfg["arena_anchor_corner"] is None  # not persisted


# ── start_arena env-var wiring tests ────────────────────────────────────────

def _capture_popen():
    """Returns (mock_popen, captured_env_holder).

    captured_env_holder['env'] is set on Popen call.
    """
    captured: dict[str, Any] = {}

    def fake_popen(*args, **kwargs):
        captured["env"] = kwargs.get("env", {})
        proc = MagicMock()
        proc.poll.return_value = None
        proc.pid = 7777
        proc.wait.return_value = 0
        return proc

    return fake_popen, captured


@patch("lucid_component_viz.component._find_pid", return_value=None)
def test_start_arena_sets_env_from_payload(mock_pgrep, component, mqtt):
    fake_popen, captured = _capture_popen()
    with patch("lucid_component_viz.component.subprocess.Popen", side_effect=fake_popen):
        component.on_cmd_start_arena(json.dumps({"request_id": "r1", **ALL_GOAL_VALUES}))
    result = json.loads(mqtt.by_suffix("evt/start_arena/result")[-1]["payload"])
    assert result["ok"] is True, result
    env = captured["env"]
    for field, value in ALL_GOAL_VALUES.items():
        assert env[ARENA_ENV_NAMES[field]] == str(value)


@patch("lucid_component_viz.component._find_pid", return_value=None)
def test_start_arena_uses_cfg_when_payload_partial(mock_pgrep, component, mqtt):
    # Persist all 9 in cfg first.
    component.on_cmd_cfg_set(json.dumps({"request_id": "rc", "set": ALL_GOAL_VALUES}))
    fake_popen, captured = _capture_popen()
    with patch("lucid_component_viz.component.subprocess.Popen", side_effect=fake_popen):
        # Payload missing all goal fields → cfg supplies them.
        component.on_cmd_start_arena(json.dumps({"request_id": "r2"}))
    result = json.loads(mqtt.by_suffix("evt/start_arena/result")[-1]["payload"])
    assert result["ok"] is True, result
    env = captured["env"]
    for field, value in ALL_GOAL_VALUES.items():
        assert env[ARENA_ENV_NAMES[field]] == str(value)


@patch("lucid_component_viz.component._find_pid", return_value=None)
def test_start_arena_payload_overrides_cfg(mock_pgrep, component, mqtt):
    component.on_cmd_cfg_set(json.dumps({"request_id": "rc", "set": ALL_GOAL_VALUES}))
    overrides = {**ALL_GOAL_VALUES, "goal_x": 9.99, "arena_anchor_corner": "TR"}
    fake_popen, captured = _capture_popen()
    with patch("lucid_component_viz.component.subprocess.Popen", side_effect=fake_popen):
        component.on_cmd_start_arena(json.dumps({"request_id": "r3", **overrides}))
    env = captured["env"]
    assert env["LUCID_ARENA_GOAL_X"] == "9.99"
    assert env["LUCID_ARENA_ANCHOR_CORNER"] == "TR"


@patch("lucid_component_viz.component._find_pid", return_value=None)
def test_start_arena_cross_group_independence(mock_pgrep, component, mqtt):
    # Arena group from cfg, robot group from payload.
    component.on_cmd_cfg_set(json.dumps({"request_id": "rc", "set": ARENA_VALUES}))
    fake_popen, captured = _capture_popen()
    with patch("lucid_component_viz.component.subprocess.Popen", side_effect=fake_popen):
        component.on_cmd_start_arena(json.dumps({"request_id": "r4", **ROBOT_VALUES}))
    result = json.loads(mqtt.by_suffix("evt/start_arena/result")[-1]["payload"])
    assert result["ok"] is True, result
    env = captured["env"]
    for field, value in ARENA_VALUES.items():
        assert env[ARENA_ENV_NAMES[field]] == str(value)
    for field, value in ROBOT_VALUES.items():
        assert env[ARENA_ENV_NAMES[field]] == str(value)


@patch("lucid_component_viz.component._find_pid", return_value=None)
def test_start_arena_fails_when_groups_incomplete(mock_pgrep, component, mqtt):
    fake_popen, captured = _capture_popen()
    with patch("lucid_component_viz.component.subprocess.Popen", side_effect=fake_popen):
        component.on_cmd_start_arena(json.dumps({"request_id": "r5"}))
    result = json.loads(mqtt.by_suffix("evt/start_arena/result")[-1]["payload"])
    assert result["ok"] is False
    error = result.get("error") or ""
    assert "arena" in error or "goal" in error
    assert "env" not in captured  # subprocess never spawned


@patch("lucid_component_viz.component._find_pid", return_value=None)
def test_start_arena_fails_with_only_arena_group(mock_pgrep, component, mqtt):
    component.on_cmd_cfg_set(json.dumps({"request_id": "rc", "set": ARENA_VALUES}))
    fake_popen, captured = _capture_popen()
    with patch("lucid_component_viz.component.subprocess.Popen", side_effect=fake_popen):
        # No robot group anywhere.
        component.on_cmd_start_arena(json.dumps({"request_id": "r6"}))
    result = json.loads(mqtt.by_suffix("evt/start_arena/result")[-1]["payload"])
    assert result["ok"] is False
    assert "goal" in (result.get("error") or "")


@patch("lucid_component_viz.component._find_pid", return_value=None)
def test_schema_includes_arena_robot_fields(mock_pgrep, component):
    s = component.schema()
    cfg_fields = s["publishes"]["cfg"]["fields"]
    for f in (*ARENA_FIELDS, *ROBOT_FIELDS):
        assert f in cfg_fields, f"cfg schema missing {f}"
    start_fields = s["subscribes"]["cmd/start-arena"]["fields"]
    for f in (*ARENA_FIELDS, *ROBOT_FIELDS):
        assert f in start_fields, f"start-arena schema missing {f}"
