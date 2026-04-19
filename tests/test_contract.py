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

@patch("lucid_component_viz.component._find_pid", return_value=None)
@patch("lucid_component_viz.component.subprocess.Popen")
def test_cmd_start_arena(mock_popen, mock_pgrep, component, mqtt):
    mock_proc = MagicMock()
    mock_proc.poll.return_value = None
    mock_proc.pid = 99
    mock_popen.return_value = mock_proc

    payload = json.dumps({"request_id": "r1"})
    component.on_cmd_start_arena(payload)

    results = mqtt.by_suffix("evt/start_arena/result")
    assert len(results) == 1
    result_payload = json.loads(results[0]["payload"])
    assert result_payload["request_id"] == "r1"
    assert result_payload["ok"] is True


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
