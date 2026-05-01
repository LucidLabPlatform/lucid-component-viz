"""
Viz component — process supervisor for arena.py and TouchDesigner.

Launches arena.py (bundled) and TouchDesigner as subprocesses.
Monitors health, publishes state/telemetry, and accepts start/stop commands.
"""
from __future__ import annotations

import copy
import json
import os
import signal
import subprocess
import sys
import threading
from datetime import datetime, timezone
from importlib import resources
from typing import Any, Optional

from lucid_component_base import Component, ComponentContext

# Arena data commands — topic links forward data to these cmd/ topics
# instead of telemetry/ so agents don't need subscribe ACL on their own telemetry.
_ARENA_DATA_COMMANDS = [
    "puck_registry",
    "aruco_registry",
    "robot_pose_optitrack",
    "robot_pose_odom",
    "scan",
]

# ── Arena/robot calibration cfg ─────────────────────────────────────────────
# Two independent groups, each all-or-nothing for start_arena:
#   • arena: anchor corner + its OptiTrack (x, y) — places the arena rectangle
#     in OptiTrack world.
#   • robot: goal pose (x, y, qx, qy, qz, qw) in OptiTrack — anchors the robot
#     tree (= odom frame) under world.
# Field names mirror the existing reset-to-start template params so values can
# flow through experiment templates → cfg/start_arena payload → arena.py
# subprocess via env vars.
_ARENA_FIELDS = ("arena_anchor_corner", "arena_anchor_x", "arena_anchor_y")
_ROBOT_FIELDS = ("goal_x", "goal_y", "goal_qx", "goal_qy", "goal_qz", "goal_qw")
_ALL_GOAL_FIELDS = (*_ARENA_FIELDS, *_ROBOT_FIELDS)
_VALID_ANCHOR_CORNERS = ("TL", "TR", "BR", "BL")

# Field name → env var name passed to arena.py subprocess.
_ENV_VAR_FOR: dict[str, str] = {
    "arena_anchor_corner": "LUCID_ARENA_ANCHOR_CORNER",
    "arena_anchor_x":      "LUCID_ARENA_ANCHOR_X",
    "arena_anchor_y":      "LUCID_ARENA_ANCHOR_Y",
    "goal_x":              "LUCID_ARENA_GOAL_X",
    "goal_y":              "LUCID_ARENA_GOAL_Y",
    "goal_qx":             "LUCID_ARENA_GOAL_QX",
    "goal_qy":             "LUCID_ARENA_GOAL_QY",
    "goal_qz":             "LUCID_ARENA_GOAL_QZ",
    "goal_qw":             "LUCID_ARENA_GOAL_QW",
}


def _utc_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _arena_script_path() -> str:
    """Resolve the bundled arena.py path."""
    return str(resources.files("lucid_component_viz").joinpath("arena/arena.py"))


def _find_pid(name: str) -> Optional[int]:
    """Find a running process PID by name using pgrep."""
    try:
        result = subprocess.run(
            ["pgrep", "-f", name],
            capture_output=True, text=True, timeout=5,
        )
        if result.returncode == 0 and result.stdout.strip():
            return int(result.stdout.strip().splitlines()[0])
    except Exception:
        pass
    return None


class VizComponent(Component):
    """
    Visualization process supervisor.

    Manages arena.py (pygame overlay) and TouchDesigner as subprocesses.
    Retained: metadata, status, state, cfg.
    Commands: start_arena, stop_arena, start_touchdesigner, stop_touchdesigner, restart.
    """

    _HEALTH_INTERVAL_S = 5.0

    def __init__(self, context: ComponentContext) -> None:
        super().__init__(context)
        self._log = self.context.logger()
        self._stop_event = threading.Event()
        self._health_thread: Optional[threading.Thread] = None
        self._arena_proc: Optional[subprocess.Popen] = None
        self._td_pid: Optional[int] = None
        self._touchdesigner_app: str = self.context.config.get(
            "touchdesigner_app", "/Applications/TouchDesigner.app"
        )
        self._touchdesigner_file: Optional[str] = self.context.config.get(
            "touchdesigner_file", "/Users/roboticslab/Documents/WorkingProjectionV2.1.toe"
        )
        # Arena/robot calibration cfg fields, all default None until either
        # cfg/set persists them or they're supplied in start_arena payload.
        self._goal_cfg: dict[str, Any] = {f: self.context.config.get(f) for f in _ALL_GOAL_FIELDS}

    @property
    def component_id(self) -> str:
        return "viz"

    def capabilities(self) -> list[str]:
        return [
            "start-arena", "stop-arena",
            "start-touchdesigner", "stop-touchdesigner",
            "restart",
        ] + _ARENA_DATA_COMMANDS

    def __getattr__(self, name: str) -> Any:
        """Route on_cmd_<data_command> calls to the arena subprocess."""
        if name.startswith("on_cmd_"):
            command = name[7:]  # strip "on_cmd_"
            if command in _ARENA_DATA_COMMANDS:
                def _handler(payload_str: str) -> None:
                    topic = self.context.topic("cmd/" + command)
                    self._on_arena_message(topic, payload_str)
                return _handler
        raise AttributeError(f"{type(self).__name__!r} has no attribute {name!r}")

    def metadata(self) -> dict[str, Any]:
        out = super().metadata()
        out["capabilities"] = self.capabilities()
        return out

    def get_state_payload(self) -> dict[str, Any]:
        arena_running = self._arena_proc is not None and self._arena_proc.poll() is None
        arena_pid = self._arena_proc.pid if arena_running and self._arena_proc else None

        td_pid = _find_pid("TouchDesigner")
        self._td_pid = td_pid

        return {
            "arena": {"running": arena_running, "pid": arena_pid},
            "touchdesigner": {"running": td_pid is not None, "pid": td_pid},
        }

    def get_cfg_payload(self) -> dict[str, Any]:
        return {
            "touchdesigner_app": self._touchdesigner_app,
            "touchdesigner_file": self._touchdesigner_file,
            **self._goal_cfg,
        }

    def schema(self) -> dict[str, Any]:
        s = copy.deepcopy(super().schema())
        s["publishes"]["state"]["fields"].update({
            "arena": {
                "type": "object",
                "fields": {
                    "running": {"type": "boolean"},
                    "pid": {"type": "integer"},
                },
            },
            "touchdesigner": {
                "type": "object",
                "fields": {
                    "running": {"type": "boolean"},
                    "pid": {"type": "integer"},
                },
            },
        })
        s["publishes"]["cfg"]["fields"].update({
            "touchdesigner_app": {"type": "string"},
            "touchdesigner_file": {"type": "string"},
            "arena_anchor_corner": {"type": "string"},
            "arena_anchor_x": {"type": "number"},
            "arena_anchor_y": {"type": "number"},
            "goal_x": {"type": "number"},
            "goal_y": {"type": "number"},
            "goal_qx": {"type": "number"},
            "goal_qy": {"type": "number"},
            "goal_qz": {"type": "number"},
            "goal_qw": {"type": "number"},
        })
        s["subscribes"].update({
            "cmd/start-arena": {"fields": {
                "arena_anchor_corner": {"type": "string"},
                "arena_anchor_x": {"type": "number"},
                "arena_anchor_y": {"type": "number"},
                "goal_x": {"type": "number"},
                "goal_y": {"type": "number"},
                "goal_qx": {"type": "number"},
                "goal_qy": {"type": "number"},
                "goal_qz": {"type": "number"},
                "goal_qw": {"type": "number"},
            }},
            "cmd/stop-arena": {"fields": {}},
            "cmd/start-touchdesigner": {"fields": {}},
            "cmd/stop-touchdesigner": {"fields": {}},
            "cmd/restart": {"fields": {}},
        })
        return s

    # ── Lifecycle ────────────────────────────────────────────────────────────

    def _start(self) -> None:
        self._publish_all_retained()
        self._stop_event.clear()
        self._health_thread = threading.Thread(
            target=self._health_loop, name="LucidVizHealth", daemon=True,
        )
        self._health_thread.start()
        self._log.info("Started viz component")

    def _stop(self) -> None:
        self._stop_event.set()
        t = self._health_thread
        if t:
            t.join(timeout=3.0)
            self._health_thread = None
        self._stop_arena()
        self._stop_touchdesigner()
        self._log.info("Stopped viz component")

    def _publish_all_retained(self) -> None:
        self.publish_metadata()
        self.publish_schema()
        self.publish_status()
        self.publish_state()
        self.publish_cfg()

    # ── Arena process management ─────────────────────────────────────────────

    def _resolve_goal_config(
        self, payload_overrides: dict[str, Any]
    ) -> tuple[Optional[dict[str, Any]], Optional[str]]:
        """Resolve the 9 calibration fields, all-or-nothing per group.

        For each of the two groups (arena, robot), prefers payload (if
        complete) over cfg (if complete), else fails. Returns (resolved, None)
        on success or (None, error_message) on failure.
        """
        resolved: dict[str, Any] = {}
        problems: list[str] = []
        for group_name, fields in (("arena", _ARENA_FIELDS), ("robot", _ROBOT_FIELDS)):
            payload_full = all(payload_overrides.get(f) is not None for f in fields)
            cfg_full = all(self._goal_cfg.get(f) is not None for f in fields)
            if payload_full:
                source = {f: payload_overrides[f] for f in fields}
            elif cfg_full:
                source = {f: self._goal_cfg[f] for f in fields}
            else:
                missing = [
                    f for f in fields
                    if payload_overrides.get(f) is None and self._goal_cfg.get(f) is None
                ]
                problems.append(
                    f"{group_name} group incomplete; missing in both payload and cfg: {missing}"
                )
                continue
            resolved.update(source)
        if problems:
            return None, "; ".join(problems)
        # Validate anchor corner string.
        corner = resolved.get("arena_anchor_corner")
        if corner not in _VALID_ANCHOR_CORNERS:
            return None, (
                f"arena_anchor_corner must be one of {_VALID_ANCHOR_CORNERS}, "
                f"got {corner!r}"
            )
        return resolved, None

    def _start_arena(self, payload_overrides: Optional[dict[str, Any]] = None) -> tuple[bool, Optional[str]]:
        """Start arena.py subprocess. Returns (ok, error_message_if_any)."""
        if self._arena_proc is not None and self._arena_proc.poll() is None:
            self._log.info("arena.py already running (pid=%s)", self._arena_proc.pid)
            return True, None
        resolved, error = self._resolve_goal_config(payload_overrides or {})
        if resolved is None:
            self._log.error("start_arena: cannot resolve calibration: %s", error)
            return False, error
        try:
            arena_path = _arena_script_path()
            env = dict(os.environ)
            env["PYTHONUNBUFFERED"] = "1"
            for field, value in resolved.items():
                env[_ENV_VAR_FOR[field]] = str(value)
            log_dir = os.path.join(os.getenv("LUCID_AGENT_BASE_DIR", "."), "logs")
            os.makedirs(log_dir, exist_ok=True)
            arena_log = open(os.path.join(log_dir, "arena.log"), "a")
            self._arena_proc = subprocess.Popen(
                [sys.executable, "-u", arena_path],
                env=env,
                stdin=subprocess.PIPE,
                stdout=arena_log,
                stderr=arena_log,
            )
            self._log.info("Started arena.py (pid=%s), log: %s/arena.log", self._arena_proc.pid, log_dir)
            return True, None
        except Exception as e:
            self._log.exception("Failed to start arena.py")
            return False, f"subprocess.Popen failed: {e}"

    def _on_arena_message(self, topic: str, payload_str: str) -> None:
        """Forward an MQTT message to the arena subprocess via stdin.

        Drops are logged (not silenced) so we can see when arena is wedged or
        the pipe is full. Use suffix-only topic in log to keep volume readable.
        """
        proc = self._arena_proc
        if proc is None or proc.poll() is not None or proc.stdin is None:
            return
        try:
            payload = json.loads(payload_str) if payload_str else {}
        except json.JSONDecodeError as e:
            self._log.warning("arena forward: bad JSON for %s: %s", topic, e)
            return
        try:
            line = json.dumps({"topic": topic, "payload": payload}) + "\n"
            proc.stdin.write(line.encode())
            proc.stdin.flush()
        except (BrokenPipeError, OSError) as e:
            self._log.warning("arena forward: stdin write failed for %s: %s", topic, e)
        except Exception:
            self._log.exception("arena forward: unexpected error for %s", topic)

    def _stop_arena(self) -> bool:
        proc = self._arena_proc
        if proc is None or proc.poll() is not None:
            self._arena_proc = None
            return True
        try:
            if proc.stdin:
                try:
                    proc.stdin.close()
                except Exception:
                    pass
            proc.terminate()
            try:
                proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=2.0)
            self._log.info("Stopped arena.py (pid=%s)", proc.pid)
        except Exception:
            self._log.exception("Failed to stop arena.py")
            return False
        finally:
            self._arena_proc = None
        return True

    # ── TouchDesigner process management ─────────────────────────────────────

    def _start_touchdesigner(self) -> bool:
        if _find_pid("TouchDesigner") is not None:
            self._log.info("TouchDesigner already running")
            return True
        try:
            cmd = ["open", "-a", self._touchdesigner_app]
            if self._touchdesigner_file:
                cmd.append(self._touchdesigner_file)
            self._log.info("Launching TouchDesigner: %s", " ".join(cmd))
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=10,
            )
            if result.returncode != 0:
                self._log.error(
                    "TouchDesigner launch failed (exit=%s): %s",
                    result.returncode, result.stderr.strip(),
                )
                return False
            self._log.info("Launched TouchDesigner (file=%s)", self._touchdesigner_file or "default")
            return True
        except Exception:
            self._log.exception("Failed to start TouchDesigner")
            return False

    def _stop_touchdesigner(self) -> bool:
        pid = _find_pid("TouchDesigner")
        if pid is None:
            return True
        try:
            os.kill(pid, signal.SIGTERM)
            self._log.info("Sent SIGTERM to TouchDesigner (pid=%s)", pid)
            return True
        except Exception:
            self._log.exception("Failed to stop TouchDesigner (pid=%s)", pid)
            return False

    # ── Health monitoring ────────────────────────────────────────────────────

    def _health_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                state = self.get_state_payload()
                self.publish_state(state)
                arena = state.get("arena", {})
                td = state.get("touchdesigner", {})
                if self.should_publish_telemetry("process/arena", arena):
                    self.publish_telemetry("process/arena", arena)
                if self.should_publish_telemetry("process/touchdesigner", td):
                    self.publish_telemetry("process/touchdesigner", td)
            except Exception:
                self._log.exception("Health check failed")
            if self._stop_event.wait(self._HEALTH_INTERVAL_S):
                break

    # ── Command handlers ─────────────────────────────────────────────────────

    def _parse_request_id(self, payload_str: str) -> str:
        try:
            payload = json.loads(payload_str) if payload_str else {}
            return payload.get("request_id", "")
        except json.JSONDecodeError:
            return ""

    def on_cmd_start_arena(self, payload_str: str) -> None:
        self._log.info("cmd/start_arena received: %s", payload_str)
        request_id = self._parse_request_id(payload_str)
        # Pull any of the 9 calibration fields out of the payload for one-shot
        # override; missing fields fall through to cfg.
        try:
            payload = json.loads(payload_str) if payload_str else {}
        except json.JSONDecodeError:
            payload = {}
        overrides = {f: payload[f] for f in _ALL_GOAL_FIELDS if f in payload}
        ok, error = self._start_arena(payload_overrides=overrides)
        self._log.info("start_arena result: ok=%s, request_id=%s", ok, request_id)
        self.publish_result("start_arena", request_id, ok=ok, error=None if ok else error)
        try:
            self.publish_state()
        except Exception:
            self._log.exception("publish_state failed after start_arena")

    def on_cmd_stop_arena(self, payload_str: str) -> None:
        request_id = self._parse_request_id(payload_str)
        ok = self._stop_arena()
        self.publish_result("stop_arena", request_id, ok=ok, error=None if ok else "failed to stop")
        try:
            self.publish_state()
        except Exception:
            self._log.exception("publish_state failed after stop_arena")

    def on_cmd_start_touchdesigner(self, payload_str: str) -> None:
        request_id = self._parse_request_id(payload_str)
        ok = self._start_touchdesigner()
        self.publish_result(
            "start_touchdesigner", request_id, ok=ok,
            error=None if ok else "failed to start",
        )
        try:
            self.publish_state()
        except Exception:
            self._log.exception("publish_state failed after start_touchdesigner")

    def on_cmd_stop_touchdesigner(self, payload_str: str) -> None:
        request_id = self._parse_request_id(payload_str)
        ok = self._stop_touchdesigner()
        self.publish_result(
            "stop_touchdesigner", request_id, ok=ok,
            error=None if ok else "failed to stop",
        )
        try:
            self.publish_state()
        except Exception:
            self._log.exception("publish_state failed after stop_touchdesigner")

    def on_cmd_restart(self, payload_str: str) -> None:
        request_id = self._parse_request_id(payload_str)
        self._stop_arena()
        self._stop_touchdesigner()
        self.publish_result("restart", request_id, ok=True, error=None)
        try:
            self.publish_state()
        except Exception:
            self._log.exception("publish_state failed after restart")

    def on_cmd_reset(self, payload_str: str) -> None:
        self.on_cmd_restart(payload_str)

    def on_cmd_ping(self, payload_str: str) -> None:
        request_id = self._parse_request_id(payload_str)
        self.publish_result("ping", request_id, ok=True, error=None)

    def on_cmd_cfg_set(self, payload_str: str) -> None:
        request_id, set_dict, parse_error = self._parse_cfg_set_payload(payload_str)
        if parse_error:
            self.publish_cfg_set_result(
                request_id=request_id, ok=False, applied=None,
                error=parse_error, ts=_utc_iso(), action="cfg/set",
            )
            return

        applied: dict[str, Any] = {}
        if "touchdesigner_app" in set_dict:
            self._touchdesigner_app = str(set_dict["touchdesigner_app"])
            applied["touchdesigner_app"] = self._touchdesigner_app
        if "touchdesigner_file" in set_dict:
            val = set_dict["touchdesigner_file"]
            self._touchdesigner_file = str(val) if val is not None else None
            applied["touchdesigner_file"] = self._touchdesigner_file

        # Validate anchor corner before applying anything from the goal group;
        # rejecting one field aborts the whole set so cfg never lands in a
        # partially-bad state.
        if "arena_anchor_corner" in set_dict:
            corner = set_dict["arena_anchor_corner"]
            if corner is not None and corner not in _VALID_ANCHOR_CORNERS:
                self.publish_cfg_set_result(
                    request_id=request_id, ok=False, applied=None,
                    error=(
                        f"arena_anchor_corner must be one of "
                        f"{_VALID_ANCHOR_CORNERS}, got {corner!r}"
                    ),
                    ts=_utc_iso(), action="cfg/set",
                )
                return

        for field in _ALL_GOAL_FIELDS:
            if field not in set_dict:
                continue
            value = set_dict[field]
            if value is None:
                self._goal_cfg[field] = None
            elif field == "arena_anchor_corner":
                self._goal_cfg[field] = str(value)
            else:
                try:
                    self._goal_cfg[field] = float(value)
                except (TypeError, ValueError):
                    self.publish_cfg_set_result(
                        request_id=request_id, ok=False, applied=None,
                        error=f"{field}={value!r} is not a valid number",
                        ts=_utc_iso(), action="cfg/set",
                    )
                    return
            applied[field] = self._goal_cfg[field]

        self.publish_cfg()
        self.publish_cfg_set_result(
            request_id=request_id, ok=True,
            applied=applied if applied else None,
            error=None, ts=_utc_iso(), action="cfg/set",
        )

