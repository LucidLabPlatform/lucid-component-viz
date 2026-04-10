"""
Viz component — process supervisor for arena.py and TouchDesigner.

Launches arena.py (bundled) and TouchDesigner as subprocesses.
Monitors health, publishes state/telemetry, and accepts start/stop commands.
"""
from __future__ import annotations

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

    @property
    def component_id(self) -> str:
        return "viz"

    def capabilities(self) -> list[str]:
        return [
            "start_arena", "stop_arena",
            "start_touchdesigner", "stop_touchdesigner",
            "restart",
        ]

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
        }

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
        self.publish_status()
        self.publish_state()
        self.publish_cfg()

    # ── Arena process management ─────────────────────────────────────────────

    def _start_arena(self) -> bool:
        if self._arena_proc is not None and self._arena_proc.poll() is None:
            self._log.info("arena.py already running (pid=%s)", self._arena_proc.pid)
            return True
        try:
            arena_path = _arena_script_path()
            env = dict(os.environ)
            env.update({
                "MQTT_HOST": os.getenv("MQTT_HOST", "localhost"),
                "MQTT_PORT": os.getenv("MQTT_PORT", "1883"),
                "AGENT_USERNAME": os.getenv("AGENT_USERNAME", ""),
                "AGENT_PASSWORD": os.getenv("AGENT_PASSWORD", ""),
            })
            log_dir = os.path.join(os.getenv("LUCID_AGENT_BASE_DIR", "."), "logs")
            os.makedirs(log_dir, exist_ok=True)
            arena_log = open(os.path.join(log_dir, "arena.log"), "a")
            self._arena_proc = subprocess.Popen(
                [sys.executable, arena_path],
                env=env,
                stdout=arena_log,
                stderr=arena_log,
            )
            self._log.info("Started arena.py (pid=%s), log: %s/arena.log", self._arena_proc.pid, log_dir)
            return True
        except Exception:
            self._log.exception("Failed to start arena.py")
            return False

    def _stop_arena(self) -> bool:
        proc = self._arena_proc
        if proc is None or proc.poll() is not None:
            self._arena_proc = None
            return True
        try:
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
        ok = self._start_arena()
        self._log.info("start_arena result: ok=%s, request_id=%s", ok, request_id)
        self.publish_result("start_arena", request_id, ok=ok, error=None if ok else "failed to start")
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

        self.publish_cfg()
        self.publish_cfg_set_result(
            request_id=request_id, ok=True,
            applied=applied if applied else None,
            error=None, ts=_utc_iso(), action="cfg/set",
        )

