#!/usr/bin/env python3
from __future__ import annotations

import json
import os
import hashlib
from pathlib import Path
import pty
import queue
import select
import subprocess
import threading
import time
from dataclasses import dataclass
from datetime import datetime, UTC
from typing import Any
from urllib.parse import urlencode, urlparse, urlunparse

import requests
from websocket import WebSocketException, WebSocketTimeoutException, create_connection

from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.system.hardware import HARDWARE

ACTIVATION_CACHE_FILE = Path(os.getenv("ACTIVATION_CACHE_FILE_PATH", "/data/openpilot_cache/activation_authorized_serial"))
ERROR_LOG_STATE_FILE = Path(os.getenv("REMOTE_AGENT_ERROR_LOG_STATE_FILE", "/data/openpilot_cache/remote_agent_error_log.sha256"))


def _now_iso() -> str:
  return datetime.now(UTC).isoformat()


def _clip_text(text: str, max_len: int) -> str:
  if len(text) <= max_len:
    return text
  return text[:max_len] + "\n...[truncated]"


@dataclass
class AgentConfig:
  base_url: str
  heartbeat_sec: int
  request_timeout_sec: int
  max_output_chars: int
  error_log_enabled: bool
  error_log_path: str
  error_log_max_chars: int
  error_log_sync_interval_sec: float
  allow_custom_commands: bool
  terminal_enabled: bool
  terminal_max_sessions: int
  terminal_recv_timeout_sec: float
  terminal_ping_interval_sec: float
  terminal_shell: str
  readonly_prefixes: tuple[str, ...]

  @classmethod
  def from_env(cls) -> AgentConfig:
    prefixes = tuple(filter(None, (p.strip() for p in os.getenv(
      "REMOTE_AGENT_READONLY_PREFIXES",
      "whoami,uname,id,ip,ifconfig,nmcli,df,free,uptime,ps,cat /proc",
    ).split(","))))

    return cls(
      # Default control-plane host for auto-connect after device update.
      # Emergency override: set REMOTE_AGENT_BASE_URL to another endpoint or empty string.
      base_url=os.getenv("REMOTE_AGENT_BASE_URL", "https://cp.zh182.cn").rstrip("/"),
      heartbeat_sec=max(int(os.getenv("REMOTE_AGENT_HEARTBEAT_SEC", "20")), 5),
      request_timeout_sec=max(int(os.getenv("REMOTE_AGENT_REQUEST_TIMEOUT_SEC", "10")), 3),
      max_output_chars=max(int(os.getenv("REMOTE_AGENT_MAX_OUTPUT_CHARS", "65536")), 1024),
      error_log_enabled=os.getenv("REMOTE_AGENT_ERROR_LOG_ENABLED", "1") == "1",
      error_log_path=os.getenv("REMOTE_AGENT_ERROR_LOG_PATH", "/data/community/crashes/error.log"),
      error_log_max_chars=max(int(os.getenv("REMOTE_AGENT_ERROR_LOG_MAX_CHARS", "65536")), 1024),
      error_log_sync_interval_sec=max(float(os.getenv("REMOTE_AGENT_ERROR_LOG_SYNC_INTERVAL_SEC", "20")), 1.0),
      allow_custom_commands=os.getenv("REMOTE_AGENT_ALLOW_CUSTOM_CMDS", "0") == "1",
      terminal_enabled=os.getenv("REMOTE_AGENT_TERMINAL_ENABLED", "1") == "1",
      terminal_max_sessions=max(int(os.getenv("REMOTE_AGENT_TERMINAL_MAX_SESSIONS", "1")), 1),
      terminal_recv_timeout_sec=max(float(os.getenv("REMOTE_AGENT_TERMINAL_RECV_TIMEOUT_SEC", "0.2")), 0.05),
      terminal_ping_interval_sec=max(float(os.getenv("REMOTE_AGENT_TERMINAL_PING_INTERVAL_SEC", "10")), 1.0),
      terminal_shell=os.getenv("REMOTE_AGENT_TERMINAL_SHELL", "/bin/bash"),
      readonly_prefixes=prefixes,
    )


class RemoteAgent:
  def __init__(self, config: AgentConfig):
    self.cfg = config
    self.params = Params()
    self.session = requests.Session()
    self.serial = HARDWARE.get_serial()
    self.device_fingerprint = self._build_device_fingerprint()
    self.token: str | None = None
    self.license_status = "unknown"
    self.next_poll_sec = self.cfg.heartbeat_sec
    self.last_uploaded_error_log_sha256 = self._read_last_uploaded_error_log_hash()
    self.server_error_log_sha256 = ""
    self.server_error_log_sha_known = False
    self.last_error_log_sync_monotonic = 0.0
    self.terminal_workers: dict[str, threading.Thread] = {}
    self.terminal_workers_lock = threading.Lock()

  def _headers(self) -> dict[str, str]:
    headers = {"Content-Type": "application/json"}
    if self.token:
      headers["Authorization"] = f"Bearer {self.token}"
    return headers

  def _post(self, endpoint: str, payload: dict[str, Any]) -> dict[str, Any]:
    resp = self.session.post(
      f"{self.cfg.base_url}{endpoint}",
      json=payload,
      headers=self._headers(),
      timeout=self.cfg.request_timeout_sec,
    )
    resp.raise_for_status()
    data = resp.json()
    if not isinstance(data, dict):
      raise ValueError(f"invalid response payload from {endpoint}")
    return data

  def _is_onroad(self) -> bool:
    try:
      return self.params.get_bool("IsOnroad")
    except Exception:
      return False

  def _get_str_param(self, key: str) -> str:
    try:
      value = self.params.get(key)
      if isinstance(value, bytes):
        return value.decode("utf-8", errors="ignore")
      if value is None:
        return ""
      return str(value)
    except Exception:
      return ""

  def _build_device_fingerprint(self) -> str:
    # Keep fingerprint stable across process restarts; do not rotate on each boot.
    hw_serial = self._get_str_param("HardwareSerial").strip()
    return hw_serial or self.serial

  def _is_command_allowed(self, command: str) -> bool:
    normalized = command.strip().lower()
    return any(normalized.startswith(prefix.lower()) for prefix in self.cfg.readonly_prefixes)

  def _sync_activation_cache_with_license(self) -> None:
    # If server no longer authorizes this device, remove local activation cache so next boot must re-check.
    if self.license_status == "authorized":
      return
    try:
      if ACTIVATION_CACHE_FILE.is_file():
        ACTIVATION_CACHE_FILE.unlink()
        cloudlog.info(f"remote_agent.activation_cache.cleared serial={self.serial} status={self.license_status}")
    except Exception:
      cloudlog.exception("remote_agent.activation_cache.clear_failed")

  def _read_last_uploaded_error_log_hash(self) -> str:
    try:
      if ERROR_LOG_STATE_FILE.is_file():
        return ERROR_LOG_STATE_FILE.read_text().strip()
    except Exception:
      cloudlog.exception("remote_agent.error_log.state_read_failed")
    return ""

  def _write_last_uploaded_error_log_hash(self, sha256: str) -> None:
    try:
      ERROR_LOG_STATE_FILE.parent.mkdir(parents=True, exist_ok=True)
      ERROR_LOG_STATE_FILE.write_text(sha256 + "\n")
    except Exception:
      cloudlog.exception("remote_agent.error_log.state_write_failed")

  def _read_error_log_snapshot(self) -> dict[str, Any] | None:
    log_path = Path(self.cfg.error_log_path)
    if not log_path.is_file():
      return None
    try:
      content = log_path.read_text(encoding="utf-8", errors="replace")
    except Exception:
      cloudlog.exception("remote_agent.error_log.read_failed")
      return None

    if not content.strip():
      return None

    try:
      stat = log_path.stat()
      size_bytes = int(stat.st_size)
      mtime_ns = int(stat.st_mtime_ns)
    except Exception:
      size_bytes = len(content.encode("utf-8", errors="replace"))
      mtime_ns = 0

    sha256 = hashlib.sha256(content.encode("utf-8", errors="replace")).hexdigest()
    return {
      "log_name": "error.log",
      "log_path": str(log_path),
      "sha256": sha256,
      "size_bytes": size_bytes,
      "mtime_ns": mtime_ns,
      "content": _clip_text(content, self.cfg.error_log_max_chars),
      "captured_at": _now_iso(),
    }

  def sync_error_log(self) -> None:
    if not self.cfg.error_log_enabled:
      return
    now = time.monotonic()
    if now - self.last_error_log_sync_monotonic < self.cfg.error_log_sync_interval_sec:
      return
    self.last_error_log_sync_monotonic = now

    snapshot = self._read_error_log_snapshot()
    if snapshot is None:
      return

    sha256 = str(snapshot.get("sha256", "")).strip()
    if not sha256:
      return
    if self.server_error_log_sha_known:
      if sha256 == self.last_uploaded_error_log_sha256 and sha256 == self.server_error_log_sha256:
        return
    elif sha256 == self.last_uploaded_error_log_sha256:
      return

    payload = {"serial": self.serial, **snapshot}
    try:
      self._post("/api/v1/device/error-log", payload)
      self.last_uploaded_error_log_sha256 = sha256
      self.server_error_log_sha256 = sha256
      self.server_error_log_sha_known = True
      self._write_last_uploaded_error_log_hash(sha256)
      cloudlog.info(f"remote_agent.error_log.uploaded serial={self.serial} sha256={sha256[:12]}")
    except requests.HTTPError as e:
      status_code = getattr(e.response, "status_code", 0)
      if status_code in (401, 403):
        self.token = None
      cloudlog.exception(f"remote_agent.error_log.upload_failed status_code={status_code}")
    except Exception:
      cloudlog.exception("remote_agent.error_log.upload_exception")

  def register(self) -> None:
    payload = {
      "serial": self.serial,
      "device_fingerprint": self.device_fingerprint,
      "sw_version": self._get_str_param("Version"),
    }
    data = self._post("/api/v1/device/register", payload)
    self.token = data.get("token") or self.token
    self.license_status = str(data.get("license_status", self.license_status)).strip().lower()
    self._sync_activation_cache_with_license()
    self.next_poll_sec = int(data.get("heartbeat_interval_sec", self.cfg.heartbeat_sec))

  def heartbeat(self) -> None:
    payload = {
      "serial": self.serial,
      "branch": self._get_str_param("GitBranch"),
      "repo_url": self._get_str_param("GitRemote"),
      "op_version": self._get_str_param("Version"),
      "onroad": self._is_onroad(),
      "ts": _now_iso(),
    }
    data = self._post("/api/v1/device/heartbeat", payload)
    self.license_status = str(data.get("license_status", self.license_status)).strip().lower()
    if "error_log_sha256" in data:
      self.server_error_log_sha_known = True
      self.server_error_log_sha256 = str(data.get("error_log_sha256", "")).strip()
    else:
      self.server_error_log_sha_known = False
    self._sync_activation_cache_with_license()
    self.next_poll_sec = int(data.get("next_poll_sec", self.cfg.heartbeat_sec))

  def pull_commands(self) -> list[dict[str, Any]]:
    payload = {"serial": self.serial}
    data = self._post("/api/v1/device/pull-commands", payload)
    commands = data.get("commands", [])
    return commands if isinstance(commands, list) else []

  def report_result(self, command_id: str, result: dict[str, Any]) -> None:
    payload = {"serial": self.serial, "command_id": command_id, **result}
    self._post("/api/v1/device/command-result", payload)

  def _ws_url(self, endpoint: str, query: dict[str, str]) -> str:
    parsed = urlparse(self.cfg.base_url)
    if parsed.scheme not in ("http", "https"):
      raise ValueError(f"unsupported base url scheme: {parsed.scheme}")
    scheme = "wss" if parsed.scheme == "https" else "ws"
    qs = urlencode(query)
    return urlunparse((scheme, parsed.netloc, endpoint, "", qs, ""))

  def pull_terminal_sessions(self) -> list[dict[str, Any]]:
    payload = {"serial": self.serial, "limit": self.cfg.terminal_max_sessions}
    data = self._post("/api/v1/device/pull-terminal-sessions", payload)
    sessions = data.get("sessions", [])
    return sessions if isinstance(sessions, list) else []

  def _read_stream(self, stream: Any, name: str, out_q: queue.Queue[dict[str, str]], done_event: threading.Event) -> None:
    try:
      while not done_event.is_set():
        chunk = stream.read(4096)
        if not chunk:
          break
        out_q.put({
          "type": "output",
          "stream": name,
          "data": _clip_text(chunk.decode("utf-8", errors="replace"), self.cfg.max_output_chars),
        })
    except Exception as e:
      out_q.put({"type": "error", "stream": name, "message": f"stream read error: {e}"})
    finally:
      out_q.put({"type": "stream_closed", "stream": name})

  def _terminal_session_loop(self, session_id: str) -> None:
    if not self.token:
      return

    ws_url = self._ws_url("/api/v1/device/terminal/attach", {
      "serial": self.serial,
      "session_id": session_id,
      "token": self.token,
    })

    ws = None
    proc = None
    master_fd: int | None = None
    exit_code: int | None = None
    exit_sent = False
    last_ping_ts = time.monotonic()

    try:
      ws = create_connection(ws_url, timeout=max(self.cfg.request_timeout_sec, 3))
      ws.settimeout(self.cfg.terminal_recv_timeout_sec)

      master_fd, slave_fd = pty.openpty()
      try:
        proc = subprocess.Popen(  # pylint: disable=consider-using-with
          [self.cfg.terminal_shell],
          stdin=slave_fd,
          stdout=slave_fd,
          stderr=slave_fd,
          bufsize=0,
          close_fds=True,
          start_new_session=True,
        )
      finally:
        os.close(slave_fd)

      ws.send(json.dumps({"type": "status", "status": "attached", "session_id": session_id}))

      while True:
        if self._is_onroad():
          ws.send(json.dumps({"type": "status", "status": "closing", "reason": "onroad"}))
          break

        now_monotonic = time.monotonic()
        if now_monotonic - last_ping_ts >= self.cfg.terminal_ping_interval_sec:
          ws.send(json.dumps({"type": "ping", "ts": _now_iso()}))
          last_ping_ts = now_monotonic

        if proc.poll() is not None and not exit_sent:
          exit_code = int(proc.returncode)
          ws.send(json.dumps({"type": "exit", "exit_code": str(exit_code)}))
          exit_sent = True

        if master_fd is not None:
          while True:
            try:
              readable, _, _ = select.select([master_fd], [], [], 0)
            except Exception:
              readable = []
            if not readable:
              break

            try:
              chunk = os.read(master_fd, 4096)
            except OSError:
              chunk = b""

            if not chunk:
              break

            ws.send(json.dumps({
              "type": "output",
              "stream": "pty",
              "data": _clip_text(chunk.decode("utf-8", errors="replace"), self.cfg.max_output_chars),
            }))

        if exit_sent and proc.poll() is not None:
          if master_fd is None:
            break
          try:
            readable, _, _ = select.select([master_fd], [], [], 0)
          except Exception:
            readable = []
          if not readable:
            break

        try:
          raw_msg = ws.recv()
        except WebSocketTimeoutException:
          continue

        if raw_msg is None:
          break
        if isinstance(raw_msg, bytes):
          raw_msg = raw_msg.decode("utf-8", errors="replace")
        if not isinstance(raw_msg, str):
          continue

        try:
          message = json.loads(raw_msg)
        except Exception:
          message = {"type": "input", "data": raw_msg}

        msg_type = str(message.get("type", "")).strip().lower()
        if msg_type == "input":
          data = str(message.get("data", ""))
          if master_fd is not None:
            try:
              os.write(master_fd, data.encode("utf-8", errors="ignore"))
            except OSError:
              break
        elif msg_type == "close":
          break
        elif msg_type == "ping":
          ws.send(json.dumps({"type": "pong", "ts": _now_iso()}))
        elif msg_type == "pong":
          continue

      cloudlog.info(f"remote_agent.terminal.session.done session_id={session_id} exit_code={exit_code}")
    except WebSocketException as e:
      cloudlog.exception(f"remote_agent.terminal.ws_error session_id={session_id} error={e}")
    except Exception:
      cloudlog.exception(f"remote_agent.terminal.loop_exception session_id={session_id}")
    finally:
      if proc is not None and proc.poll() is None:
        try:
          proc.terminate()
          proc.wait(timeout=2)
        except Exception:
          try:
            proc.kill()
          except Exception:
            pass

      if master_fd is not None:
        try:
          os.close(master_fd)
        except OSError:
          pass

      if ws is not None:
        try:
          ws.close()
        except Exception:
          pass

      with self.terminal_workers_lock:
        self.terminal_workers.pop(session_id, None)

  def sync_terminal_sessions(self) -> None:
    if not self.cfg.terminal_enabled:
      return
    if self._is_onroad():
      return
    if self.license_status != "authorized":
      return

    with self.terminal_workers_lock:
      active = len(self.terminal_workers)
      if active >= self.cfg.terminal_max_sessions:
        return

    sessions = self.pull_terminal_sessions()
    for session in sessions:
      session_id = str(session.get("id", "")).strip()
      if not session_id:
        continue

      with self.terminal_workers_lock:
        if session_id in self.terminal_workers:
          continue
        if len(self.terminal_workers) >= self.cfg.terminal_max_sessions:
          break

        worker = threading.Thread(
          target=self._terminal_session_loop,
          args=(session_id,),
          daemon=True,
          name=f"remote_agent_terminal_{session_id[:8]}",
        )
        self.terminal_workers[session_id] = worker
        worker.start()

  def _execute_shell(self, command: str, timeout_sec: int) -> dict[str, Any]:
    if not self.cfg.allow_custom_commands and not self._is_command_allowed(command):
      return {
        "status": "failed",
        "exit_code": 126,
        "stdout": "",
        "stderr": f"command rejected by readonly policy: {command}",
        "duration_ms": 0,
      }

    started = time.monotonic()
    try:
      proc = subprocess.run(
        ["/bin/bash", "-lc", command],
        capture_output=True,
        text=True,
        timeout=max(timeout_sec, 1),
      )
      duration_ms = int((time.monotonic() - started) * 1000)
      return {
        "status": "success" if proc.returncode == 0 else "failed",
        "exit_code": proc.returncode,
        "stdout": _clip_text(proc.stdout or "", self.cfg.max_output_chars),
        "stderr": _clip_text(proc.stderr or "", self.cfg.max_output_chars),
        "duration_ms": duration_ms,
      }
    except subprocess.TimeoutExpired as e:
      duration_ms = int((time.monotonic() - started) * 1000)
      return {
        "status": "timeout",
        "exit_code": 124,
        "stdout": _clip_text(e.stdout or "", self.cfg.max_output_chars),
        "stderr": _clip_text((e.stderr or "") + "\ncommand timed out", self.cfg.max_output_chars),
        "duration_ms": duration_ms,
      }
    except Exception as e:
      duration_ms = int((time.monotonic() - started) * 1000)
      return {
        "status": "failed",
        "exit_code": 1,
        "stdout": "",
        "stderr": f"execution error: {e}",
        "duration_ms": duration_ms,
      }

  def execute_command(self, command: dict[str, Any]) -> dict[str, Any]:
    kind = str(command.get("kind", "shell"))
    timeout_sec = int(command.get("timeout_sec", 20))
    cmd = str(command.get("command", "")).strip()

    if kind != "shell":
      return {
        "status": "failed",
        "exit_code": 2,
        "stdout": "",
        "stderr": f"unsupported command kind: {kind}",
        "duration_ms": 0,
      }
    if not cmd:
      return {
        "status": "failed",
        "exit_code": 2,
        "stdout": "",
        "stderr": "empty command",
        "duration_ms": 0,
      }
    return self._execute_shell(cmd, timeout_sec)

  def sync_commands(self) -> None:
    if self._is_onroad():
      # Safety guard: do not execute remote commands while driving.
      return
    if self.license_status != "authorized":
      return

    commands = self.pull_commands()
    for cmd in commands:
      command_id = str(cmd.get("id", "")).strip()
      if not command_id:
        cloudlog.warning("remote_agent.command.skip_missing_id")
        continue

      result = self.execute_command(cmd)
      try:
        self.report_result(command_id, result)
      except Exception:
        cloudlog.exception(f"remote_agent.command.report_failed command_id={command_id}")

  def run_forever(self) -> None:
    if not self.cfg.base_url:
      cloudlog.warning("remote_agent.disabled_no_base_url")
      while True:
        time.sleep(60)

    cloudlog.info(f"remote_agent.start serial={self.serial} base_url={self.cfg.base_url}")
    while True:
      try:
        if not self.token:
          self.register()

        self.heartbeat()
        self.sync_error_log()
        self.sync_commands()
        self.sync_terminal_sessions()
      except requests.HTTPError as e:
        # Token may be invalid/expired; force re-register.
        status_code = getattr(e.response, "status_code", 0)
        if status_code in (401, 403):
          self.token = None
        cloudlog.exception(f"remote_agent.http_error status_code={status_code}")
      except Exception:
        cloudlog.exception("remote_agent.loop_exception")

      time.sleep(max(self.next_poll_sec, 5))


def main() -> None:
  agent = RemoteAgent(AgentConfig.from_env())
  agent.run_forever()


if __name__ == "__main__":
  main()
