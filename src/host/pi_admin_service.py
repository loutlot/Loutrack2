from __future__ import annotations

import os
import posixpath
import shlex
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, Protocol


class _SFTPLike(Protocol):
    def mkdir(self, path: str) -> None: ...
    def put(self, localpath: str, remotepath: str) -> None: ...
    def close(self) -> None: ...


class _SSHLike(Protocol):
    def set_missing_host_key_policy(self, policy: Any) -> None: ...
    def connect(self, **kwargs: Any) -> None: ...
    def exec_command(self, command: str, timeout: float | None = None) -> Any: ...
    def open_sftp(self) -> _SFTPLike: ...
    def close(self) -> None: ...


@dataclass(frozen=True)
class PiAdminConfig:
    ssh_user: str = "pi"
    ssh_key_path: Path = Path("~/.ssh/loutrack_deploy_key")
    ssh_port: int = 22
    timeout_s: float = 8.0
    service_name: str = "loutrack.service"
    remote_user: str = "pi"
    remote_base: str = "/opt/loutrack"
    release_keep: int = 3

    @property
    def expanded_key_path(self) -> Path:
        return Path(os.path.expanduser(str(self.ssh_key_path)))


class PiAdminService:
    """SSH-backed Pi service and release management for the host GUI."""

    def __init__(
        self,
        project_root: Path,
        config: PiAdminConfig | None = None,
        *,
        ssh_client_factory: Any | None = None,
        pkey_loader: Any | None = None,
    ) -> None:
        self.project_root = Path(project_root)
        self.config = config or PiAdminConfig()
        self._ssh_client_factory = ssh_client_factory
        self._pkey_loader = pkey_loader

    def status(self, targets: Iterable[Any]) -> Dict[str, Dict[str, Any]]:
        return {target.camera_id: self._status_one(target) for target in targets}

    def run_action(self, targets: Iterable[Any], action: str) -> Dict[str, Dict[str, Any]]:
        normalized = str(action or "").strip()
        allowed = {"service_start", "service_stop", "reboot", "shutdown", "update", "rollback"}
        if normalized not in allowed:
            raise ValueError(f"unsupported pi admin action: {normalized}")
        return {target.camera_id: self._action_one(target, normalized) for target in targets}

    def _status_one(self, target: Any) -> Dict[str, Any]:
        ssh = None
        try:
            ssh = self._connect(str(target.ip))
            active = self._run(ssh, f"systemctl is-active {self.config.service_name}", check=False)
            enabled = self._run(ssh, f"systemctl is-enabled {self.config.service_name}", check=False)
            ptp = self._run(
                ssh,
                "systemctl is-active loutrack-ptp4l.service ptp4l-master.service ptp4l-slave.service 2>/dev/null",
                check=False,
            )
            uptime = self._run(ssh, "uptime -p", check=False)
            temp = self._run(
                ssh,
                "cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null || true",
                check=False,
            )
            disk = self._run(ssh, "df -h /opt / | tail -n 1", check=False)
            release = self._run(ssh, f"readlink -f {self.config.remote_base}/current 2>/dev/null || true", check=False)
            journal = self._run(
                ssh,
                f"journalctl -u {self.config.service_name} -n 20 --no-pager 2>/dev/null",
                check=False,
            )
            return {
                "ok": True,
                "ssh_reachable": True,
                "service_active": active.stdout.strip(),
                "service_enabled": enabled.stdout.strip(),
                "ptp_active": ptp.stdout.strip(),
                "uptime": uptime.stdout.strip(),
                "cpu_temp_c": self._parse_temp_c(temp.stdout),
                "disk": disk.stdout.strip(),
                "release": self._release_name(release.stdout.strip()),
                "journal_tail": journal.stdout.strip(),
            }
        except Exception as exc:  # noqa: BLE001
            return self._error_result(exc)
        finally:
            if ssh is not None:
                ssh.close()

    def _action_one(self, target: Any, action: str) -> Dict[str, Any]:
        ssh = None
        try:
            ssh = self._connect(str(target.ip))
            if action == "service_start":
                result = self._run(ssh, f"sudo -n systemctl restart {self.config.service_name}", check=True)
            elif action == "service_stop":
                result = self._run(ssh, f"sudo -n systemctl stop {self.config.service_name}", check=True)
            elif action == "reboot":
                result = self._run_disconnect_ok(ssh, "sudo -n reboot")
            elif action == "shutdown":
                result = self._run_disconnect_ok(ssh, "sudo -n shutdown -h now")
            elif action == "update":
                return self._update(ssh, str(target.camera_id))
            elif action == "rollback":
                return self._rollback(ssh)
            else:
                raise ValueError(f"unsupported pi admin action: {action}")
            return {"ok": True, "action": action, "stdout": result.stdout.strip(), "stderr": result.stderr.strip()}
        except Exception as exc:  # noqa: BLE001
            payload = self._error_result(exc)
            payload["action"] = action
            return payload
        finally:
            if ssh is not None:
                ssh.close()

    def _update(self, ssh: _SSHLike, camera_id: str) -> Dict[str, Any]:
        version = time.strftime("gui-%Y%m%d%H%M%S")
        remote_release = posixpath.join(self.config.remote_base, "releases", version)
        remote_src = posixpath.join(remote_release, "src")
        self._run(ssh, f"sudo -n mkdir -p {shlex.quote(remote_src + '/pi')} {shlex.quote(remote_src + '/camera-calibration')}", check=True)
        self._run(ssh, f"sudo -n chown -R {self.config.remote_user}:{self.config.remote_user} {shlex.quote(self.config.remote_base)}", check=True)
        sftp = ssh.open_sftp()
        try:
            self._upload_tree(sftp, self.project_root / "src" / "pi", posixpath.join(remote_src, "pi"))
            self._upload_tree(
                sftp,
                self.project_root / "src" / "camera-calibration",
                posixpath.join(remote_src, "camera-calibration"),
            )
        finally:
            sftp.close()
        self._run(ssh, f"ln -sfn {shlex.quote(remote_release)} {shlex.quote(posixpath.join(self.config.remote_base, 'current'))}", check=True)
        self._install_service(ssh, camera_id)
        self._run(ssh, f"sudo -n systemctl restart {self.config.service_name}", check=True)
        self._cleanup_releases(ssh)
        return {"ok": True, "action": "update", "release": version}

    def _rollback(self, ssh: _SSHLike) -> Dict[str, Any]:
        releases_dir = posixpath.join(self.config.remote_base, "releases")
        cmd = f"cd {shlex.quote(releases_dir)} && ls -1dt */ 2>/dev/null | sed -n '2p'"
        previous = self._run(ssh, cmd, check=True).stdout.strip().strip("/")
        if not previous:
            raise RuntimeError("rollback_unavailable")
        target_release = posixpath.join(releases_dir, previous)
        self._run(ssh, f"ln -sfn {shlex.quote(target_release)} {shlex.quote(posixpath.join(self.config.remote_base, 'current'))}", check=True)
        self._run(ssh, f"sudo -n systemctl restart {self.config.service_name}", check=True)
        return {"ok": True, "action": "rollback", "release": previous}

    def _install_service(self, ssh: _SSHLike, camera_id: str) -> None:
        current = posixpath.join(self.config.remote_base, "current")
        remote_src_dir = posixpath.join(current, "src")
        service = f"""[Unit]
Description=Loutrack camera capture
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User={self.config.remote_user}
WorkingDirectory={current}
Environment=PYTHONUNBUFFERED=1
Environment=PYTHONDONTWRITEBYTECODE=1
ExecStart=/usr/bin/python3 {remote_src_dir}/pi/service/capture_runtime.py --camera-id {camera_id} --udp-dest 255.255.255.255:5000 --sync-role auto
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
"""
        quoted = shlex.quote(service)
        self._run(ssh, f"printf %s {quoted} | sudo -n tee /etc/systemd/system/{self.config.service_name} >/dev/null", check=True)
        self._run(ssh, f"sudo -n systemctl daemon-reload && sudo -n systemctl enable {self.config.service_name} >/dev/null", check=True)

    def _cleanup_releases(self, ssh: _SSHLike) -> None:
        releases_dir = posixpath.join(self.config.remote_base, "releases")
        cmd = (
            f"cd {shlex.quote(releases_dir)} && "
            f"ls -1dt */ 2>/dev/null | tail -n +{self.config.release_keep + 1} | xargs -r rm -rf"
        )
        self._run(ssh, cmd, check=False)

    def _upload_tree(self, sftp: _SFTPLike, local_root: Path, remote_root: str) -> None:
        self._mkdir_p(sftp, remote_root)
        for current_root, dirs, files in os.walk(local_root):
            dirs[:] = [name for name in dirs if name != "__pycache__"]
            rel = Path(current_root).relative_to(local_root)
            remote_dir = remote_root if str(rel) == "." else posixpath.join(remote_root, *rel.parts)
            self._mkdir_p(sftp, remote_dir)
            for filename in files:
                if filename.endswith(".pyc"):
                    continue
                local_file = Path(current_root) / filename
                remote_file = posixpath.join(remote_dir, filename)
                sftp.put(str(local_file), remote_file)

    @staticmethod
    def _mkdir_p(sftp: _SFTPLike, remote_path: str) -> None:
        parts = [part for part in remote_path.split("/") if part]
        current = ""
        for part in parts:
            current = current + "/" + part
            try:
                sftp.mkdir(current)
            except OSError:
                continue

    def _connect(self, ip: str) -> _SSHLike:
        key_path = self.config.expanded_key_path
        if not key_path.exists():
            raise FileNotFoundError(f"ssh_key_missing: {key_path}")
        paramiko = self._load_paramiko()
        pkey = self._load_pkey(paramiko, key_path)
        client = self._new_client(paramiko)
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        client.connect(
            hostname=ip,
            port=self.config.ssh_port,
            username=self.config.ssh_user,
            pkey=pkey,
            timeout=self.config.timeout_s,
            banner_timeout=self.config.timeout_s,
            auth_timeout=self.config.timeout_s,
            look_for_keys=False,
            allow_agent=False,
        )
        return client

    def _new_client(self, paramiko: Any) -> _SSHLike:
        if self._ssh_client_factory is not None:
            return self._ssh_client_factory()
        return paramiko.SSHClient()

    def _load_pkey(self, paramiko: Any, key_path: Path) -> Any:
        if self._pkey_loader is not None:
            return self._pkey_loader(key_path)
        loaders = [
            getattr(paramiko, "Ed25519Key", None),
            getattr(paramiko, "RSAKey", None),
            getattr(paramiko, "ECDSAKey", None),
        ]
        last_exc: Exception | None = None
        for loader in loaders:
            if loader is None:
                continue
            try:
                return loader.from_private_key_file(str(key_path))
            except Exception as exc:  # noqa: BLE001
                last_exc = exc
        raise RuntimeError(f"ssh_key_load_failed: {last_exc}")

    @staticmethod
    def _load_paramiko() -> Any:
        try:
            import paramiko  # type: ignore[import-not-found]
        except ImportError as exc:
            raise RuntimeError("paramiko_missing") from exc
        return paramiko

    def _run_disconnect_ok(self, ssh: _SSHLike, command: str) -> Any:
        try:
            return self._run(ssh, command, check=True)
        except Exception as exc:  # noqa: BLE001
            text = str(exc).lower()
            if "socket" in text or "closed" in text or "reset" in text:
                return _CommandResult(0, "", "")
            raise

    def _run(self, ssh: _SSHLike, command: str, *, check: bool) -> Any:
        stdin, stdout, stderr = ssh.exec_command(command, timeout=self.config.timeout_s)
        _ = stdin
        out = self._read_stream(stdout)
        err = self._read_stream(stderr)
        status_code = int(stdout.channel.recv_exit_status())
        result = _CommandResult(status_code, out, err)
        if check and status_code != 0:
            raise RuntimeError(self._command_error_code(command, result))
        return result

    @staticmethod
    def _read_stream(stream: Any) -> str:
        data = stream.read()
        if isinstance(data, bytes):
            return data.decode("utf-8", errors="replace")
        return str(data or "")

    @staticmethod
    def _command_error_code(command: str, result: Any) -> str:
        combined = f"{result.stdout}\n{result.stderr}".lower()
        if "a password is required" in combined or "sudo:" in combined and "password" in combined:
            return "sudo_requires_password"
        return f"command_failed exit={result.exit_status} cmd={command} stderr={result.stderr.strip()}"

    @staticmethod
    def _parse_temp_c(raw: str) -> float | None:
        text = raw.strip()
        if not text:
            return None
        try:
            value = float(text.splitlines()[0])
        except ValueError:
            return None
        return round(value / 1000.0, 1) if value > 200 else round(value, 1)

    @staticmethod
    def _release_name(path: str) -> str:
        if not path:
            return ""
        return posixpath.basename(path.rstrip("/"))

    @staticmethod
    def _error_result(exc: Exception) -> Dict[str, Any]:
        text = str(exc)
        if isinstance(exc, FileNotFoundError) and "ssh_key_missing" in text:
            code = "ssh_key_missing"
        elif "Authentication" in type(exc).__name__ or "auth" in text.lower():
            code = "ssh_auth_failed"
        elif "sudo_requires_password" in text:
            code = "sudo_requires_password"
        elif "paramiko_missing" in text:
            code = "paramiko_missing"
        else:
            code = "ssh_or_command_failed"
        return {"ok": False, "ssh_reachable": False, "error_code": code, "error": text}


@dataclass(frozen=True)
class _CommandResult:
    exit_status: int
    stdout: str
    stderr: str
