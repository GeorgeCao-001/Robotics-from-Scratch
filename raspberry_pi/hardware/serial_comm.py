from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
import json
from math import isfinite
from typing import Any, Callable


JsonMessage = dict[str, Any]


@dataclass(frozen=True)
class SerialConfig:
    port: str
    baudrate: int = 115200
    timeout_s: float = 0.05
    write_timeout_s: float = 1.0
    newline: str = "\n"


def _is_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and isfinite(float(value))


class SerialComm:
    def __init__(
        self,
        config: SerialConfig,
        serial_port: Any | None = None,
        logger: Callable[[str], None] | None = None,
    ):
        self._config = config
        self._serial = serial_port
        self._logger = logger or print

    @property
    def is_open(self) -> bool:
        return bool(
            self._serial is not None and getattr(self._serial, "is_open", False)
        )

    def open(self) -> None:
        if self._serial is None:
            import serial

            self._serial = serial.Serial(
                port=self._config.port,
                baudrate=self._config.baudrate,
                timeout=self._config.timeout_s,
                write_timeout=self._config.write_timeout_s,
            )
            return

        if not self.is_open and hasattr(self._serial, "open"):
            self._serial.open()

    def close(self) -> None:
        if self._serial is not None and self.is_open:
            self._serial.close()

    def send_move(self, v: float, w: float) -> None:
        self.send_message({"cmd": "move", "v": float(v), "w": float(w)})

    def send_stop(self) -> None:
        self.send_move(0.0, 0.0)

    def enter_rpi_auto_mode(self) -> None:
        self._ensure_open()
        data = ("p" + self._config.newline).encode("utf-8")
        self._serial.write(data)
        self._log("TX", "p")

    def request_status(self) -> JsonMessage | None:
        self.send_message({"cmd": "status"})
        return self.read_message()

    def send_message(self, message: JsonMessage) -> None:
        self._ensure_open()
        self._validate_message(message)
        payload = json.dumps(message, separators=(",", ":"), ensure_ascii=True)
        data = (payload + self._config.newline).encode("utf-8")
        self._serial.write(data)
        self._log("TX", payload)

    def read_message(self) -> JsonMessage | None:
        self._ensure_open()
        raw = self._serial.readline()
        if not raw:
            return None

        text = raw.decode("utf-8", errors="replace").strip()
        if not text:
            return None

        self._log("RX", text)
        parsed = json.loads(text)
        if not isinstance(parsed, dict):
            raise ValueError("Incoming JSON must be an object")
        return parsed

    def _ensure_open(self) -> None:
        if not self.is_open:
            self.open()
        if not self.is_open:
            raise RuntimeError("Serial port is not open")

    def _validate_message(self, message: JsonMessage) -> None:
        cmd = message.get("cmd")
        if cmd not in {"move", "status", "mode"}:
            raise ValueError("Unsupported cmd")

        if cmd == "status":
            return

        if cmd == "mode":
            mode = message.get("mode")
            if mode not in {"rpi_auto", "remote"}:
                raise ValueError("mode command requires mode=rpi_auto or mode=remote")
            return

        if cmd == "move":
            v = message.get("v")
            w = message.get("w")
            if not _is_number(v) or not _is_number(w):
                raise ValueError("move command requires numeric v and w")
            return

    def _log(self, direction: str, message: str) -> None:
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self._logger(f"[{timestamp}] [{direction}] {message}")
