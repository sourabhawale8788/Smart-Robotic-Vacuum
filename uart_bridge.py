"""
uart_bridge.py — UART Serial Communication Bridge
==================================================
Project  : Smart Robotic Vacuum Cleaner with Object Detection
Platform : Raspberry Pi 4B

Handles all serial communication between the Raspberry Pi (Python)
and the ESP32 firmware.

Protocol:
  RPi  → ESP32 : "CMD:<command>\\n"
  ESP32 → RPi  : "TEL:<front_cm>,<left_cm>,<right_cm>,<speed>\\n"
                 "EVT:<event_name>\\n"
"""

import serial
import threading
import queue
import logging
import time
from typing import Optional, Dict

log = logging.getLogger(__name__)


class UARTBridge:
    """
    Thread-safe UART bridge between Raspberry Pi and ESP32.

    - A background reader thread buffers all incoming lines from ESP32.
    - Telemetry and event frames are parsed and stored.
    - `send_command()` is safe to call from any thread.

    Parameters
    ----------
    port    : str   — e.g. "/dev/ttyS0" or "/dev/ttyAMA0"
    baud    : int   — must match ESP32 UART config (default 115200)
    timeout : float — read timeout in seconds
    """

    _CMD_PREFIX = "CMD:"
    _TEL_PREFIX = "TEL:"
    _EVT_PREFIX = "EVT:"

    def __init__(
        self,
        port: str   = "/dev/ttyS0",
        baud: int   = 115200,
        timeout: float = 0.05,
    ) -> None:
        self._port    = port
        self._baud    = baud
        self._timeout = timeout

        self._serial: Optional[serial.Serial] = None
        self._lock    = threading.Lock()
        self._running = False

        # Latest parsed telemetry
        self._telemetry: Dict[str, int] = {}
        self._telemetry_lock = threading.Lock()

        # Event queue (EVT: frames from ESP32)
        self._event_queue: queue.Queue = queue.Queue(maxsize=20)

        # Reader thread
        self._reader_thread: Optional[threading.Thread] = None

    # ── Lifecycle ────────────────────────────

    def connect(self) -> None:
        """Open the serial port and start the background reader thread."""
        self._serial = serial.Serial(
            port=self._port,
            baudrate=self._baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self._timeout,
        )
        self._running = True
        self._reader_thread = threading.Thread(
            target=self._reader_loop,
            name="UARTReader",
            daemon=True,
        )
        self._reader_thread.start()
        log.info(f"UART connected: {self._port} @ {self._baud} baud")

    def disconnect(self) -> None:
        """Stop reader thread and close the serial port."""
        self._running = False
        if self._reader_thread:
            self._reader_thread.join(timeout=1.0)
        if self._serial and self._serial.is_open:
            self._serial.close()
        log.info("UART disconnected.")

    # ── Send ─────────────────────────────────

    def send_command(self, command: str) -> bool:
        """
        Send a command to the ESP32.

        Frame format: "CMD:<command>\\n"
        Example: send_command("FWD") → writes b"CMD:FWD\\n"

        Returns True if write succeeded, False otherwise.
        """
        if not self._serial or not self._serial.is_open:
            log.error("Cannot send — serial port not open.")
            return False

        frame = f"{self._CMD_PREFIX}{command}\n".encode("ascii")
        try:
            with self._lock:
                self._serial.write(frame)
            log.debug(f"TX → {frame.strip()}")
            return True
        except serial.SerialException as exc:
            log.error(f"UART write error: {exc}")
            return False

    # ── Receive ──────────────────────────────

    def read_telemetry(self) -> Dict[str, int]:
        """
        Return the most recently received telemetry snapshot.

        Keys: front_cm, left_cm, right_cm, speed
        Returns {} if no telemetry received yet.
        """
        with self._telemetry_lock:
            return dict(self._telemetry)

    def poll_event(self) -> Optional[str]:
        """
        Return the next unread event from the ESP32, or None if the queue is empty.

        Example return values: "COLLISION", "BUMPER_HIT", "LOW_BATTERY"
        """
        try:
            return self._event_queue.get_nowait()
        except queue.Empty:
            return None

    # ── Background reader ─────────────────────

    def _reader_loop(self) -> None:
        """
        Background thread: continuously reads lines from the serial port
        and dispatches them to telemetry or event handlers.
        """
        buf = b""
        while self._running:
            try:
                if not self._serial or not self._serial.is_open:
                    time.sleep(0.01)
                    continue

                chunk = self._serial.read(64)
                if not chunk:
                    continue

                buf += chunk
                while b"\n" in buf:
                    line_bytes, buf = buf.split(b"\n", 1)
                    line = line_bytes.decode("ascii", errors="ignore").strip()
                    if line:
                        self._dispatch_line(line)

            except serial.SerialException as exc:
                log.warning(f"UART read error: {exc}")
                time.sleep(0.05)

    def _dispatch_line(self, line: str) -> None:
        """Parse a received line and update internal state."""
        log.debug(f"RX ← {line}")

        if line.startswith(self._TEL_PREFIX):
            self._parse_telemetry(line[len(self._TEL_PREFIX):])
        elif line.startswith(self._EVT_PREFIX):
            event = line[len(self._EVT_PREFIX):]
            log.info(f"ESP32 EVENT: {event}")
            try:
                self._event_queue.put_nowait(event)
            except queue.Full:
                pass   # drop oldest implicitly (queue maxsize=20)

    def _parse_telemetry(self, payload: str) -> None:
        """
        Parse telemetry payload: "<front_cm>,<left_cm>,<right_cm>,<speed>"
        Updates self._telemetry atomically.
        """
        try:
            parts = payload.split(",")
            if len(parts) < 4:
                return
            tel = {
                "front_cm": int(parts[0]),
                "left_cm":  int(parts[1]),
                "right_cm": int(parts[2]),
                "speed":    int(parts[3]),
            }
            with self._telemetry_lock:
                self._telemetry = tel
        except (ValueError, IndexError) as exc:
            log.debug(f"Telemetry parse error: {exc} — payload='{payload}'")
