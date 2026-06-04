from __future__ import annotations

import logging
import threading
import time
from typing import Optional

logger = logging.getLogger(__name__)

# Single-char commands, newline-terminated
_CMD_UP = b"U\n"
_CMD_DOWN = b"D\n"
_CMD_STOP = b"S\n"
_CMD_ESTOP = b"E\n"
_CMD_START = b"R\n"


class ArduinoSerial:
    """Non-blocking serial manager for the Arduino actuator driver.

    Sends single-char commands (U/D/S/E/R + newline) to the Arduino.
    Auto-reconnects if the USB cable is unplugged and replugged.
    All sends are fire-and-forget; no ACK required.
    """

    def __init__(self, port: str, baud: int = 115200, reconnect_delay: float = 8.0) -> None:
        self.port = port
        self.baud = baud
        self.reconnect_delay = reconnect_delay
        self._ser = None
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None

    @property
    def connected(self) -> bool:
        with self._lock:
            return self._ser is not None and self._ser.is_open

    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(target=self._connect_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        with self._lock:
            if self._ser:
                try:
                    self._ser.close()
                except Exception:
                    pass
                self._ser = None

    # ---- Command helpers ------------------------------------------------

    def up(self) -> None:
        self._send(_CMD_UP)

    def down(self) -> None:
        self._send(_CMD_DOWN)

    def stop_motion(self) -> None:
        self._send(_CMD_STOP)

    def emergency_stop(self) -> None:
        self._send(_CMD_ESTOP)

    def resume(self) -> None:
        self._send(_CMD_START)

    # ---- Internal -------------------------------------------------------

    def _send(self, data: bytes) -> None:
        with self._lock:
            if self._ser and self._ser.is_open:
                try:
                    self._ser.write(data)
                    self._ser.flush()
                except Exception as exc:
                    logger.warning("Arduino write error: %s", exc)
                    try:
                        self._ser.close()
                    except Exception:
                        pass
                    self._ser = None
            else:
                logger.debug("Arduino not connected — dropping command: %s", data)

    @staticmethod
    def _find_port() -> Optional[str]:
        """Return the first USB serial port found (auto-detect)."""
        import glob
        for pattern in ("/dev/cu.usbmodem*", "/dev/ttyUSB*", "/dev/ttyACM*"):
            ports = sorted(glob.glob(pattern))
            if ports:
                return ports[0]
        return None

    def _connect_loop(self) -> None:
        while self._running:
            try:
                import serial  # pyserial; optional import so server starts without it

                port = self.port or self._find_port()
                if not port:
                    raise OSError("No USB serial port found")
                ser = serial.Serial(port, self.baud, timeout=0.1)
                time.sleep(2.0)  # wait for Arduino to finish booting after DTR reset
                logger.warning("Arduino connected on %s at %d baud", port, self.baud)
                with self._lock:
                    self._ser = ser

                while self._running:
                    if not ser.is_open:
                        break
                    time.sleep(0.5)

            except Exception as exc:
                logger.warning(
                    "Arduino not found on %s: %s — retrying in %.1fs",
                    self.port or "auto",
                    exc,
                    self.reconnect_delay,
                )
                with self._lock:
                    self._ser = None
                time.sleep(self.reconnect_delay)


_arduino: Optional[ArduinoSerial] = None


def init_arduino(port: str, baud: int = 115200) -> None:
    global _arduino
    _arduino = ArduinoSerial(port, baud)
    _arduino.start()
    logger.info("Arduino serial manager started (port=%s)", port)


def get_arduino() -> Optional[ArduinoSerial]:
    return _arduino
