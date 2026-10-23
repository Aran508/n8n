"""Flash firmware into a product through the USB-C programming dock, then read its self-test.

The product's firmware is expected to print a line containing SELFTEST OK on
its USB serial port within a few seconds of booting (the watch firmware
checks its OLED, buttons and charger before printing it).
"""

from __future__ import annotations

import subprocess
import time
from pathlib import Path
from typing import Callable


def make_flasher(port: str, firmware_dir: str | Path, chip: str = "esp32c3", log: Callable[[str], None] = print):
    firmware_dir = Path(firmware_dir)

    def flash(data: dict) -> bool:
        image = firmware_dir / data["firmware"]
        if not image.exists():
            log(f"firmware image {image} not found")
            return False
        cmd = ["esptool.py", "--chip", chip, "--port", port, "--baud", "921600", "write_flash", "0x0", str(image)]
        log("flashing: " + " ".join(cmd))
        if subprocess.run(cmd, capture_output=True, text=True, timeout=120).returncode != 0:
            return False
        return self_test(port, log=log)

    return flash


def self_test(port: str, timeout: float = 8.0, log: Callable[[str], None] = print) -> bool:
    import serial  # pyserial

    time.sleep(1.5)  # USB re-enumerates after the reset
    with serial.Serial(port, 115200, timeout=0.2) as ser:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            line = ser.readline().decode(errors="ignore").strip()
            if line:
                log(f"product: {line}")
            if "SELFTEST OK" in line:
                return True
            if "SELFTEST FAIL" in line:
                return False
    return False
