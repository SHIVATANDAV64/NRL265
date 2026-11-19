#!/usr/bin/env python3
import sys, time, subprocess, tempfile
from pathlib import Path

# Import from existing flash_controller
sys.path.insert(0, str(Path(__file__).parent))
import flash_controller

MERGED = Path(__file__).resolve().parent / "prebuilt" / "controller_merged.bin"
PORT = "COM10"
BAUD = "460800"

print(f"[NRL Auto-Flasher] Monitoring {PORT}...")
print("Please hold one button, tap the other button, and release.")
print("Waiting for ESP32-S3 download mode...")

with tempfile.TemporaryDirectory() as tmp:
    head, tail = flash_controller.split_image(MERGED, tmp)
    
    connected = False
    for attempt in range(120): # wait up to 60 seconds
        res = subprocess.run(
            [sys.executable, "-m", "esptool", "--chip", "esp32s3", "--port", PORT, "--baud", "115200", "--before", "no_reset", "chip_id"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        if "Chip type: ESP32-S3" in res.stdout or "MAC:" in res.stdout:
            connected = True
            print("\n>>> ESP32-S3 DETECTED IN DOWNLOAD MODE! <<<")
            break
        time.sleep(0.5)
        print(".", end="", flush=True)

    if not connected:
        print("\nTimed out waiting for download mode.")
        sys.exit(1)

    print(f"\nFlashing controller firmware at {BAUD} baud...")
    cmd = [
        sys.executable, "-m", "esptool", "--chip", "esp32s3", "--port", PORT, "--baud", BAUD,
        "--before", "no_reset", "--after", "no_reset",
        "write_flash", "0x0", str(head), f"0x{flash_controller.NVS_END:x}", str(tail)
    ]
    res2 = subprocess.run(cmd)
    if res2.returncode == 0:
        print("\n" + "="*50)
        print(">>> FLASH SUCCESSFUL! <<<")
        print("Now press the RESET button on the controller to boot the firmware!")
        print("="*50)
    else:
        print(f"Flash failed with code {res2.returncode}")
        sys.exit(res2.returncode)
