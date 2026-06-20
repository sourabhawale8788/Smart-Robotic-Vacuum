#!/usr/bin/env python3
"""
Smart Robotic Vacuum Cleaner — Raspberry Pi Main Entry Point
=============================================================
Project  : Smart Robotic Vacuum Cleaner with Object Detection
Platform : Raspberry Pi 4B
Author   : [Your Team]
Date     : 2024-2025

Description:
    Main control loop coordinating:
      - ESP32-CAM MJPEG video capture
      - OpenCV obstacle detection pipeline
      - UART command dispatch to ESP32 firmware
      - Dynamic path re-routing

Run:
    python3 main.py [--cam-port /dev/ttyUSB0] [--uart-port /dev/ttyS0]
"""

import cv2
import time
import argparse
import logging
import signal
import sys

from obstacle_detector import ObstacleDetector
from uart_bridge import UARTBridge
from navigation import NavigationController

# ──────────────────────────────────────────────
# Logging setup
# ──────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="[%(levelname)s %(asctime)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("Main")


# ──────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────
DEFAULT_CAM_PORT   = "/dev/ttyUSB0"   # ESP32-CAM serial port
DEFAULT_UART_PORT  = "/dev/ttyS0"     # ESP32 command/telemetry UART
CAM_WIDTH          = 320
CAM_HEIGHT         = 240
TARGET_FPS         = 15
OBSTACLE_THRESHOLD = 25               # cm — if closer, reroute
LOOP_PERIOD_S      = 1.0 / TARGET_FPS


# ──────────────────────────────────────────────
# Graceful shutdown handler
# ──────────────────────────────────────────────
_running = True

def _shutdown_handler(sig, frame):
    global _running
    log.info("Shutdown signal received — stopping robot.")
    _running = False

signal.signal(signal.SIGINT,  _shutdown_handler)
signal.signal(signal.SIGTERM, _shutdown_handler)


# ──────────────────────────────────────────────
# Main control loop
# ──────────────────────────────────────────────
def main(cam_port: str, uart_port: str, headless: bool) -> None:
    log.info("Initialising subsystems …")

    # 1. UART bridge to ESP32
    uart = UARTBridge(port=uart_port, baud=115200, timeout=0.05)
    uart.connect()
    log.info(f"UART bridge open on {uart_port}")

    # 2. OpenCV capture — ESP32-CAM MJPEG stream via USB serial
    #    The ESP32-CAM firmware streams MJPEG over serial at 921600 baud.
    #    OpenCV reads it as a standard VideoCapture URL.
    cap = cv2.VideoCapture(0)          # change to 1/2 if needed
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAM_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS,          TARGET_FPS)

    if not cap.isOpened():
        log.error("Cannot open camera — check connection and port number.")
        uart.disconnect()
        sys.exit(1)

    log.info("Camera stream open.")

    # 3. Vision pipeline
    detector = ObstacleDetector(
        frame_width=CAM_WIDTH,
        frame_height=CAM_HEIGHT,
        obstacle_area_min=400,         # px² — ignore noise below this
    )

    # 4. Navigation controller
    nav = NavigationController(
        uart=uart,
        obstacle_threshold_cm=OBSTACLE_THRESHOLD,
        base_speed=180,                # PWM duty 0–255
    )

    # 5. Robot start
    uart.send_command("SPD:180")
    uart.send_command("FWD")
    uart.send_command("CLEAN:ON")
    log.info("Robot started — entering main loop.")

    frame_count   = 0
    fps_clock     = time.time()
    last_telemetry: dict = {}

    while _running:
        loop_start = time.perf_counter()

        # ── A. Capture frame ──────────────────
        ret, frame = cap.read()
        if not ret:
            log.warning("Frame grab failed — retrying …")
            time.sleep(0.05)
            continue

        # ── B. Read ESP32 telemetry (non-blocking) ──
        tel = uart.read_telemetry()
        if tel:
            last_telemetry = tel

        # ── C. Obstacle detection ─────────────
        result = detector.process(frame)

        # ── D. Navigation decision ────────────
        nav.update(
            vision_result=result,
            telemetry=last_telemetry,
        )

        # ── E. Display (dev mode) ─────────────
        if not headless:
            annotated = detector.draw_annotations(frame.copy(), result)
            cv2.imshow("Smart Vacuum — Vision", annotated)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        # ── F. FPS throttle ───────────────────
        frame_count += 1
        elapsed = time.perf_counter() - loop_start
        sleep_t = max(0.0, LOOP_PERIOD_S - elapsed)
        time.sleep(sleep_t)

        # Log FPS every 5 seconds
        if time.time() - fps_clock >= 5.0:
            fps = frame_count / 5.0
            log.info(
                f"FPS={fps:.1f} | "
                f"Obstacles={result.get('obstacle_count', 0)} | "
                f"State={nav.current_state} | "
                f"Tel={last_telemetry}"
            )
            frame_count = 0
            fps_clock   = time.time()

    # ── Cleanup ───────────────────────────────
    log.info("Stopping robot …")
    uart.send_command("STOP")
    uart.send_command("CLEAN:OFF")
    cap.release()
    cv2.destroyAllWindows()
    uart.disconnect()
    log.info("Clean shutdown complete.")


# ──────────────────────────────────────────────
# CLI
# ──────────────────────────────────────────────
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Smart Robotic Vacuum — Raspberry Pi controller"
    )
    parser.add_argument(
        "--cam-port",
        default=DEFAULT_CAM_PORT,
        help=f"ESP32-CAM serial port (default: {DEFAULT_CAM_PORT})",
    )
    parser.add_argument(
        "--uart-port",
        default=DEFAULT_UART_PORT,
        help=f"ESP32 command UART port (default: {DEFAULT_UART_PORT})",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Disable OpenCV display window (use on headless Pi)",
    )
    args = parser.parse_args()

    main(
        cam_port=args.cam_port,
        uart_port=args.uart_port,
        headless=args.headless,
    )
