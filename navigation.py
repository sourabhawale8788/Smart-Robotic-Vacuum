"""
navigation.py — Navigation Controller & State Machine
======================================================
Project  : Smart Robotic Vacuum Cleaner with Object Detection
Platform : Raspberry Pi 4B

Fuses vision results (ObstacleDetector) with ESP32 sensor telemetry
and drives the motor state machine via UARTBridge.

States
------
  FORWARD  : Default — move forward and vacuum
  AVOID_L  : Turn left for AVOID_DURATION seconds
  AVOID_R  : Turn right for AVOID_DURATION seconds
  BACKUP   : Reverse briefly before turning
  STOP     : Emergency stop (collision or low battery)

Transition priority (highest first):
  1. EVT:COLLISION from ESP32  → BACKUP → AVOID
  2. front_cm < SENSOR_THRESH  → AVOID (sensor-driven)
  3. Vision CENTRE close        → AVOID (vision-driven)
  4. Vision clear               → FORWARD
"""

import time
import logging
from typing import Dict, Optional

from uart_bridge import UARTBridge
from obstacle_detector import DetectionResult

log = logging.getLogger(__name__)


# ──────────────────────────────────────────────
# Tuneable constants
# ──────────────────────────────────────────────

SENSOR_THRESH_CM = 20       # ultrasonic/IR front distance → trigger avoid
AVOID_DURATION_S = 1.2      # seconds to spend turning
BACKUP_DURATION_S = 0.6     # seconds to reverse before turning


class NavigationState:
    FORWARD = "FORWARD"
    AVOID_L = "AVOID_L"
    AVOID_R = "AVOID_R"
    BACKUP  = "BACKUP"
    STOP    = "STOP"


class NavigationController:
    """
    State machine that translates fused sensor+vision data into
    UART motor commands.

    Parameters
    ----------
    uart                  : UARTBridge — active serial bridge to ESP32
    obstacle_threshold_cm : int        — vision depth threshold to trigger avoidance
    base_speed            : int        — PWM duty cycle 0–255 for forward motion
    """

    def __init__(
        self,
        uart: UARTBridge,
        obstacle_threshold_cm: int = 25,
        base_speed: int            = 180,
    ) -> None:
        self._uart          = uart
        self._obs_thresh    = obstacle_threshold_cm
        self._base_speed    = base_speed

        self._state         = NavigationState.FORWARD
        self._state_entered = time.time()
        self._last_turn_dir = "LEFT"   # alternate turns to avoid looping

        log.info(
            f"NavigationController ready — "
            f"obs_thresh={obstacle_threshold_cm}cm, "
            f"base_speed={base_speed}"
        )

    # ── Public API ────────────────────────────

    @property
    def current_state(self) -> str:
        return self._state

    def update(
        self,
        vision_result: DetectionResult,
        telemetry: Dict[str, int],
    ) -> None:
        """
        Called once per camera frame. Evaluates all inputs and
        drives state transitions + UART commands.
        """
        # 1. Poll for ESP32 hardware events (highest priority)
        event = self._uart.poll_event()
        if event == "COLLISION":
            self._transition(NavigationState.BACKUP)
            return

        now = time.time()
        time_in_state = now - self._state_entered

        # 2. Timed-state completions
        if self._state == NavigationState.BACKUP:
            if time_in_state >= BACKUP_DURATION_S:
                # Choose turn direction (alternate to prevent corner trapping)
                turn = self._pick_turn(vision_result, telemetry)
                self._transition(turn)
            return

        if self._state in (NavigationState.AVOID_L, NavigationState.AVOID_R):
            if time_in_state >= AVOID_DURATION_S:
                self._transition(NavigationState.FORWARD)
            return

        # 3. Sensor-driven avoidance (overrides vision)
        front_cm = telemetry.get("front_cm", 999)
        if front_cm < SENSOR_THRESH_CM:
            log.debug(f"Sensor trigger: front_cm={front_cm}")
            self._transition(NavigationState.BACKUP)
            return

        # 4. Vision-driven avoidance
        if not vision_result.clear_path:
            closest = self._closest_obstacle_cm(vision_result)
            if closest < self._obs_thresh:
                log.debug(
                    f"Vision trigger: closest={closest}cm "
                    f"action={vision_result.recommended_action}"
                )
                target_state = (
                    NavigationState.AVOID_L
                    if vision_result.recommended_action == "LEFT"
                    else NavigationState.AVOID_R
                )
                self._transition(target_state)
                return

        # 5. Default: forward
        if self._state != NavigationState.FORWARD:
            self._transition(NavigationState.FORWARD)

    # ── Private helpers ───────────────────────

    def _transition(self, new_state: str) -> None:
        """Execute state transition: update state, send motor commands."""
        if self._state == new_state:
            return

        old_state    = self._state
        self._state  = new_state
        self._state_entered = time.time()
        log.info(f"State: {old_state} → {new_state}")

        if new_state == NavigationState.FORWARD:
            self._uart.send_command(f"SPD:{self._base_speed}")
            self._uart.send_command("FWD")

        elif new_state == NavigationState.BACKUP:
            self._uart.send_command(f"SPD:{int(self._base_speed * 0.7)}")
            self._uart.send_command("REV")

        elif new_state == NavigationState.AVOID_L:
            self._last_turn_dir = "LEFT"
            self._uart.send_command(f"SPD:{int(self._base_speed * 0.8)}")
            self._uart.send_command("LEFT")

        elif new_state == NavigationState.AVOID_R:
            self._last_turn_dir = "RIGHT"
            self._uart.send_command(f"SPD:{int(self._base_speed * 0.8)}")
            self._uart.send_command("RIGHT")

        elif new_state == NavigationState.STOP:
            self._uart.send_command("STOP")

    def _pick_turn(
        self,
        vision_result: DetectionResult,
        telemetry: Dict[str, int],
    ) -> str:
        """
        Choose AVOID_L or AVOID_R after a backup by comparing:
          - Sensor data (left_cm vs right_cm)
          - Vision recommendation
          - Alternation flag as tiebreaker
        """
        left_cm  = telemetry.get("left_cm",  999)
        right_cm = telemetry.get("right_cm", 999)

        if abs(left_cm - right_cm) > 10:
            # Clear sensor preference
            return NavigationState.AVOID_L if left_cm > right_cm else NavigationState.AVOID_R

        vis_action = vision_result.recommended_action
        if vis_action == "LEFT":
            return NavigationState.AVOID_L
        if vis_action == "RIGHT":
            return NavigationState.AVOID_R

        # Alternate to prevent looping in corners
        if self._last_turn_dir == "LEFT":
            return NavigationState.AVOID_R
        return NavigationState.AVOID_L

    @staticmethod
    def _closest_obstacle_cm(result: DetectionResult) -> int:
        """Return the minimum depth estimate across all detected obstacles."""
        if not result.obstacles:
            return 999
        return min(obs.depth_estimate_cm for obs in result.obstacles)
