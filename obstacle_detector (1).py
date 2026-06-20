"""
obstacle_detector.py — OpenCV Vision Pipeline
==============================================
Project  : Smart Robotic Vacuum Cleaner with Object Detection
Platform : Raspberry Pi 4B + ESP32-CAM

Pipeline stages:
  1. BGR → Greyscale → Gaussian blur
  2. MOG2 background subtraction (adaptive)
  3. Morphological open/close to remove noise
  4. Contour detection + bounding-box filtering
  5. Obstacle zone classification (LEFT / CENTRE / RIGHT)
  6. Depth heuristic: contour area inversely proportional to distance
"""

import cv2
import numpy as np
import logging
from dataclasses import dataclass, field
from typing import List, Tuple, Optional

log = logging.getLogger(__name__)


# ──────────────────────────────────────────────
# Data classes
# ──────────────────────────────────────────────

@dataclass
class BoundingBox:
    x: int
    y: int
    w: int
    h: int
    area: int
    zone: str              # "LEFT" | "CENTRE" | "RIGHT"
    depth_estimate_cm: int  # heuristic depth from camera


@dataclass
class DetectionResult:
    obstacle_count: int = 0
    obstacles: List[BoundingBox] = field(default_factory=list)
    dominant_zone: Optional[str] = None   # zone with biggest obstacle
    clear_path: bool = True
    recommended_action: str = "FWD"       # FWD | LEFT | RIGHT | STOP


# ──────────────────────────────────────────────
# ObstacleDetector
# ──────────────────────────────────────────────

class ObstacleDetector:
    """
    Real-time obstacle detector for the vacuum robot camera feed.

    Parameters
    ----------
    frame_width      : int   — camera horizontal resolution
    frame_height     : int   — camera vertical resolution
    obstacle_area_min: int   — minimum contour area (px²) to consider valid
    history          : int   — MOG2 background model history length (frames)
    var_threshold    : float — MOG2 pixel variance threshold
    """

    # Depth heuristic calibration constants
    # Derived empirically: area_ref px² → depth_ref cm at the test distance
    _AREA_REF_PX   = 8000
    _DEPTH_REF_CM  = 30

    def __init__(
        self,
        frame_width: int      = 320,
        frame_height: int     = 240,
        obstacle_area_min: int = 400,
        history: int          = 200,
        var_threshold: float  = 50.0,
    ) -> None:
        self.frame_width      = frame_width
        self.frame_height     = frame_height
        self.obstacle_area_min = obstacle_area_min

        # Zone boundaries (vertical thirds of the frame)
        self._zone_left_x  = frame_width // 3
        self._zone_right_x = (frame_width * 2) // 3

        # MOG2 background subtractor
        self._bg_sub = cv2.createBackgroundSubtractorMOG2(
            history=history,
            varThreshold=var_threshold,
            detectShadows=True,
        )

        # Morphological kernels
        self._kernel_open  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        self._kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))

        log.info(
            f"ObstacleDetector ready — "
            f"frame={frame_width}×{frame_height}, "
            f"min_area={obstacle_area_min}px²"
        )

    # ── Public API ────────────────────────────

    def process(self, frame: np.ndarray) -> DetectionResult:
        """
        Run the full detection pipeline on a single BGR frame.

        Returns a DetectionResult with all detected obstacles and
        a recommended navigation action.
        """
        mask = self._build_foreground_mask(frame)
        boxes = self._extract_bounding_boxes(mask)
        return self._classify_and_decide(boxes)

    def draw_annotations(
        self,
        frame: np.ndarray,
        result: DetectionResult,
    ) -> np.ndarray:
        """
        Draw bounding boxes, zone lines, and HUD text onto a copy of the frame.
        Returns the annotated frame.
        """
        h, w = frame.shape[:2]

        # Zone dividers
        cv2.line(frame, (self._zone_left_x, 0),  (self._zone_left_x, h),  (200, 200, 0), 1)
        cv2.line(frame, (self._zone_right_x, 0), (self._zone_right_x, h), (200, 200, 0), 1)

        # Obstacle bounding boxes
        for obs in result.obstacles:
            color = {
                "LEFT":   (0, 165, 255),
                "CENTRE": (0, 0, 255),
                "RIGHT":  (255, 165, 0),
            }.get(obs.zone, (255, 255, 255))

            cv2.rectangle(
                frame,
                (obs.x, obs.y),
                (obs.x + obs.w, obs.y + obs.h),
                color,
                2,
            )
            label = f"{obs.zone} ~{obs.depth_estimate_cm}cm"
            cv2.putText(
                frame, label,
                (obs.x, max(obs.y - 5, 12)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45, color, 1, cv2.LINE_AA,
            )

        # HUD
        hud_lines = [
            f"Obstacles: {result.obstacle_count}",
            f"Action:    {result.recommended_action}",
            f"ClearPath: {result.clear_path}",
        ]
        for i, line in enumerate(hud_lines):
            cv2.putText(
                frame, line,
                (6, 18 + i * 18),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 255, 0), 1, cv2.LINE_AA,
            )

        return frame

    # ── Private helpers ───────────────────────

    def _build_foreground_mask(self, frame: np.ndarray) -> np.ndarray:
        """
        Convert frame to greyscale, blur, apply MOG2 background subtraction,
        then morphologically clean the mask.
        """
        grey  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur  = cv2.GaussianBlur(grey, (5, 5), 0)
        fgmask = self._bg_sub.apply(blur)

        # Remove shadow pixels (value 127 in MOG2 output)
        _, fgmask = cv2.threshold(fgmask, 200, 255, cv2.THRESH_BINARY)

        # Morphological opening — remove small noise
        fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_OPEN,  self._kernel_open)
        # Morphological closing — fill gaps in detected blobs
        fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_CLOSE, self._kernel_close)

        return fgmask

    def _extract_bounding_boxes(self, mask: np.ndarray) -> List[BoundingBox]:
        """
        Find contours in the binary mask and return filtered bounding boxes.
        """
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        boxes: List[BoundingBox] = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.obstacle_area_min:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            zone  = self._classify_zone(x, w)
            depth = self._estimate_depth(area)

            boxes.append(BoundingBox(x=x, y=y, w=w, h=h, area=int(area),
                                     zone=zone, depth_estimate_cm=depth))

        # Sort by area descending (largest = closest)
        boxes.sort(key=lambda b: b.area, reverse=True)
        return boxes

    def _classify_zone(self, x: int, w: int) -> str:
        """Classify bounding box horizontal centre into LEFT / CENTRE / RIGHT."""
        cx = x + w // 2
        if cx < self._zone_left_x:
            return "LEFT"
        elif cx > self._zone_right_x:
            return "RIGHT"
        return "CENTRE"

    def _estimate_depth(self, area_px: float) -> int:
        """
        Heuristic depth estimate using inverse-square law approximation:
            depth_cm ≈ depth_ref × sqrt(area_ref / area_px)
        Calibrate _AREA_REF_PX and _DEPTH_REF_CM for your lens and room.
        """
        if area_px <= 0:
            return 999
        depth = self._DEPTH_REF_CM * np.sqrt(self._AREA_REF_PX / area_px)
        return max(5, int(round(depth)))

    def _classify_and_decide(self, boxes: List[BoundingBox]) -> DetectionResult:
        """
        From the list of obstacles, decide the recommended navigation action.

        Decision rules (priority order):
          1. No obstacles → FWD
          2. CENTRE obstacle close (< 40 cm) → check LEFT/RIGHT clearance
          3. CENTRE obstacle far (≥ 40 cm) → slow FWD
          4. Only LEFT → RIGHT
          5. Only RIGHT → LEFT
        """
        result = DetectionResult()
        result.obstacles      = boxes
        result.obstacle_count = len(boxes)

        if not boxes:
            result.clear_path         = True
            result.recommended_action = "FWD"
            return result

        result.clear_path = False

        # Find closest obstacle in each zone
        zone_closest: dict = {}
        for box in boxes:
            if box.zone not in zone_closest or box.depth_estimate_cm < zone_closest[box.zone]:
                zone_closest[box.zone] = box.depth_estimate_cm

        result.dominant_zone = min(zone_closest, key=zone_closest.get)

        centre_dist = zone_closest.get("CENTRE", 999)
        left_dist   = zone_closest.get("LEFT",   999)
        right_dist  = zone_closest.get("RIGHT",  999)

        # Decision tree
        if centre_dist < 40:
            if left_dist >= right_dist:
                result.recommended_action = "LEFT"
            else:
                result.recommended_action = "RIGHT"
        elif left_dist < 20:
            result.recommended_action = "RIGHT"
        elif right_dist < 20:
            result.recommended_action = "LEFT"
        else:
            result.recommended_action = "FWD"

        return result
