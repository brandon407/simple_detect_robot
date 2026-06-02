"""
Visualization Utilities — draw detection results on images.

Handles bounding boxes, labels, confidence bars, zone overlays.
"""
import cv2
import numpy as np

# Industrial inspection color palette (BGR format)
COLORS = {
    'defect': {
        'scratch': (0, 140, 255),       # Orange
        'crack': (0, 0, 255),           # Red
        'deformation': (0, 255, 255),   # Yellow
        'stain': (255, 140, 0),         # Light Blue
        'missing_part': (0, 0, 200),    # Dark Red
        'default': (0, 165, 255),       # Orange default
    },
    'meter': {
        'ok': (0, 255, 0),              # Green
        'warning': (0, 255, 255),       # Yellow
        'error': (0, 0, 255),           # Red
        'default': (255, 255, 255),     # White
    },
    'safety': {
        'helmet_ok': (0, 255, 0),       # Green
        'helmet_missing': (0, 0, 255),  # Red
        'vest_ok': (0, 255, 0),         # Green
        'vest_missing': (0, 0, 255),    # Red
        'zone_intrusion': (0, 140, 255),# Orange
        'fire': (0, 0, 255),            # Red
        'smoke': (128, 128, 128),       # Gray
        'default': (255, 255, 0),       # Cyan
    },
    'severity': {
        'minor': (0, 255, 255),         # Yellow
        'major': (0, 165, 255),         # Orange
        'critical': (0, 0, 255),        # Red
        'ok': (0, 255, 0),              # Green
    },
}


def draw_detections(image: np.ndarray,
                    detections: list[dict],
                    detector_type: str = 'defect') -> np.ndarray:
    """Draw detection results on an image.

    Args:
        image: BGR image (OpenCV format), modified in-place.
        detections: List of detection dicts, each with:
            - bbox: [x1, y1, x2, y2] (optional)
            - label: class name string
            - confidence: float 0-1 (optional)
            - severity: 'minor'|'major'|'critical' (optional)
        detector_type: 'defect' | 'meter' | 'safety'.

    Returns:
        Annotated image.
    """
    annotated = image.copy()
    palette = COLORS.get(detector_type, COLORS['defect'])

    for det in detections:
        label = det.get('label', 'unknown')
        confidence = det.get('confidence', 1.0)
        severity = det.get('severity', None)
        bbox = det.get('bbox', None)

        # Pick color
        color = palette.get(label, palette['default'])
        if severity:
            severity_color = COLORS['severity'].get(severity, (255, 255, 255))
        else:
            severity_color = color

        # Draw bounding box
        if bbox:
            x1, y1, x2, y2 = [int(v) for v in bbox]
            cv2.rectangle(annotated, (x1, y1), (x2, y2), severity_color, 2)

            # Label background
            text = f"{label} {confidence:.0%}" if confidence < 1.0 else label
            if severity:
                text += f" [{severity}]"

            (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
            cv2.rectangle(annotated,
                          (x1, y1 - th - 8), (x1 + tw + 4, y1),
                          severity_color, -1)
            cv2.putText(annotated, text, (x1 + 2, y1 - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        else:
            # No bbox — just put text at top
            text = f"{label}: {confidence:.0%}" if confidence < 1.0 else label
            y_offset = 30 + detections.index(det) * 30
            cv2.putText(annotated, text, (10, y_offset),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, severity_color, 2)

    return annotated


def draw_zones(image: np.ndarray,
               zones: list[np.ndarray],
               labels: list[str] | None = None) -> np.ndarray:
    """Draw safety/restricted zones as polygons.

    Args:
        image: BGR image.
        zones: List of polygon point arrays, each shape (N, 2).
        labels: Optional labels per zone.

    Returns:
        Annotated image.
    """
    annotated = image.copy()
    for i, zone in enumerate(zones):
        pts = zone.reshape(-1, 1, 2).astype(np.int32)
        cv2.polylines(annotated, [pts], True, (0, 255, 255), 2)

        if labels and i < len(labels):
            cx, cy = int(np.mean(pts[:, 0, 0])), int(np.mean(pts[:, 0, 1]))
            cv2.putText(annotated, labels[i], (cx - 20, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    return annotated


def draw_info_overlay(image: np.ndarray,
                      info: dict,
                      position: tuple[int, int] = (10, 10)) -> np.ndarray:
    """Draw an info panel overlay on the image.

    Args:
        image: BGR image.
        info: Dict of {label: value} to display.
        position: (x, y) top-left of the info panel.

    Returns:
        Annotated image.
    """
    annotated = image.copy()
    x, y = position
    line_height = 20

    # Panel background
    panel_h = len(info) * line_height + 10
    panel_w = 280
    overlay = annotated.copy()
    cv2.rectangle(overlay, (x, y), (x + panel_w, y + panel_h),
                  (30, 30, 30), -1)
    cv2.addWeighted(overlay, 0.6, annotated, 0.4, 0, annotated)

    for i, (key, value) in enumerate(info.items()):
        text = f"{key}: {value}"
        cv2.putText(annotated, text, (x + 5, y + (i + 1) * line_height),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

    return annotated
