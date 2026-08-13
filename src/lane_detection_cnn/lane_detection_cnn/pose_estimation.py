"""
pose_estimation.py

Replica en Python de LaneDetectionFcmNode::estimatePoseAndAngle() (ver
LaneDetectionFcmNode.cpp, lineas 337-444). Se usa la MISMA lógica de ajuste
de línea (equivalente a cv::fitLine con DIST_L2), el mismo offset de 73 px
para el caso de un solo lado detectado, y la misma fórmula de ángulo
(Ec. 10-11 del paper), para que la comparación CNN vs FCM/SW/Hough sea
sobre el mismo pipeline downstream, no solo sobre la misma entrada.

Diferencia respecto a FCM: en vez de centroides de clusters difusos, aquí
los "centroides" son el x promedio de los píxeles de la máscara U-Net dentro
de bandas horizontales (bins), replicando el conteo de 4 semillas por lado
que usa FCM por defecto (fcm.seed_left_y / seed_right_y = 48, 96, 144, 192).
"""

from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np

LANE_HALF_WIDTH_PX = 73.0  # mismo valor que LaneDetectionFcmNode.cpp linea 405/414


@dataclass
class PoseResult:
    angle_deg: float
    center_deviation_px: float
    status: int  # 0=ninguno, 1=solo L, 2=solo R, 3=L+R
    centroids_left: List[Tuple[int, int]]
    centroids_right: List[Tuple[int, int]]


def extract_band_centroids(
    roi_mask: np.ndarray, roi_x_offset: int, n_bands: int = 4
) -> List[Tuple[int, int]]:
    """
    Divide roi_mask (binaria, 0/1 o 0/255) en n_bands franjas horizontales y
    calcula el centroide (x, y) de los píxeles activos en cada franja.
    Bandas sin píxeles activos se omiten (igual que FCM no "inventa" un
    cluster donde no hay puntos de entrada).

    Las coordenadas x devueltas están en el sistema de la imagen completa
    (se les suma roi_x_offset).
    """
    h = roi_mask.shape[0]
    edges = np.linspace(0, h, n_bands + 1).astype(int)
    centroids = []
    for i in range(n_bands):
        y0, y1 = edges[i], edges[i + 1]
        band = roi_mask[y0:y1, :]
        ys, xs = np.nonzero(band)
        if xs.size == 0:
            continue
        x_mean = float(xs.mean()) + roi_x_offset
        y_mean = float(ys.mean()) + y0
        centroids.append((int(round(x_mean)), int(round(y_mean))))
    centroids.sort(key=lambda p: p[1])
    return centroids


def _fit_line(points: List[Tuple[int, int]]) -> Optional[np.ndarray]:
    """Equivalente a cv::fitLine(points, DIST_L2, 0, 0.01, 0.01) -> [vx, vy, x0, y0]."""
    if len(points) < 2:
        return None
    pts = np.array(points, dtype=np.float32)
    line = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01)
    return line.flatten()  # [vx, vy, x0, y0]


def _x_at_y(line: np.ndarray, y: float) -> float:
    vx, vy, x0, y0 = line
    if abs(vy) < 1e-6:
        return float(x0)
    return float(x0 + (vx / vy) * (y - y0))


def estimate_pose_and_angle(
    centroids_left: List[Tuple[int, int]],
    centroids_right: List[Tuple[int, int]],
    image_cols: int,
    image_rows: int,
    camera_center_offset: int,
) -> PoseResult:
    """Puerto directo de LaneDetectionFcmNode::estimatePoseAndAngle()."""
    center_cam = (image_cols // 2) - camera_center_offset
    y_mid = image_rows // 2
    y_bot = image_rows

    line_l = _fit_line(centroids_left)
    line_r = _fit_line(centroids_right)
    found_left = line_l is not None
    found_right = line_r is not None

    center_lane = center_cam
    status = 0

    if found_left and found_right:
        status = 3
        xl_mid = _x_at_y(line_l, y_mid)
        xr_mid = _x_at_y(line_r, y_mid)
        center_lane = int((xl_mid + xr_mid) / 2.0)
    elif found_left:
        status = 1
        xl_mid = _x_at_y(line_l, y_mid)
        center_lane = int(xl_mid + LANE_HALF_WIDTH_PX)
    elif found_right:
        status = 2
        xr_mid = _x_at_y(line_r, y_mid)
        center_lane = int(xr_mid - LANE_HALF_WIDTH_PX)
    else:
        status = 0

    center_deviation_px = float(center_cam - center_lane)

    dy = float(y_bot - y_mid)
    dx = float(center_cam - center_lane)
    if abs(dy) < 1e-6:
        angle_deg = 90.0
    else:
        m = dx / dy
        angle_deg = 90.0 - np.degrees(np.arctan(m))

    return PoseResult(
        angle_deg=float(angle_deg),
        center_deviation_px=center_deviation_px,
        status=status,
        centroids_left=centroids_left,
        centroids_right=centroids_right,
    )
