"""
preprocessing.py

Reimplementación en Python del paso de IPM de lane_common::Preprocessor
(C++), usando los MISMOS parámetros (ipm_src_flat / ipm_dst_flat /
resize_factor) que se cargan desde el YAML compartido con los otros tres
nodos de detección.

NOTA IMPORTANTE (léela antes de confiar en los números del CNN):
Este módulo reimplementa el paso de IPM, no hace binding directo a la clase
C++ lane_common::Preprocessor. Con los mismos ipm_src_flat/ipm_dst_flat
calibrados el resultado geométrico debería ser idéntico, pero no aplica el
Canny/white_filter de los otros nodos (el U-Net no lo necesita: reemplaza esa
etapa de segmentación clásica). Antes de usar los números para el paper,
compara visualmente ~/debug_image de este nodo contra el de lane_detection_fcm
con el mismo frame para confirmar que la imagen IPM coincide pixel a pixel.
"""

from dataclasses import dataclass, field
from typing import List

import cv2
import numpy as np


@dataclass
class RoiParams:
    x: int
    y: int
    width: int
    height: int


@dataclass
class PreprocessingParams:
    resize_factor: float = 0.5
    ipm_src_flat: List[float] = field(
        default_factory=lambda: [56.0, 110.0, 250.0, 110.0, 0.0, 200.0, 320.0, 200.0]
    )
    ipm_dst_flat: List[float] = field(
        default_factory=lambda: [56.0, 0.0, 250.0, 0.0, 56.0, 240.0, 250.0, 240.0]
    )
    ipm_out_width: int = 320
    ipm_out_height: int = 240
    roi_left: RoiParams = field(default_factory=lambda: RoiParams(0, 0, 160, 240))
    roi_right: RoiParams = field(default_factory=lambda: RoiParams(160, 0, 160, 240))


class Preprocessor:
    """Aplica resize + IPM, replicando la geometría de lane_common::Preprocessor."""

    def __init__(self, params: PreprocessingParams):
        self.params = params
        src = np.array(params.ipm_src_flat, dtype=np.float32).reshape(4, 2)
        dst = np.array(params.ipm_dst_flat, dtype=np.float32).reshape(4, 2)
        self._M = cv2.getPerspectiveTransform(src, dst)

    def process(self, bgr_image: np.ndarray) -> np.ndarray:
        """
        Args:
            bgr_image: imagen de entrada en BGR (tal como llega del topic de cámara).

        Returns:
            ipm_gray: imagen IPM en escala de grises, tamaño
                      (ipm_out_height, ipm_out_width).
        """
        p = self.params
        if p.resize_factor != 1.0:
            bgr_image = cv2.resize(
                bgr_image, None, fx=p.resize_factor, fy=p.resize_factor,
                interpolation=cv2.INTER_LINEAR,
            )

        ipm_bgr = cv2.warpPerspective(
            bgr_image, self._M, (p.ipm_out_width, p.ipm_out_height),
            flags=cv2.INTER_LINEAR,
        )
        ipm_gray = cv2.cvtColor(ipm_bgr, cv2.COLOR_BGR2GRAY)
        return ipm_gray, ipm_bgr

    def crop_roi(self, mask: np.ndarray, roi: RoiParams) -> np.ndarray:
        """Recorta 'mask' (o cualquier imagen 2D) a la ROI dada."""
        return mask[roi.y:roi.y + roi.height, roi.x:roi.x + roi.width]
