# SPDX-License-Identifier: Apache-2.0
"""Pure ArUco identity detection; confidence is visible image area, not probability."""

from dataclasses import dataclass
from typing import Mapping

import cv2
import numpy as np


@dataclass(frozen=True)
class MarkerDetection:
    station_id: str
    marker_id: int
    confidence: float


class ArucoStationDetector:
    def __init__(self, marker_stations: Mapping[int, str] | None = None):
        self.marker_stations = dict({10: "assembly", 20: "inspection"}
                                    if marker_stations is None else marker_stations)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self._detector = cv2.aruco.ArucoDetector(dictionary)

    def detect(self, image: np.ndarray) -> list[MarkerDetection]:
        """Return configured markers ordered by decreasing normalized image area.

        Accept grayscale or BGR/RGB uint8 images. Color ordering does not change
        the black-and-white marker identity. The first result is the winner.
        """
        corners, ids, _ = self._detector.detectMarkers(image)
        if ids is None:
            return []
        image_area = image.shape[0] * image.shape[1]
        detections = []
        for marker_corners, marker_id in zip(corners, ids.reshape(-1)):
            marker_id = int(marker_id)
            if marker_id in self.marker_stations:
                area = abs(cv2.contourArea(marker_corners.reshape(4, 2)))
                confidence = float(np.clip(area / image_area, 0.0, 1.0))
                detections.append(MarkerDetection(self.marker_stations[marker_id], marker_id, confidence))
        return sorted(detections, key=lambda detection: (-detection.confidence, detection.marker_id))
