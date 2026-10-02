"""Detect known station identities and rank their visible image area."""

from pathlib import Path
import sys

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from station_perception.detector import ArucoStationDetector

generate_marker = (
    cv2.aruco.generateImageMarker
    if hasattr(cv2.aruco, "generateImageMarker")
    else cv2.aruco.drawMarker
)


def marker_image(marker_id, size=180, dictionary=cv2.aruco.DICT_4X4_50):
    image = np.full((480, 640), 255, dtype=np.uint8)
    marker = generate_marker(
        cv2.aruco.getPredefinedDictionary(dictionary), marker_id, size
    )
    image[100 : 100 + size, 100 : 100 + size] = marker
    return image


@pytest.mark.parametrize("marker_id,station", [(10, "assembly"), (20, "inspection")])
def test_white_quiet_zone_on_colored_station_board(marker_id, station):
    # Gazebo's white texture margin is itself a contour on the dark blue board.
    # OpenCV 4.5.4's default candidate-distance grouping suppresses the actual
    # black-border candidate in favor of that quiet-zone contour.
    image = np.full((480, 640, 3), 127, dtype=np.uint8)
    image[157:433, 98:539] = (85, 40, 20)
    texture = np.full((120, 120), 255, dtype=np.uint8)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    texture[12:108, 12:108] = generate_marker(dictionary, marker_id, 96)
    image[179:299, 258:378] = texture[:, :, None]
    detections = ArucoStationDetector().detect(image)
    assert [(item.marker_id, item.station_id) for item in detections] == [
        (marker_id, station)
    ]


@pytest.mark.parametrize("marker_id,station_id", [(10, "assembly"), (20, "inspection")])
def test_known_station_and_normalized_visible_area(marker_id, station_id):
    (detection,) = ArucoStationDetector().detect(marker_image(marker_id))
    assert (detection.marker_id, detection.station_id) == (marker_id, station_id)
    assert detection.confidence == pytest.approx(179 * 179 / (640 * 480), abs=0.001)


def test_no_marker_has_no_detection():
    assert ArucoStationDetector().detect(np.full((480, 640, 3), 255, np.uint8)) == []


def test_wrong_dictionary_has_no_detection():
    assert (
        ArucoStationDetector().detect(
            marker_image(10, dictionary=cv2.aruco.DICT_5X5_50)
        )
        == []
    )


def test_occluded_marker_does_not_invent_identity():
    image = marker_image(10)
    image[100:190, 100:190] = 255
    assert ArucoStationDetector().detect(image) == []


def test_largest_valid_marker_wins_over_smaller_station_and_unknown_marker():
    image = np.full((480, 640, 3), 255, np.uint8)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    for marker_id, x, size in [(10, 20, 120), (20, 160, 160), (30, 350, 240)]:
        marker = generate_marker(dictionary, marker_id, size)
        image[100 : 100 + size, x : x + size] = marker[:, :, None]
    detections = ArucoStationDetector().detect(image)
    assert [d.marker_id for d in detections] == [20, 10]
    assert detections[0].confidence > detections[1].confidence


def test_configured_identity_mapping_filters_other_markers():
    detector = ArucoStationDetector({20: "quality"})
    assert detector.detect(marker_image(10)) == []
    assert detector.detect(marker_image(20))[0].station_id == "quality"
