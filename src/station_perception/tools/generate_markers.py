#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Generate reproducible DICT_4X4_50 textures with a white quiet zone."""

import argparse
from pathlib import Path

import cv2
import numpy as np


def generate_markers(output_directory: Path):
    output_directory.mkdir(parents=True, exist_ok=True)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    generate_marker = (
        cv2.aruco.generateImageMarker
        if hasattr(cv2.aruco, "generateImageMarker")
        else cv2.aruco.drawMarker
    )
    for station, marker_id in [("assembly", 10), ("inspection", 20)]:
        image = np.full((300, 300), 255, dtype=np.uint8)
        image[30:270, 30:270] = generate_marker(dictionary, marker_id, 240)
        path = output_directory / f"{station}_{marker_id}.png"
        if not cv2.imwrite(str(path), image):
            raise RuntimeError(f"Could not write marker texture {path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-directory",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "markers",
    )
    generate_markers(parser.parse_args().output_directory)
