# SPDX-License-Identifier: Apache-2.0
"""Reproduce the committed raster from axis-aligned, static SDF collisions.

Dynamic bodies are deliberately absent; live /scan costmap layers track them.
Run from any directory with: python3 path/to/generate_factory_map.py
"""

from pathlib import Path
import xml.etree.ElementTree as ET

RESOLUTION = 0.05
ORIGIN = (-6.2, -4.2)
WIDTH, HEIGHT = 248, 168


def pose(element):
    values = tuple(map(float, element.findtext("pose", "0 0 0 0 0 0").split()))
    if values[3:] != (0.0, 0.0, 0.0):
        raise ValueError("factory raster requires axis-aligned geometry")
    return values[:3]


def generate():
    simulation = Path(__file__).resolve().parents[2] / "factory_simulation"
    world = ET.parse(simulation / "worlds/factory_floor.world").getroot().find("world")
    models = [(model, pose(model)) for model in world.findall("model")]
    for include in world.findall("include"):
        name = include.findtext("uri").removeprefix("model://")
        model = (
            ET.parse(simulation / "models" / name / "model.sdf").getroot().find("model")
        )
        models.append((model, pose(include)))
    rectangles = []
    for model, offset in models:
        if model.findtext("static") != "true" or model.attrib["name"] == "floor":
            continue
        for link in model.findall("link"):
            link_pose = pose(link)
            for collision in link.findall("collision"):
                size = collision.findtext("geometry/box/size")
                if size is None:
                    raise ValueError("unsupported static collision geometry")
                center = tuple(
                    a + b + c for a, b, c in zip(offset, link_pose, pose(collision))
                )
                sx, sy, sz = map(float, size.split())
                if center[2] - sz / 2 > 0.18:
                    continue
                rectangles.append(
                    (
                        center[0] - sx / 2,
                        center[0] + sx / 2,
                        center[1] - sy / 2,
                        center[1] + sy / 2,
                    )
                )
    pixels = bytearray()
    for row in reversed(range(HEIGHT)):
        y = ORIGIN[1] + (row + 0.5) * RESOLUTION
        for column in range(WIDTH):
            x = ORIGIN[0] + (column + 0.5) * RESOLUTION
            occupied = any(
                left <= x <= right and bottom <= y <= top
                for left, right, bottom, top in rectangles
            )
            # The area outside the closed factory wall is unknown.
            pixels.append(
                0 if occupied else 254 if -5.95 < x < 5.95 and -3.95 < y < 3.95 else 205
            )
    target = Path(__file__).with_name("factory_map.pgm")
    target.write_bytes(f"P5\n{WIDTH} {HEIGHT}\n255\n".encode() + pixels)
    print(
        f"Generated {target}: {WIDTH}x{HEIGHT}, {len(rectangles)} static collision rectangles"
    )


if __name__ == "__main__":
    generate()
