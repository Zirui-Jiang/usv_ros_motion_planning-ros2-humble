#!/usr/bin/env python3
"""Create a runtime-only copy of an SDF world with the 0211 ENU origin."""

import argparse
from pathlib import Path
import re
import xml.etree.ElementTree as ET


LATITUDE = "30.5453"
LONGITUDE = "114.392931"


def replace_single(text, tag, value):
    pattern = rf"(<{tag}>\s*)[^<]+?(\s*</{tag}>)"
    updated, count = re.subn(pattern, rf"\g<1>{value}\g<2>", text)
    if count != 1:
        raise ValueError(f"world must contain exactly one <{tag}> element; found {count}")
    return updated


def main():
    parser = argparse.ArgumentParser(
        description="Copy an existing Gazebo world and replace only its WGS84 ENU origin")
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()

    source = args.source.resolve(strict=True)
    if source == args.output.resolve():
        raise ValueError("output must be an independent world copy")
    raw = source.read_bytes()
    text = raw.decode("utf-8")
    root = ET.fromstring(text)
    worlds = [root] if root.tag == "world" else root.findall("world")
    if len(worlds) != 1:
        raise ValueError("source must contain exactly one SDF world")
    coordinates = worlds[0].find("spherical_coordinates")
    if coordinates is None:
        raise ValueError("source world has no spherical_coordinates")
    if coordinates.findtext("surface_model", "").strip() != "EARTH_WGS84":
        raise ValueError("source world must use EARTH_WGS84")
    if coordinates.findtext("world_frame_orientation", "").strip() != "ENU":
        raise ValueError("source world must use ENU")

    for tag, value in (("latitude_deg", LATITUDE), ("longitude_deg", LONGITUDE),
                       ("elevation", "0"), ("heading_deg", "0")):
        text = replace_single(text, tag, value)
    # Parse the result before writing so a malformed copy can never reach Gazebo.
    ET.fromstring(text)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_bytes(text.encode("utf-8"))


if __name__ == "__main__":
    main()
