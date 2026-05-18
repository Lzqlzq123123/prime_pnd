#!/usr/bin/env python3
from __future__ import annotations

import copy
import math
import pathlib
import xml.etree.ElementTree as ET
from typing import Iterable

THIS_DIR = pathlib.Path(__file__).resolve().parent
REPO_ROOT = THIS_DIR.parent
PRIMEU_MJCF = THIS_DIR / "primeu_robot.xml"
PRIMEU_SCENE = THIS_DIR / "scene.xml"
WUJI_MJCF_DIR = REPO_ROOT / "wuji-hand-description" / "mjcf"
WUJI_URDF_DIR = REPO_ROOT / "wuji-hand-description" / "urdf"
OUTPUT_MJCF = THIS_DIR / "primeu_robot_with_wuji_hands.xml"
OUTPUT_SCENE = THIS_DIR / "scene_with_wuji_hands.xml"

HAND_CONFIG = {
    "left": {
        "site": "left_hand",
        "mount_body": "left_wuji_hand_mount",
        "mesh_prefix": "../wuji-hand-description/meshes/left/",
        "mjcf": WUJI_MJCF_DIR / "left.xml",
        "urdf": WUJI_URDF_DIR / "left-ros.urdf",
    },
    "right": {
        "site": "right_hand",
        "mount_body": "right_wuji_hand_mount",
        "mesh_prefix": "../wuji-hand-description/meshes/right/",
        "mjcf": WUJI_MJCF_DIR / "right.xml",
        "urdf": WUJI_URDF_DIR / "right-ros.urdf",
    },
}


def indent(element: ET.Element, level: int = 0) -> None:
    indent_text = "\n" + level * "  "
    if len(element):
        if not element.text or not element.text.strip():
            element.text = indent_text + "  "
        for child in element:
            indent(child, level + 1)
        if not element[-1].tail or not element[-1].tail.strip():
            element[-1].tail = indent_text
    if level and (not element.tail or not element.tail.strip()):
        element.tail = indent_text


def find_body_with_site(root: ET.Element, site_name: str) -> tuple[ET.Element, ET.Element]:
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError("MJCF is missing <worldbody>")

    def walk(body: ET.Element) -> tuple[ET.Element, ET.Element] | None:
        for site in body.findall("site"):
            if site.get("name") == site_name:
                return body, site
        for child in body.findall("body"):
            result = walk(child)
            if result is not None:
                return result
        return None

    for top_level_body in worldbody.findall("body"):
        result = walk(top_level_body)
        if result is not None:
            return result
    raise ValueError(f"Could not find site {site_name!r}")


def ensure_default_class(root: ET.Element) -> None:
    default_root = root.find("default")
    if default_root is None:
        default_root = ET.SubElement(root, "default")
    for child in default_root.findall("default"):
        if child.get("class") == "wuji_hand":
            return
    hand_default = ET.SubElement(default_root, "default", {"class": "wuji_hand"})
    ET.SubElement(hand_default, "joint", {"armature": "0.005", "damping": "0.05", "frictionloss": "0.01"})


def rewrite_mesh_paths(asset_elements: Iterable[ET.Element], mesh_prefix: str) -> list[ET.Element]:
    rewritten = []
    for element in asset_elements:
        cloned = copy.deepcopy(element)
        if cloned.tag == "mesh":
            file_name = cloned.get("file")
            if file_name:
                cloned.set("file", f"{mesh_prefix}{file_name}")
        rewritten.append(cloned)
    return rewritten


def format_vec(values: Iterable[float]) -> str:
    return " ".join(f"{value:.9g}" for value in values)


def build_palm_inertial(urdf_path: pathlib.Path) -> ET.Element:
    urdf_root = ET.parse(urdf_path).getroot()
    palm_link = urdf_root.find("link")
    if palm_link is None:
        raise ValueError(f"URDF {urdf_path} does not contain a root link")
    inertial = palm_link.find("inertial")
    if inertial is None:
        raise ValueError(f"URDF {urdf_path} root link is missing inertial data")

    origin = inertial.find("origin")
    mass = inertial.find("mass")
    inertia = inertial.find("inertia")
    if origin is None or mass is None or inertia is None:
        raise ValueError(f"URDF {urdf_path} root inertial block is incomplete")

    xyz = origin.get("xyz", "0 0 0")
    attributes = {
        "pos": xyz,
        "mass": mass.get("value", "0"),
        "fullinertia": " ".join(
            [
                inertia.get("ixx", "0"),
                inertia.get("iyy", "0"),
                inertia.get("izz", "0"),
                inertia.get("ixy", "0"),
                inertia.get("ixz", "0"),
                inertia.get("iyz", "0"),
            ]
        ),
    }
    return ET.Element("inertial", attributes)


def append_hand(root: ET.Element, side: str) -> None:
    config = HAND_CONFIG[side]
    parent_body, site = find_body_with_site(root, config["site"])
    hand_root = ET.parse(config["mjcf"]).getroot()
    hand_worldbody = hand_root.find("worldbody")
    if hand_worldbody is None:
        raise ValueError(f"{config['mjcf']} is missing <worldbody>")

    mount_body = ET.Element(
        "body",
        {
            "name": config["mount_body"],
            "pos": site.get("pos", "0 0 0"),
            "quat": site.get("quat", "1 0 0 0"),
            "childclass": "wuji_hand",
        },
    )
    mount_body.append(build_palm_inertial(config["urdf"]))

    for child in hand_worldbody:
        mount_body.append(copy.deepcopy(child))

    parent_body.append(mount_body)

    asset_root = root.find("asset")
    if asset_root is None:
        asset_root = ET.SubElement(root, "asset")
    hand_asset = hand_root.find("asset")
    if hand_asset is not None:
        for asset in rewrite_mesh_paths(hand_asset, config["mesh_prefix"]):
            asset_root.append(asset)

    actuator_root = root.find("actuator")
    if actuator_root is None:
        actuator_root = ET.SubElement(root, "actuator")
    hand_actuator = hand_root.find("actuator")
    if hand_actuator is not None:
        for actuator in hand_actuator:
            actuator_root.append(copy.deepcopy(actuator))


def generate_scene(output_robot_file: str) -> ET.ElementTree:
    scene_root = ET.parse(PRIMEU_SCENE).getroot()
    include = scene_root.find("include")
    if include is None:
        include = ET.SubElement(scene_root, "include")
    include.set("file", output_robot_file)
    indent(scene_root)
    return ET.ElementTree(scene_root)


def main() -> None:
    root = ET.parse(PRIMEU_MJCF).getroot()
    ensure_default_class(root)
    append_hand(root, "left")
    append_hand(root, "right")
    indent(root)
    ET.ElementTree(root).write(OUTPUT_MJCF, encoding="utf-8", xml_declaration=True)
    generate_scene(OUTPUT_MJCF.name).write(OUTPUT_SCENE, encoding="utf-8", xml_declaration=True)
    print(f"Generated {OUTPUT_MJCF}")
    print(f"Generated {OUTPUT_SCENE}")


if __name__ == "__main__":
    main()
