#!/usr/bin/env python3
from __future__ import annotations

import argparse
import copy
import pathlib
import xml.etree.ElementTree as ET
from typing import cast

THIS_DIR = pathlib.Path(__file__).resolve().parent
REPO_ROOT = THIS_DIR.parent
PRIMEU_URDF = THIS_DIR / "primeu_robot.urdf"
WUJI_URDF_DIR = REPO_ROOT / "wuji-hand-description" / "urdf"
DEFAULT_OUTPUT = THIS_DIR / "primeu_robot_with_wuji_hands.urdf"

HAND_CONFIG = {
    "left": {
        "source": WUJI_URDF_DIR / "left-ros.urdf",
        "parent": "left_hand",
        "child": "left_palm_link",
        "joint": "left_wuji_hand_mount",
    },
    "right": {
        "source": WUJI_URDF_DIR / "right-ros.urdf",
        "parent": "right_hand",
        "child": "right_palm_link",
        "joint": "right_wuji_hand_mount",
    },
}


def parse_triplet(value: str) -> tuple[float, float, float]:
    parts = [part.strip() for part in value.replace(",", " ").split() if part.strip()]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError(f"Expected 3 values, got: {value!r}")
    try:
        values = tuple(float(part) for part in parts)
        return cast(tuple[float, float, float], values)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(str(exc)) from exc


def format_triplet(values: tuple[float, float, float]) -> str:
    return " ".join(f"{value:.9g}" for value in values)


def rewrite_mesh_paths(element: ET.Element) -> None:
    for mesh in element.findall(".//mesh"):
        filename = mesh.attrib.get("filename")
        if not filename:
            continue
        filename = filename.replace("package://wuji_hand_description/", "package://primeu_description/wuji-hand-description/")
        filename = filename.replace("../meshes/", "package://primeu_description/wuji-hand-description/meshes/")
        mesh.set("filename", filename)


def load_robot(path: pathlib.Path) -> ET.Element:
    return ET.parse(path).getroot()


def append_hand(robot: ET.Element, side: str, xyz: tuple[float, float, float], rpy: tuple[float, float, float]) -> None:
    config = HAND_CONFIG[side]
    hand_root = load_robot(config["source"])

    for element in hand_root:
        if element.tag not in {"link", "joint"}:
            continue
        cloned = copy.deepcopy(element)
        rewrite_mesh_paths(cloned)
        robot.append(cloned)

    mount_joint = ET.Element("joint", {"name": config["joint"], "type": "fixed"})
    ET.SubElement(mount_joint, "origin", {"xyz": format_triplet(xyz), "rpy": format_triplet(rpy)})
    ET.SubElement(mount_joint, "parent", {"link": config["parent"]})
    ET.SubElement(mount_joint, "child", {"link": config["child"]})
    ET.SubElement(mount_joint, "axis", {"xyz": "0 0 0"})
    robot.append(mount_joint)


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


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Merge Wuji hands into primeu robot URDF.")
    parser.add_argument("--base-urdf", type=pathlib.Path, default=PRIMEU_URDF)
    parser.add_argument("--output", type=pathlib.Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--sides", nargs="+", choices=["left", "right"], default=["left", "right"])
    parser.add_argument("--left-xyz", type=parse_triplet, default=(0.0, 0.0, 0.0))
    parser.add_argument("--left-rpy", type=parse_triplet, default=(0.0, 0.0, 0.0))
    parser.add_argument("--right-xyz", type=parse_triplet, default=(0.0, 0.0, 0.0))
    parser.add_argument("--right-rpy", type=parse_triplet, default=(0.0, 0.0, 0.0))
    return parser


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()

    robot = load_robot(args.base_urdf)
    selected_sides = list(dict.fromkeys(args.sides))

    for side in selected_sides:
        xyz = getattr(args, f"{side}_xyz")
        rpy = getattr(args, f"{side}_rpy")
        append_hand(robot, side, xyz, rpy)

    indent(robot)
    tree = ET.ElementTree(robot)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(args.output, encoding="utf-8", xml_declaration=True)

    print(f"Generated merged URDF: {args.output}")
    for side in selected_sides:
        xyz = getattr(args, f"{side}_xyz")
        rpy = getattr(args, f"{side}_rpy")
        print(
            f"  {side}: parent={HAND_CONFIG[side]['parent']} child={HAND_CONFIG[side]['child']} xyz={format_triplet(xyz)} rpy={format_triplet(rpy)}"
        )


if __name__ == "__main__":
    main()
