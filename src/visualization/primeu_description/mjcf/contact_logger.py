#!/usr/bin/env python3
import argparse
import collections
import math
from typing import Dict, Optional, Tuple

import mujoco as mj


def obj_name(model: mj.MjModel, obj_type: mj.mjtObj, obj_id: int) -> str:
    name = mj.mj_id2name(model, obj_type, int(obj_id))
    return name if name is not None else f"{obj_type.name.lower()}_{obj_id}"


def body_pair(model: mj.MjModel, geom1: int, geom2: int) -> Tuple[str, str]:
    b1 = int(model.geom_bodyid[geom1])
    b2 = int(model.geom_bodyid[geom2])
    n1 = obj_name(model, mj.mjtObj.mjOBJ_BODY, b1)
    n2 = obj_name(model, mj.mjtObj.mjOBJ_BODY, b2)
    return tuple(sorted((n1, n2)))


def geom_desc(model: mj.MjModel, geom_id: int) -> str:
    gname = obj_name(model, mj.mjtObj.mjOBJ_GEOM, geom_id)
    body_id = int(model.geom_bodyid[geom_id])
    bname = obj_name(model, mj.mjtObj.mjOBJ_BODY, body_id)

    mesh_txt = ""
    if int(model.geom_type[geom_id]) == int(mj.mjtGeom.mjGEOM_MESH):
        mesh_id = int(model.geom_dataid[geom_id])
        if mesh_id >= 0:
            mesh_name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_MESH, mesh_id)
            if mesh_name:
                mesh_txt = f", mesh={mesh_name}"

    return f"geom={gname}:{geom_id}, body={bname}:{body_id}{mesh_txt}"


def main() -> None:
    parser = argparse.ArgumentParser(description="Log MuJoCo contact pairs for debugging")
    parser.add_argument("model", help="Path to scene.xml or robot xml")
    parser.add_argument("--steps", type=int, default=4000, help="Simulation steps")
    parser.add_argument("--report-every", type=int, default=200, help="Print summary every N steps")
    parser.add_argument("--only-penetrating", action="store_true", help="Only log contacts with dist < 0")
    parser.add_argument("--contains", type=str, default="waist", help="Only report body pairs containing this keyword (case-insensitive). Empty to disable")
    parser.add_argument("--drive-waist", action="store_true", help="Apply sinusoidal control to actuators containing 'waist'")
    parser.add_argument("--drive-amp", type=float, default=0.15, help="Amplitude for drive-waist")
    parser.add_argument("--drive-freq", type=float, default=1.2, help="Frequency (Hz) for drive-waist")
    parser.add_argument("--print-contacts", action="store_true", help="Print detailed contacts in each report window")
    parser.add_argument("--max-contact-lines", type=int, default=12, help="Max detailed contact lines per report")
    parser.add_argument(
        "--print-current-contacts",
        action="store_true",
        help="Print current-step contacts at report time (ignores --contains / --only-penetrating)",
    )
    args = parser.parse_args()

    model = mj.MjModel.from_xml_path(args.model)
    data = mj.MjData(model)

    waist_actuators = []
    for i in range(model.nu):
        aname = obj_name(model, mj.mjtObj.mjOBJ_ACTUATOR, i)
        if "waist" in aname.lower():
            waist_actuators.append(i)

    pair_counter = collections.Counter()
    worst_contact: Optional[Tuple[float, str, str, int, int]] = None
    detailed_worst: Dict[Tuple[str, str, int, int], float] = {}

    for step in range(args.steps):
        t = step * model.opt.timestep

        if args.drive_waist and waist_actuators:
            u = args.drive_amp * math.sin(2.0 * math.pi * args.drive_freq * t)
            for aid in waist_actuators:
                data.ctrl[aid] = u

        mj.mj_step(model, data)

        for ci in range(int(data.ncon)):
            c = data.contact[ci]
            dist = float(c.dist)
            if args.only_penetrating and dist >= 0.0:
                continue

            g1, g2 = int(c.geom1), int(c.geom2)
            bpair = body_pair(model, g1, g2)

            if args.contains:
                s = args.contains.lower()
                if s not in bpair[0].lower() and s not in bpair[1].lower():
                    continue

            pair_counter[bpair] += 1

            if worst_contact is None or dist < worst_contact[0]:
                worst_contact = (dist, bpair[0], bpair[1], g1, g2)

            key = (bpair[0], bpair[1], g1, g2)
            prev = detailed_worst.get(key)
            if prev is None or dist < prev:
                detailed_worst[key] = dist

        if (step + 1) % args.report_every == 0:
            print(f"\n[step={step + 1}] ncon={int(data.ncon)}")
            if pair_counter:
                print("Top contact body-pairs:")
                for (b1, b2), cnt in pair_counter.most_common(10):
                    print(f"  {b1:30s} <-> {b2:30s}  count={cnt}")
            else:
                print("No matching contacts in this window.")

            if worst_contact is not None:
                dist, b1, b2, g1, g2 = worst_contact
                g1name = obj_name(model, mj.mjtObj.mjOBJ_GEOM, g1)
                g2name = obj_name(model, mj.mjtObj.mjOBJ_GEOM, g2)
                print(
                    "Worst penetration: "
                    f"dist={dist:.6e}, pair=({b1} <-> {b2}), "
                    f"geom=({g1name}:{g1}, {g2name}:{g2})"
                )

            if args.print_contacts and detailed_worst:
                print("Detailed contacts (worst dist per geom pair):")
                items = sorted(detailed_worst.items(), key=lambda kv: kv[1])
                for i, ((b1, b2, g1, g2), dist) in enumerate(items[: args.max_contact_lines], start=1):
                    gd1 = geom_desc(model, g1)
                    gd2 = geom_desc(model, g2)
                    print(f"  [{i:02d}] dist={dist:.6e} | {b1} <-> {b2}")
                    print(f"       {gd1}")
                    print(f"       {gd2}")

            if args.print_current_contacts:
                print("Current-step contacts (all):")
                current = []
                for ci in range(int(data.ncon)):
                    c = data.contact[ci]
                    g1, g2 = int(c.geom1), int(c.geom2)
                    bpair = body_pair(model, g1, g2)
                    current.append((float(c.dist), bpair[0], bpair[1], g1, g2))

                current.sort(key=lambda x: x[0])
                for i, (dist, b1, b2, g1, g2) in enumerate(current[: args.max_contact_lines], start=1):
                    print(f"  [{i:02d}] dist={dist:.6e} | {b1} <-> {b2}")
                    print(f"       {geom_desc(model, g1)}")
                    print(f"       {geom_desc(model, g2)}")

            pair_counter.clear()
            worst_contact = None
            detailed_worst.clear()


if __name__ == "__main__":
    main()
