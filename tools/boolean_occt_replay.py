"""Compare identical canonical V1 inputs with OpenRCAD and external OCCT.

OCCT is an independent verification dependency, never an application dependency.
"""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import subprocess

import OCP
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common, BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepGProp import BRepGProp
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCone, BRepPrimAPI_MakeCylinder, BRepPrimAPI_MakeSphere
from OCP.GProp import GProp_GProps
from OCP.TopAbs import TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.gp import gp_Ax1, gp_Ax2, gp_Dir, gp_Pnt, gp_Trsf, gp_Vec


def identity(case):
    data = bytearray(b"ZBCV1\0\0\0")
    data += bytes([["union", "cut", "intersect"].index(case["operation"]),
                   ["disjoint", "touching", "overlapping", "contained", "coincident"].index(case["contact_class"]),
                   case["logarithmic_scale"] & 255])
    for key in ["object", "tool"]:
        operand = case[key]
        primitive = operand["primitive"]
        kind = primitive["kind"]
        data.append(["box", "cylinder", "cone", "sphere"].index(kind))
        values = {"box": lambda: primitive["size"],
                  "cylinder": lambda: [primitive["radius"], primitive["height"]],
                  "cone": lambda: [primitive["radius1"], primitive["radius2"], primitive["height"]],
                  "sphere": lambda: [primitive["radius"]]}[kind]()
        transform = operand.get("transform", {})
        values = [*values, *transform.get("translation", [0, 0, 0]),
                  *transform.get("rotation_axis", [0, 0, 1]), transform.get("rotation_radians", 0)]
        for value in values:
            data += struct.pack("<d", 0.0 if value == 0 else value)
    return hashlib.sha256(data).hexdigest()


def operand_shape(operand, scale):
    p = operand["primitive"]
    axis = gp_Ax2(gp_Pnt(), gp_Dir(0, 0, 1))
    shape = {
        "box": lambda: BRepPrimAPI_MakeBox(*[v * scale for v in p["size"]]).Shape(),
        "cylinder": lambda: BRepPrimAPI_MakeCylinder(axis, p["radius"] * scale, p["height"] * scale).Shape(),
        "cone": lambda: BRepPrimAPI_MakeCone(axis, p["radius1"] * scale, p["radius2"] * scale, p["height"] * scale).Shape(),
        "sphere": lambda: BRepPrimAPI_MakeSphere(p["radius"] * scale).Shape(),
    }[p["kind"]]()
    t = operand.get("transform", {})
    rotation = gp_Trsf()
    rotation.SetRotation(gp_Ax1(gp_Pnt(), gp_Dir(*t.get("rotation_axis", [0, 0, 1]))), t.get("rotation_radians", 0))
    shape = BRepBuilderAPI_Transform(shape, rotation, True).Shape()
    translation = gp_Trsf()
    translation.SetTranslation(gp_Vec(*[v * scale for v in t.get("translation", [0, 0, 0])]))
    return BRepBuilderAPI_Transform(shape, translation, True).Shape()


def observe(case):
    scale = 10.0 ** case["logarithmic_scale"]
    operation = {"union": BRepAlgoAPI_Fuse, "cut": BRepAlgoAPI_Cut, "intersect": BRepAlgoAPI_Common}[case["operation"]](
        operand_shape(case["object"], scale), operand_shape(case["tool"], scale))
    if not operation.IsDone():
        raise RuntimeError("OCCT operation failed")
    shape = operation.Shape()
    mass, area = GProp_GProps(), GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, mass)
    BRepGProp.SurfaceProperties_s(shape, area)
    explorer = TopExp_Explorer(shape, TopAbs_SOLID)
    count = 0
    while explorer.More():
        count += 1
        explorer.Next()
    center = mass.CentreOfMass()
    return {"body_count": count, "volume": mass.Mass(), "surface_area": area.Mass(),
            "centroid": [center.X(), center.Y(), center.Z()]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rust-executable", type=Path, required=True)
    parser.add_argument("--cases", type=Path, default=Path("zerocad-core/tests/boolean_cases"))
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    observations = []
    failed = False
    for path in sorted(args.cases.glob("*.json")):
        value = json.loads(path.read_text(encoding="utf-8"))
        case = value.get("case", value)
        expected_identity = identity(case)
        process = subprocess.run([str(args.rust_executable.resolve()), str(path)], capture_output=True, text=True, check=True, timeout=120)
        actual = json.loads(process.stdout.strip().splitlines()[-1])
        if actual["identity"] != expected_identity:
            raise RuntimeError(f"Canonical identity differs for {path}")
        expected = observe(case)
        observed = actual["observation"]
        passed = observed is not None and observed["body_count"] == expected["body_count"]
        if passed:
            # Volume/area use the checked mesh's approximation; centroid has a
            # length tolerance tied to its 0.01 mm tessellation chord budget.
            for key in ["volume", "surface_area"]:
                passed = passed and abs(observed[key] - expected[key]) <= 1e-6 + .01 * abs(expected[key])
            for a, b in zip(observed["centroid"], expected["centroid"]):
                passed = passed and abs(a - b) <= .01 + .001 * abs(b)
        failed |= not passed
        observations.append({"case": path.name, "identity": expected_identity, "passed": passed,
                             "openrcad": actual, "occt": expected})
    if not observations:
        raise RuntimeError("No differential cases found")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps({"occt_version": OCP.__version__, "tolerances": {"volume_area_relative": 0.01, "volume_area_absolute": 1e-6, "centroid_absolute_mm": 0.01, "centroid_relative": 0.001}, "cases": observations}, indent=2) + "\n", encoding="utf-8")
    print(f"Compared {len(observations)} canonical cases; report: {args.output}")
    if failed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
