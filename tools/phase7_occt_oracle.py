"""Generate the frozen Phase 7 differential oracle with OCCT Python bindings.

This tool is release/test-only. It is never imported, linked, or shipped by
ZeroCAD or OpenRCAD. The committed JSON records the exact OCP/OCCT version, this
generator path, and the SHA-256 of the foreign STEP input.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import OCP
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common, BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepGProp import BRepGProp
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder
from OCP.GProp import GProp_GProps
from OCP.STEPControl import STEPControl_Reader
from OCP.TopAbs import TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt, gp_Trsf


def properties(shape):
    volume = GProp_GProps()
    surface = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, volume)
    BRepGProp.SurfaceProperties_s(shape, surface)
    center = volume.CentreOfMass()
    return {
        "classification": "solid" if solid_count(shape) else "empty_or_contact",
        "volume": volume.Mass(),
        "surface_area": surface.Mass(),
        "centroid": [center.X(), center.Y(), center.Z()],
    }


def solid_count(shape):
    count = 0
    explorer = TopExp_Explorer(shape, TopAbs_SOLID)
    while explorer.More():
        count += 1
        explorer.Next()
    return count


def case(case_id, category, shape):
    return {"id": case_id, "category": category, **properties(shape)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--step-fixture",
        type=Path,
        default=Path("zerocad-core/tests/fixtures/step/mayo-freecad-cube.step"),
    )
    args = parser.parse_args()

    box = BRepPrimAPI_MakeBox(10.0, 8.0, 6.0).Shape()
    cylinder = BRepPrimAPI_MakeCylinder(
        gp_Ax2(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)), 3.0, 8.0
    ).Shape()
    fuse_a = BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape()
    fuse_b = BRepPrimAPI_MakeBox(gp_Pnt(5.0, 0.0, 0.0), 10.0, 10.0, 10.0).Shape()
    hole_box = BRepPrimAPI_MakeBox(10.0, 8.0, 6.0).Shape()
    hole_tool = BRepPrimAPI_MakeCylinder(
        gp_Ax2(gp_Pnt(5.0, 4.0, -1.0), gp_Dir(0.0, 0.0, 1.0)), 2.0, 8.0
    ).Shape()
    common_b = BRepPrimAPI_MakeBox(gp_Pnt(5.0, 0.0, 0.0), 10.0, 10.0, 10.0).Shape()

    scale = gp_Trsf()
    scale.SetScale(gp_Pnt(0.0, 0.0, 0.0), 2.0)
    scaled = BRepBuilderAPI_Transform(box, scale, True).Shape()

    reader = STEPControl_Reader()
    status = reader.ReadFile(str(args.step_fixture.resolve()))
    if "RetDone" not in str(status) or reader.TransferRoots() == 0:
        raise RuntimeError(f"OCCT could not import {args.step_fixture}: {status}")
    imported = reader.OneShape()

    disjoint = BRepPrimAPI_MakeBox(gp_Pnt(20.0, 0.0, 0.0), 2.0, 2.0, 2.0).Shape()
    tangent = BRepPrimAPI_MakeBox(gp_Pnt(10.0, 0.0, 0.0), 2.0, 2.0, 2.0).Shape()

    cases = [
        case("primitive_box", "primitive", box),
        case("primitive_cylinder", "primitive", cylinder),
        case("boolean_fuse_boxes", "boolean", BRepAlgoAPI_Fuse(fuse_a, fuse_b).Shape()),
        case("boolean_cut_through_hole", "boolean", BRepAlgoAPI_Cut(hole_box, hole_tool).Shape()),
        case("boolean_common_boxes", "boolean", BRepAlgoAPI_Common(fuse_a, common_b).Shape()),
        case("import_mayo_cube", "import", imported),
        case("direct_uniform_scale", "direct_edit", scaled),
        case("failure_disjoint_common", "failure_classification", BRepAlgoAPI_Common(fuse_a, disjoint).Shape()),
        case("failure_tangent_common", "failure_classification", BRepAlgoAPI_Common(fuse_a, tangent).Shape()),
    ]
    document = {
        "schema": 1,
        "generator": "tools/phase7_occt_oracle.py",
        "occt_version": OCP.__version__,
        "step_fixture": str(args.step_fixture).replace("\\", "/"),
        "step_fixture_sha256": hashlib.sha256(args.step_fixture.read_bytes()).hexdigest(),
        "cases": cases,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(document, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
