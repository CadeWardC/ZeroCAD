"""Optional, test-machine-only STEP verification; OCP never ships with ZeroCAD.

Generate fixtures with ZEROCAD_INTERCHANGE_EVIDENCE_DIR set while running
`cargo test -p zerocad-core --test mechanical_workflows --test step_export`.
Install cadquery-ocp==7.8.1.1.post1 in a separate verification environment.
"""
import argparse
import hashlib
import json
from pathlib import Path

import OCP
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_Reader
from OCP.TopAbs import TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer


def verify(directory):
    records = []
    for path in sorted(directory.glob("*.step")):
        expected_path = path.with_suffix(".json")
        if not expected_path.exists():
            raise RuntimeError(f"Missing expected measurements: {expected_path}")
        expected = json.loads(expected_path.read_text(encoding="utf-8"))
        reader = STEPControl_Reader()
        if reader.ReadFile(str(path.resolve())) != IFSelect_RetDone:
            raise RuntimeError(f"Could not read {path}")
        if reader.TransferRoots() < 1:
            raise RuntimeError(f"No transferable roots in {path}")
        shape = reader.OneShape()
        properties = GProp_GProps()
        BRepGProp.VolumeProperties_s(shape, properties)
        volume = properties.Mass()
        explorer = TopExp_Explorer(shape, TopAbs_SOLID)
        count = 0
        centers = []
        while explorer.More():
            count += 1
            solid_properties = GProp_GProps()
            BRepGProp.VolumeProperties_s(explorer.Current(), solid_properties)
            center = solid_properties.CentreOfMass()
            centers.append([center.X(), center.Y(), center.Z()])
            explorer.Next()
        valid = BRepCheck_Analyzer(shape).IsValid()
        passed = valid and count == expected["body_count"] and abs(volume - expected["volume"]) <= 1e-6 + .01 * abs(expected["volume"])
        if "centers" in expected:
            passed = passed and len(centers) == len(expected["centers"]) and all(
                all(abs(a - b) < 1e-6 for a, b in zip(actual, wanted))
                for actual, wanted in zip(sorted(centers), sorted(expected["centers"]))
            )
        records.append({"file": path.name, "sha256": hashlib.sha256(path.read_bytes()).hexdigest(), "passed": passed, "valid": valid,
                        "body_count": count, "centers_mm": centers, "volume_mm3": volume, "expected": expected})
    if not records:
        raise RuntimeError("No STEP fixtures found")
    return {"occt_version": OCP.__version__, "tolerances": {"volume_relative": 0.01, "volume_absolute_mm3": 1e-6}, "exports": records}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = verify(args.directory)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"Verified {len(report['exports'])} exports; report: {args.output}")
    if not all(record["passed"] for record in report["exports"]):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
