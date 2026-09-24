"""Check the exported shared-definition assembly using independent OCCT XCAF."""
import argparse
import json
from pathlib import Path

from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label, TDF_LabelSequence
from OCP.TDocStd import TDocStd_Document
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ShapeTool


def verify(path):
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    assert reader.ReadFile(str(path.resolve())) == IFSelect_RetDone
    document = TDocStd_Document(TCollection_ExtendedString("assembly verification"))
    assert reader.Transfer(document)
    tool = XCAFDoc_DocumentTool.ShapeTool_s(document.Main())
    roots = TDF_LabelSequence()
    tool.GetFreeShapes(roots)
    assert roots.Length() == 1
    components = TDF_LabelSequence()
    assert XCAFDoc_ShapeTool.GetComponents_s(roots.Value(1), components)
    assert components.Length() == 3
    definitions, names = [], []
    for index in range(1, components.Length() + 1):
        component = components.Value(index)
        definition = TDF_Label()
        assert XCAFDoc_ShapeTool.GetReferredShape_s(component, definition)
        definitions.append(definition)
        name = TDataStd_Name()
        assert component.FindAttribute(TDataStd_Name.GetID_s(), name)
        names.append(name.Get().ToExtString())
    assert all(label.IsEqual(definitions[0]) for label in definitions)
    assert "Rotated plate" in names
    return {"passed": True, "root_assemblies": 1, "components": 3,
            "shared_definitions": 1, "occurrence_names": names}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("file", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    result = verify(args.file)
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(result)
