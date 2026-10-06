#!/usr/bin/env python3
"""Export only overlay-owned rigid lift bodies from the original STEP.

No URDF generation: the authored xacro owns the kinematic tree. This tool
exports link-local STL meshes and computed geometry/inertial measurements.
"""

import argparse
import hashlib
import json
from pathlib import Path
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TDF import TDF_LabelSequence, TDF_Label
from OCP.TDataStd import TDataStd_Name
from OCP.TopLoc import TopLoc_Location
from OCP.gp import gp_Trsf, gp_Vec
from OCP.TopoDS import TopoDS_Compound
from OCP.BRep import BRep_Builder
from OCP.BRepBndLib import BRepBndLib
from OCP.Bnd import Bnd_Box
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.StlAPI import StlAPI_Writer

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / "src" / "handy101_description"
# Matching the STEP deck centre and top to base101's existing deck.
CAD_DECK_TOP_MM = (-0.43313250817525, 1.5, 64.79589027896802)
BASE_DECK_TOP_M = (0.0, 0.0, 0.084965)
STATIONARY = ("extrusion", "bearing", "screw", "servo_adapter", "servo", "screw_end", "top", "base")
MOVING = ("main", "nut", "mgn12_carriage")
# Provisional materials: STEP has geometry, not validated material/mass data.
DENSITY_KG_M3 = {
    "extrusion": 2700,
    "bearing": 7850,
    "screw": 7850,
    "servo_adapter": 1240,
    "servo": 1240,
    "screw_end": 7850,
    "top": 1240,
    "base": 1240,
    "main": 1240,
    "nut": 8500,
    "mgn12_rail": 7850,
    "mgn12_carriage": 7850,
    "crossbar": 2700,
    "top_camera_mount": 1240,
    "head_tilt_servo": 1240,
    "realsense_support v1": 1240,
    "camera_realsense": 2700,
}
HEAD_PARTS = ("top_camera_mount", "head_tilt_servo", "realsense_support v1", "camera_realsense")


def material(key):
    if key == "camera_realsense" or key == "lift_crossbar" or key.endswith("_extrusion"):
        return "handy_aluminum"
    if key.endswith(("_bearing", "_screw", "_screw_end", "_mgn12_rail", "_mgn12_carriage")):
        return "handy_steel"
    if key.endswith("_nut"):
        return "handy_brass"
    return "handy_black_plastic"


def write_mesh(shape, path):
    BRepMesh_IncrementalMesh(shape, 0.2, False, 0.25, True)
    writer = StlAPI_Writer()
    writer.ASCIIMode = False
    if not writer.Write(shape, str(path)):
        raise RuntimeError("STL export failed: " + str(path))


def bounds(shape):
    box = Bnd_Box()
    BRepBndLib.AddOptimal_s(shape, box, False, False)
    return list(box.Get())


def name(label):
    attr = TDataStd_Name()
    return attr.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(), attr) else "?"


def read_parts(path):
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    if int(reader.ReadFile(str(path))) != 1:
        raise RuntimeError("Could not read STEP")
    doc = TDocStd_Document(TCollection_ExtendedString("lift export"))
    if not reader.Transfer(doc):
        raise RuntimeError("Could not transfer STEP")
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    roots = TDF_LabelSequence()
    tool.GetFreeShapes(roots)
    parts = {}

    def visit(label, loc):
        target = label
        if tool.IsReference_s(label):
            target = TDF_Label()
            tool.GetReferredShape_s(label, target)
            loc = loc.Multiplied(tool.GetLocation_s(label))
        children = TDF_LabelSequence()
        if tool.GetComponents_s(target, children):
            for i in range(1, children.Length() + 1):
                visit(children.Value(i), loc)
        elif name(target).startswith("lift_") or name(target) in HEAD_PARTS:
            part_name = name(target)
            if part_name in parts:
                raise RuntimeError("Duplicate lift occurrence: " + part_name)
            parts[part_name] = tool.GetShape_s(target).Moved(loc)

    for i in range(1, roots.Length() + 1):
        visit(roots.Value(i), TopLoc_Location())
    return parts


def move(shape, xyz):
    tr = gp_Trsf()
    tr.SetTranslation(gp_Vec(*xyz))
    return shape.Moved(TopLoc_Location(tr))


def export_body(parts, label, names, origin):
    builder = BRep_Builder()
    compound = TopoDS_Compound()
    builder.MakeCompound(compound)
    mass = 0.0
    first_moment = [0.0] * 3
    # Sum inertias about the link origin, then shift to combined COM.
    inertia_origin = [[0.0] * 3 for _ in range(3)]
    components = []
    visual_groups = {}
    for key in names:
        local = move(parts[key], [-v for v in origin])
        builder.Add(compound, local)
        mat = material(key)
        if mat not in visual_groups:
            visual_groups[mat] = TopoDS_Compound()
            builder.MakeCompound(visual_groups[mat])
        builder.Add(visual_groups[mat], local)
        prop = GProp_GProps()
        BRepGProp.VolumeProperties_s(local, prop)
        suffix = (
            key
            if key in HEAD_PARTS
            else key.split("_", 2)[2]
            if key != "lift_crossbar"
            else "crossbar"
        )
        density = DENSITY_KG_M3[suffix]
        m = prop.Mass() * 1e-9 * density
        c = prop.CentreOfMass()
        com = [c.X() * 1e-3, c.Y() * 1e-3, c.Z() * 1e-3]
        # The motor has mixed materials: provisional 55 g whole-component mass.
        factor = density * 1e-15
        if suffix in ("servo", "head_tilt_servo", "camera_realsense"):
            target_mass = 0.072 if suffix == "camera_realsense" else 0.055
            factor *= target_mass / m
            m = target_mass
        tensor = prop.MatrixOfInertia()
        for i in range(3):
            first_moment[i] += m * com[i]
            for j in range(3):
                inertia_origin[i][j] += tensor.Value(i + 1, j + 1) * factor + m * (
                    (sum(v * v for v in com) if i == j else 0) - com[i] * com[j]
                )
        mass += m
        box = bounds(local)
        components.append({"name": key, "density_kg_m3": density, "mass_kg": m, "bounds_mm": box})
    com = [x / mass for x in first_moment]
    inertia = [
        [
            inertia_origin[i][j]
            - mass * ((sum(v * v for v in com) if i == j else 0) - com[i] * com[j])
            for j in range(3)
        ]
        for i in range(3)
    ]
    write_mesh(compound, PKG / "meshes" / (label + ".stl"))
    visuals = []
    for mat, group in visual_groups.items():
        filename = label + "_" + mat.removeprefix("handy_") + ".stl"
        write_mesh(group, PKG / "meshes" / filename)
        visuals.append({"filename": filename, "material": mat, "bounds_local_mm": bounds(group)})
    base_origin = [(origin[i] - CAD_DECK_TOP_MM[i]) * 0.001 + BASE_DECK_TOP_M[i] for i in range(3)]
    return {
        "link": label,
        "origin_cad_mm": origin,
        "origin_base_m": base_origin,
        "mass_kg": mass,
        "com_m": com,
        "inertia_kg_m2": inertia,
        "bounds_local_mm": bounds(compound),
        "components": components,
        "visuals": visuals,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("step", type=Path)
    args = ap.parse_args()
    parts = read_parts(args.step)
    outputs = []
    for side in ("left", "right"):
        stem = "lift_" + side + "_"
        # Nut centre defines the mobile link frame; axes remain CAD XYZ.
        nut = bounds(parts[stem + "nut"])
        origin = [(nut[i] + nut[i + 3]) / 2 for i in range(3)]
        outputs.append(
            export_body(
                parts,
                stem + "structure",
                [stem + x for x in STATIONARY] + [stem + "mgn12_rail"],
                CAD_DECK_TOP_MM,
            )
        )
        outputs.append(export_body(parts, stem + "main", [stem + x for x in MOVING], origin))
    outputs.append(export_body(parts, "lift_crossbar", ["lift_crossbar"], CAD_DECK_TOP_MM))
    outputs.append(
        export_body(
            parts, "head_camera_mount", ["top_camera_mount", "head_tilt_servo"], CAD_DECK_TOP_MM
        )
    )
    cam = bounds(parts["camera_realsense"])
    origin = [(cam[i] + cam[i + 3]) / 2 for i in range(3)]
    outputs.append(
        export_body(parts, "head_camera_link", ["realsense_support v1", "camera_realsense"], origin)
    )
    data = {
        "source_name": args.step.name,
        "source_sha256": hashlib.sha256(args.step.read_bytes()).hexdigest(),
        "cad_deck_top_mm": CAD_DECK_TOP_MM,
        "base_deck_top_m": BASE_DECK_TOP_M,
        "excluded": ["lift_left_carriage", "lift_right_carriage"],
        "mesh_units": "mm",
        "inertials": "estimated from CAD solid volumes and assumed densities; servo 55 g",
        "bodies": outputs,
    }
    (PKG / "config" / "cad_measurements.json").write_text(json.dumps(data, indent=2) + "\n")
    print("Exported", len(outputs), "link-local lift meshes")


if __name__ == "__main__":
    main()
