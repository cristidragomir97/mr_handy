#!/usr/bin/env python3
"""Expand the authored overlay xacro using installed packages or sibling sources."""

import argparse
import os
from pathlib import Path
import subprocess
import shutil
import tempfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--output", type=Path, default=ROOT / "tmp" / "handy101.urdf")
    ap.add_argument(
        "--bundle-meshes",
        action="store_true",
        help="Copy referenced assets into an ignored preview bundle for CAD Viewer",
    )
    ap.add_argument(
        "--local-meshes",
        action="store_true",
        help="Resolve package meshes for CAD Viewer and standalone consumers",
    )
    ap.add_argument("mappings", nargs="*", help="xacro arguments, e.g. left_lift_travel:=0.3")
    args = ap.parse_args()
    with tempfile.TemporaryDirectory(prefix="handy101-ament-") as td:
        prefix = Path(td)
        markers = prefix / "share" / "ament_index" / "resource_index" / "packages"
        markers.mkdir(parents=True)
        for workspace in (ROOT, Path("/opt/handy101_dependencies"),
                          ROOT.parent / "base101", ROOT.parent / "mod101"):
            for manifest in (workspace / "src").rglob("package.xml"):
                name = ET.parse(manifest).getroot().findtext("name")
                share = prefix / "share" / name
                if not share.exists():
                    share.symlink_to(manifest.parent.resolve())
                    (markers / name).touch()
        env = os.environ.copy()
        env["AMENT_PREFIX_PATH"] = td + os.pathsep + env.get("AMENT_PREFIX_PATH", "")
        out = subprocess.check_output(
            [
                "xacro",
                str(ROOT / "src" / "handy101_description" / "urdf" / "handy101.xacro"),
                *args.mappings,
            ],
            env=env,
        )
        robot = ET.fromstring(out)
        for mesh in robot.iter("mesh"):
            filename = mesh.attrib["filename"]
            if filename.startswith("file://"):
                mesh.set("filename", str(Path(filename[7:]).resolve()))
            elif (args.local_meshes or args.bundle_meshes) and filename.startswith("package://"):
                name, relative = filename[10:].split("/", 1)
                mesh.set("filename", str((prefix / "share" / name / relative).resolve()))
        if args.bundle_meshes:
            for mesh in robot.iter("mesh"):
                path = Path(mesh.attrib["filename"])
                if not path.is_file():
                    raise FileNotFoundError(path)
                # Preserve package ownership in the temporary bundle names.
                package = next(
                    (p.parent.name for p in path.parents if p.name == "meshes"), "assets"
                )
                relative = Path("meshes") / package / path.name
                target = args.output.resolve().parent / relative
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(path, target)
                mesh.set("filename", relative.as_posix())
        elif args.local_meshes:
            for mesh in robot.iter("mesh"):
                path = mesh.attrib["filename"]
                if Path(path).is_absolute():
                    mesh.set("filename", os.path.relpath(path, args.output.resolve().parent))
        # Keep design-ledger comments from the authored xacro in the output.
        # Only substitute mesh paths; don't reserialize the whole document.
        for old_mesh, new_mesh in zip(ET.fromstring(out).iter("mesh"), robot.iter("mesh")):
            old = old_mesh.attrib["filename"].encode()
            new = new_mesh.attrib["filename"].encode()
            out = out.replace(old, new)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_bytes(out)
        print(args.output)


if __name__ == "__main__":
    main()
