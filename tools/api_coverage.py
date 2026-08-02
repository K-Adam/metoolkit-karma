"""Report which MEAPI functions declared in include/ are defined in src/, per original library.

    python tools/api_coverage.py                 summary per library
    python tools/api_coverage.py --missing Mdt   list the functions Mdt still lacks

Only MEAPI declarations are counted. The few cdecl utilities (MeDict, MeSet, MeHeap, ...) are not.
"""
import re
import sys
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

# Headers whose library is not implied by their prefix, taken from the original library object layout.
LIBRARY = {
    "McdSpace": "McdCommon", "McdContact": "McdCommon",
    "McdBox": "McdPrimitives", "McdCylinder": "McdPrimitives", "McdSphere": "McdPrimitives",
    "McdPlane": "McdPrimitives", "McdSphyl": "McdPrimitives", "McdTriangleList": "McdPrimitives",
    "McdPrimitives": "McdPrimitives",
    "McdConvexMesh": "McdConvex", "McdGjk": "McdConvex", "McdGjkMaximumPoint": "McdConvex",
    "McdPlaneIntersect": "McdConvex", "McdPolygonIntersection": "McdConvex",
    "MdtKea": "MdtKea", "MdtBcl": "MdtBcl",
    "MeXMLParser": "MeXML", "MeXMLTree": "MeXML", "MeXMLOutput": "MeXML",
    "MeAssetDB": "MeAssetDB", "MeAssetDBXMLIO": "MeAssetDBXMLIO", "MeAssetFactory": "MeAssetFactory",
    "MeViewer": "MeViewer2", "RMenu": "MeViewer2", "RGeometryUtils": "MeViewer2",
    "MeApp": "MeApp", "RGeometry": "MeApp", "RConvex": "MeApp",
    # Not built into any original library.
    "McdCone": None, "RTogglemenu": None,
}
PREFIX = [("Mcd", "McdFrame"), ("Mdt", "Mdt"), ("Mst", "Mst"), ("Me", "MeGlobals")]
GEOMETRY_TYPE_FUNCTIONS = ["GetTypeId", "RegisterType", "Destroy", "UpdateAABB", "GetBSphere", "MaximumPoint",
                           "GetMassProperties", "GetTypeName", "DebugDraw"]


def library_of(header):
    if header in LIBRARY:
        return LIBRARY[header]
    return next((lib for prefix, lib in PREFIX if header.startswith(prefix)), None)


def strip_comments(text):
    return re.sub(r"//[^\n]*", "", re.sub(r"/\*.*?\*/", "", text, flags=re.S))


def declared():
    funcs = {}
    for path in sorted((ROOT / "include").glob("*.h")):
        lib = library_of(path.stem)
        if not lib:
            continue
        text = strip_comments(path.read_text(encoding="latin-1"))
        # Skip the macro definitions themselves, keep their uses.
        body = re.sub(r"#define[^\n]*(\\\n[^\n]*)*", "", text)
        # Functions the header defines inline come with the header, not with a library.
        inline = set(re.findall(r"\bMeINLINE\b[^;{(]*?(\w+)\s*\(", body))
        for name in re.findall(r"\bMEAPI\s*\*?\s*(\w+)\s*\(", body):
            if "##" not in name and name not in inline:
                funcs[name] = lib
        for t in re.findall(r"^\s*MCD_DECLARE_GEOMETRY_TYPE\s*\(\s*(\w+)\s*\)", body, flags=re.M):
            for suffix in GEOMETRY_TYPE_FUNCTIONS:
                funcs[t + suffix] = lib
    return funcs


def defined():
    names = set()
    for path in (ROOT / "src").rglob("*.cpp"):
        names.update(re.findall(r"\bMEAPI\s*\*?\s*(\w+)\s*\([^;{]*\)\s*\{", strip_comments(path.read_text())))
    return names


def main(argv):
    funcs, done = declared(), defined()
    if len(argv) == 3 and argv[1] == "--missing":
        for name in sorted(n for n, lib in funcs.items() if lib == argv[2] and n not in done):
            print(name)
        return 0
    total, have = defaultdict(int), defaultdict(int)
    for name, lib in funcs.items():
        total[lib] += 1
        have[lib] += name in done
    for lib in sorted(total):
        print(f"{lib:16} {have[lib]:4} / {total[lib]:4}")
    print(f"{'total':16} {sum(have.values()):4} / {sum(total.values()):4}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
