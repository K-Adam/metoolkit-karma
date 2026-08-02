# metoolkit-karma

A re-implementation of the **MathEngine Karma 1.2** physics engine (rigid-body dynamics, collision detection, constraints), written against its original public C API, with [Bullet 3](https://github.com/bulletphysics/bullet3) as the backend.

**Status:** one vertical slice works end to end. A sphere or box falls onto a plane through `MstUniverse`, with collision pairs, contacts and dynamics. Run `python tools/api_coverage.py` for per-library progress.

## Layout

| Path | Contents |
|------|----------|
| `include/` | Original Karma 1.2 public headers. This is the API being re-implemented. |
| `src/<Library>/` | One directory per original library (McdFrame, Mdt, Mst, ...), one `.cpp` per public header. |
| `src/internal/` | Private headers shared across libraries: Bullet conversions and the impl structs. |
| `tests/` | CTest executables, plus `tests/difftest/` which compares traces with the original libraries. |
| `tools/api_coverage.py` | Lists implemented and missing API functions per library. |
| `reference/metoolkit-karma-v1.2/` | Submodule [sigmaco/metoolkit-karma-v1.2](https://github.com/sigmaco/metoolkit-karma-v1.2): prebuilt SDK libraries and documentation, kept for reference only. |

## Build

Bullet comes from vcpkg through the `vcpkg.json` manifest. Point `VCPKG_ROOT` at a vcpkg checkout, or pass `-DCMAKE_TOOLCHAIN_FILE=<vcpkg>/scripts/buildsystems/vcpkg.cmake`.

```sh
git submodule update --init   # only needed for the reference material and golden traces
export VCPKG_ROOT=D:/vcpkg
cmake -B build
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

Requires CMake 3.20 or newer and a C99/C++17 toolchain. Other projects link against the `karma::karma` target.

## Design

**Public structs are the real objects.** The headers expose nearly every struct (`MdtBody`, `McdModel`, `McdModelPairContainer`, ...), and client code and header macros read their fields. Each implementation object therefore embeds the public struct as its first member and adds its Bullet state after it:

```cpp
struct McdModelImpl { McdModel pub{}; btCollisionObject object; ... };
inline McdModelImpl* impl(McdModelID m) { return reinterpret_cast<McdModelImpl*>(m); }
```

Fields that clients read (`MdtBody::bodyTM`, `keaBody.vel`, pair container ranges, `McdGeometry::mRefCtAndID`) are kept current.

**The layers stay separate, as in Karma.**

- **Mcd** wraps Bullet collision only. `McdSpace` is a `btCollisionWorld` with a DBVT broadphase, and pair transitions (hello, staying, goodbye) are derived by diffing its overlapping pairs between updates. `McdHello` caches a Bullet collision algorithm on the pair, and `McdIntersect` converts its manifold into `McdContact`s.
- **Mdt** wraps Bullet dynamics only. Bodies never enter Bullet's broadphase. Every enabled `MdtContact` is written into the dispatcher as a one-step manifold before `stepSimulation`, so contacts from any source behave the same.
- **Mst** copies Mcd contacts into Mdt contact groups, exactly like the original bridge.

**Extension points, so new work does not edit shared code:**

- Geometry types register a vtable with `McdFrameworkRegisterGeometryType` and a Bullet shape accessor with `mcdRegisterShapeFn`, from their `RegisterType` function.
- A type pair can override Bullet's narrowphase through the `McdInteractions` table. `McdBoxPlane.cpp` does this for exact box corners.
- Constraint types register an `MdtConstraintOps` table (destroy, per-step prepare) from their `Create` function. Generic `MdtConstraint*` functions dispatch through it.

**Conventions confirmed against the original libraries:**

- `McdContact::normal` points from model2 to model1, although its header comment says the opposite. `MdtContact` uses the same direction relative to body2 and body1.
- A new `McdSpace` starts inside a change block. Pairs are read between `McdSpaceEndChanges` and `McdSpaceBeginChanges`.

**Header changes.** `include/` matches the original SDK except for one fix needed for 64-bit builds: `MeUintPtr` is `uintptr_t` under `WIN32` (it was 32 bits), and the `MeMemory.h` alignment macros cast through it instead of `long unsigned`.

## Differential tests

`tests/difftest/` holds C scenarios that run unchanged against our libraries and the original 32-bit ones. Each prints a trace. Lines starting with `=` must match exactly, and lines starting with `~tol` must match within `tol`. The golden traces in `tests/difftest/golden/` were recorded from the original libraries and are checked in, so the normal build compares against them without needing the x86 toolchain.

To add a scenario, write `tests/difftest/<name>.c`, add it to the list in `tests/difftest/CMakeLists.txt` and record its golden trace:

```sh
cmake -S tests/difftest -B build-ref -A Win32
cmake --build build-ref --config Release --target golden
```

The original libraries were built with a 2002 compiler against `msvcrt.dll`. `msvcrt_shim.c` supplies the old stdio imports that today's CRT no longer exports.

## Workstreams

The slice above is the template. These areas are independent of each other and can proceed in parallel. `python tools/api_coverage.py --missing <Library>` lists what each still lacks.

| # | Area | Libraries | Notes |
|---|------|-----------|-------|
| 1 | Shared utilities | MeGlobals | MeMath out-of-line functions, MeDict (its header macros must be `#undef`ed before defining the functions), MeHash, MePool, MeChunk, MeDebugDraw. |
| 2 | Primitives and queries | McdPrimitives, McdFrame, McdCommon | Cylinder (`btCylinderShapeZ`), sphyl (`btCapsuleShapeZ`), triangle list (`btBvhTriangleMeshShape`), line segment queries (`rayTest`), culling tables, `McdSpaceGetTransitions`, pair enable and disable. |
| 3 | Convex and aggregate | McdConvex, McdConvexCreateHull, McdFrame | `btConvexHullShape` and `btCompoundShape`. |
| 4 | Bodies and contacts | Mdt | Auto-disable with Karma's thresholds, damping, impulses, centre of mass offset, full inertia tensor, epsilon and softness mapping, box friction, slip and slide. |
| 5 | Joints | Mdt, MdtBcl | Hinge (`btHingeConstraint`), ball and socket (`btPoint2PointConstraint`), prismatic (`btSliderConstraint`), car wheel, Spring6 and RPRO (`btGeneric6DofSpring2Constraint`), skeletal and cone limit (`btConeTwistConstraint`), limits, linear, angular, fixed path, user constraints. |
| 6 | Bridge | Mst | `MstBridgeUpdateTransitions`, `McdModelDynamicsReset`, batch contexts. |
| 7 | Assets | MeXML, MeAssetDB, MeAssetDBXMLIO, MeAssetFactory | No Bullet involved. `reference/.../KarmaData/RagDolls.ka` is the acceptance input. |
| 8 | Viewer | MeViewer2, MeApp | Deferred until a renderer is chosen. |

Each new library target is one `karma_add_library(<Name> DEPS ...)` line in `CMakeLists.txt` plus a `src/<Name>/` directory.

## Notes

The headers in `include/` come from the original SDK and keep their MathEngine PLC copyright notices. The implementation is C++ that exports the C API those headers declare.
