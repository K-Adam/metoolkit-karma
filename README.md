# metoolkit-karma

A re-implementation of the **MathEngine Karma 1.2** physics engine (rigid-body dynamics, collision detection, constraints), written against its original public C API, with [Bullet 3](https://github.com/bulletphysics/bullet3) as the backend.

**Status:** every function declared in the headers is defined (`python tools/api_coverage.py`). This includes:
- collision: primitives, convex meshes and aggregates
- dynamics: bodies, contacts with Karma's friction, slip and adhesion, every joint type, and user constraints
- support: XML, the asset database and factory, and the ASE loader

It is meant to be used as a library. The demo and viewer parts are stubs (see below).

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

Requires CMake 3.20 or newer and a C99/C++17 toolchain with floating-point `<charconv>` (MSVC 2019 16.4, GCC 11 or newer).

To use it from another project, either add it with `add_subdirectory` or install it and find the package. Both give the `karma::karma` target. Tests are built only when this is the top-level project (`KARMA_BUILD_TESTS`).

```sh
cmake --install build --config Release --prefix <prefix>
```

```cmake
find_package(karma CONFIG REQUIRED)   # with <prefix> in CMAKE_PREFIX_PATH, and Bullet findable
target_link_libraries(app PRIVATE karma::karma)
```

The libraries are C++, so a C-only consumer still has to enable CXX for linking.

## Design

**Public structs are the real objects.** The headers expose nearly every struct (`MdtBody`, `McdModel`, `McdModelPairContainer`, ...), and client code and header macros read their fields. Each implementation object therefore embeds the public struct as its first member and adds its Bullet state after it:

```cpp
struct McdModelImpl { McdModel pub{}; btCollisionObject object; ... };
inline McdModelImpl* impl(McdModelID m) { return reinterpret_cast<McdModelImpl*>(m); }
```

Fields that clients read (`MdtBody::bodyTM`, `keaBody.vel`, pair container ranges, `McdGeometry::mRefCtAndID`) are kept current.

**The layers stay separate, as in Karma.**

- **Mcd** wraps Bullet collision only. `McdSpace` is a `btCollisionWorld` with a DBVT broadphase, and pair transitions (hello, staying, goodbye) are derived by diffing its overlapping pairs between updates. `McdHello` caches a Bullet collision algorithm on the pair, and `McdIntersect` converts its manifold into `McdContact`s.
- **Mdt** wraps Bullet dynamics only. Bodies never enter Bullet's broadphase. Every enabled `MdtContact` is written into the dispatcher as a one-step manifold before `stepSimulation`, so contacts from any source behave the same. Friction is a custom `btTypedConstraint` per contact that reproduces Kea's force bounds. Each joint owns one persistent custom `btTypedConstraint` whose rows come from the joint type, so every joint shares Karma's limit, motor and lock behaviour.
- **Mst** copies Mcd contacts into Mdt contact groups, exactly like the original bridge.

**Extension points, so new work does not edit shared code:**

- Geometry types register a vtable with `McdFrameworkRegisterGeometryType` and a Bullet shape accessor with `mcdRegisterShapeFn`, from their `RegisterType` function.
- A type pair can override Bullet's narrowphase through the `McdInteractions` table. `McdBoxPlane.cpp` does this for exact box corners.
- Constraint types register an `MdtConstraintOps` table (destroy, per-step prepare, and for joints the Jacobian rows) from their `Create` function. Generic `MdtConstraint*` functions dispatch through it.

**Conventions confirmed against the original libraries:**

- `McdContact::normal` points from model2 to model1, although its header comment says the opposite. `MdtContact` uses the same direction relative to body2 and body1.
- A new `McdSpace` starts inside a change block. Pairs are read between `McdSpaceEndChanges` and `McdSpaceBeginChanges`.
- Primitive-plane contacts follow Karma: 4 corners for a resting box, 4 rim points per cylinder cap, 2 contacts for a sphyl.
- Auto-disable is on by default, and the default inertia is 0.4 times the mass.
- A pair built by hand must get `McdGoodbye` before either of its models is destroyed.

**Header changes.** `include/` matches the original SDK except for one fix needed for 64-bit builds: `MeUintPtr` is `uintptr_t` under `WIN32` (it was 32 bits), and the `MeMemory.h` alignment macros cast through it instead of `long unsigned`.

## Differential tests

`tests/difftest/` holds C scenarios that run unchanged against our libraries and the original 32-bit ones. Each prints a trace. Lines starting with `=` must match exactly, and lines starting with `~tol` must match within `tol`. The golden traces in `tests/difftest/golden/` were recorded from the original libraries and are checked in, so the normal build compares against them without needing the x86 toolchain.

Every `tests/difftest/*.c` is a scenario. To add one, write the file and record its golden trace:

```sh
cmake -S tests/difftest -B build-ref -A Win32
cmake --build build-ref --config Release --target golden
```

A git worktree has no submodule checkout, so there pass `-DKARMA_REFERENCE_DIR=<main checkout>/reference/metoolkit-karma-v1.2` to both builds.

The original libraries were built with a 2002 compiler against `msvcrt.dll`. `msvcrt_shim.c` supplies the old stdio imports that today's CRT no longer exports, and sends their messages to stderr instead of modal dialogs.

## Stubs

- **Viewer and debug drawing:** MeViewer2, MeApp and every `*DebugDraw` function are silent no-ops. Pointer returns are NULL and numeric returns are 0.
- **Karma's solver internals:** MdtKea, the `MdtBclAdd*` functions, MdtPartition, MdtMainLoop and the `MdtWorldGet*Chunk` family. They return neutral values and warn once on stderr through `ME_UNSUPPORTED`, because Bullet replaces that solver.
- **User constraints are real:** the four `MdtBcl` row bookkeeping helpers that user functions call are implemented, and the Kea rows a user function writes are converted into Bullet rows.
- `MstBridgeStepAll` is declared but absent from the original library, so it is stubbed too.

## Known gaps

| Area | Notes |
|------|-------|
| Dynamics | Soft joint rows are solved one at a time, and Kea's epsilon softening of hard joint rows is not applied. A very soft slip row with a finite friction bound is solved stiffer than in Kea. Very uneven mass ratios (RagDolls.ka's lower back) need more solver iterations than Kea. |
| Assets | The `.ka` property tags for rpro joint strengths are a best guess. `MeFJointScale` does not scale car wheel suspension lengths. The asset factory caches geometry by `MeFGeometry` address. The per-asset collision matrix holds 4096 parts, and parts beyond that always collide. |
| Teardown | Mst contact groups point at Mcd pairs, so the world must be destroyed before the space, as `MstUniverseDestroy` does. |

Each new library target is one `karma_add_library(<Name> DEPS ...)` line in `CMakeLists.txt` plus a `src/<Name>/` directory.

## Notes

The headers in `include/` come from the original SDK and keep their MathEngine PLC copyright notices. The implementation is C++ that exports the C API those headers declare.
