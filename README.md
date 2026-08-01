# metoolkit-karma

A re-implementation of the **MathEngine Karma 1.2** physics engine (rigid-body dynamics, collision detection, constraints), written against its original public C API.

**Status:** scaffold only. Nothing is implemented yet.

## Layout

| Path | Contents |
|------|----------|
| `include/` | Original Karma 1.2 public headers. This is the API being re-implemented. |
| `reference/metoolkit-karma-v1.2/` | Submodule [sigmaco/metoolkit-karma-v1.2](https://github.com/sigmaco/metoolkit-karma-v1.2): prebuilt SDK libraries and documentation, kept for reference only. |

## Build

```sh
git submodule update --init   # optional, only needed for the reference material
cmake -B build
cmake --build build
```

Requires CMake 3.20 or newer and a C99/C++17 toolchain. Other projects link against the `karma::karma` target.

## Notes

The headers in `include/` come from the original SDK and keep their MathEngine PLC copyright notices. The implementation is C++ that exports the C API those headers declare.
