# Build pitfalls and `eltanin_vendor`

The three commands that build the workspace are in the
[README](https://github.com/RyuYamamoto/eltanin_ros#build). This page covers what goes wrong around
them: two rebuild traps that produce confusing failures, and the options of `eltanin_vendor`.

The design reasoning is in the Japanese design document
[`eltaninnavyuros.md`](https://github.com/RyuYamamoto/eltanin_ros/blob/main/docs/design/eltaninnavyuros.md),
referenced by section number.

## Deleting a message needs `eltanin_msgs` rebuilt clean

`rosidl` keeps generating from what is in `CMakeLists.txt`, but it does not remove what a previous
build produced. Drop a `.msg` and rebuild incrementally and the C++ side is fine while the Python
type support still exports the old symbol, so the whole package fails to import:

```
ImportError: .../eltanin_msgs_s__rosidl_typesupport_c.so:
    undefined symbol: eltanin_msgs__msg__follower_diagnostic__convert_to_py
```

It is one `rclpy` node away from being the first thing anyone notices, and nothing in the C++ build
sees it. After removing or renaming an interface:

```bash
rm -rf build/eltanin_msgs install/eltanin_msgs
```

CI builds from scratch and so never hits this.

## `--packages-ignore eltanin` is not optional

colcon discovers `src/eltanin` as a plain `cmake` package even though it has no `package.xml`, so
without this flag it builds `eltanin` a second time, alongside `eltanin_vendor`. Pass the flag to
`colcon test` too. Three things go wrong otherwise:

- A second prefix `install/eltanin` appears, built **without** `-fPIC`. `find_package(eltanin)` can
  resolve to it, and which prefix wins depends on the order of `CMAKE_PREFIX_PATH` — so a node links
  fine today and fails, or misbehaves, after an unrelated change.
- `eltanin`'s own tests are enabled (they default to `ON` when `eltanin` is the top-level project)
  and require GTest, which need not be present.
- `eltanin`'s 432 tests join `colcon test`, duplicating `eltanin`'s own CI.

Symptoms that the flag was forgotten: `install/eltanin/` exists, or `eltanin`'s tests are being
compiled. Delete `build/eltanin install/eltanin` and rebuild.

## `eltanin_vendor` options

| Cache variable | Default | Meaning |
|---|---|---|
| `ELTANIN_VENDOR_SOURCE_DIR` | `../../eltanin` from the package | Source tree to build, used when `ELTANIN_VENDOR_GIT_URL` is empty |
| `ELTANIN_VENDOR_GIT_URL` | empty | Clone from here instead; requires `ELTANIN_VENDOR_GIT_TAG` |
| `ELTANIN_VENDOR_GIT_TAG` | empty | Revision for `ELTANIN_VENDOR_GIT_URL` |
| `ELTANIN_VENDOR_BUILD_TYPE` | `RelWithDebInfo` | `CMAKE_BUILD_TYPE` for the `eltanin` sub-build only |

The sub-build always gets `-DCMAKE_POSITION_INDEPENDENT_CODE=ON`. `eltanin` declares every module
`STATIC` and does not set that variable itself, while every node here is an `rclcpp_components`
shared library (design §3.3). Without the flag, linking a module into a shared object fails with an
`R_X86_64_PC32` relocation error — four of the eight modules at `-O0`, two of them under
`RelWithDebInfo` — and the rest link by accident. Which ones fail depends on the build type, so the
breakage would surface only once a later node happened to use one of them, or once someone changed
the optimization level. `eltanin_vendor/test/link_check/` builds one `SHARED` library per module with
`WHOLE_ARCHIVE` and `-Wl,--no-undefined` to keep that fixed; it needs CMake 3.24 for the
`WHOLE_ARCHIVE` link feature, which is why the requirements table in the
[README](https://github.com/RyuYamamoto/eltanin_ros#requirements) names 3.24 for this package.
Because it is part of the build rather than a test, a module that cannot be linked turns
`colcon build` red. Downstream packages do not need to repeat the check.

`BUILD_ALWAYS ON` makes the sub-build re-run every time, so an edit in `src/eltanin` is never missed.
Measured on the development machine: 11.9 s clean, 0.9 s for a no-op rebuild.

`ExternalProject` never removes what it installed. After deleting or renaming a header in `eltanin`,
the stale copy stays in `install/eltanin_vendor/include` and consumers keep including it; run
`rm -rf build/eltanin_vendor install/eltanin_vendor` to clear it.
