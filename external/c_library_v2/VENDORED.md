# MAVLink C library v2 (vendored)

Header-only MAVLink 2 bindings, vendored (not a git submodule) so that on-device and
cross-compile builds need no network access at configure time.

| | |
|---|---|
| Upstream | https://github.com/mavlink/c_library_v2 |
| Pinned commit | `a6abd1787f90ba38ec117dcfeeb511beab75b398` |
| Generated from | https://github.com/mavlink/mavlink/tree/f8b1d319d6cea3f25fea25ad30b48d73c6c623ea |
| Vendored on | 2026-08-14 |

## Local modifications

All dialects except `common/` were deleted to keep the tree small; upstream ships ~20
dialects (~25 MB). `common/` is the only dialect this project uses.

## Updating

Re-clone upstream at the desired commit, copy the top-level `*.h` files plus `common/`,
and update the pinned commit above:

```sh
git clone --depth 1 https://github.com/mavlink/c_library_v2 /tmp/mavlink
rm -rf homework_07/external/c_library_v2
mkdir -p homework_07/external/c_library_v2
cp /tmp/mavlink/*.h homework_07/external/c_library_v2/
cp -r /tmp/mavlink/common homework_07/external/c_library_v2/
```

## Use from CMake

Link the `mavlink` INTERFACE target defined in `homework_07/external/CMakeLists.txt`:

```cmake
target_link_libraries(my_target PRIVATE mavlink)
```

Then `#include <common/mavlink.h>`.
