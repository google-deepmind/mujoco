# MuJoCo Studio

MuJoCo Studio is the next iteration of the [simulate](../../../simulate)
application. The UI has been reimplemented using [Dear ImGui](https://github.com/ocornut/imgui)
and the default renderer has been switched to Filament. The application is
still WIP, see the [Future Work](#future-work) section for details.

## Usage

Configure and build MuJoCo Studio by running this command from the top-level
directory. Then follow the printed instructions to run the executable.

```
bash build.sh
```

> [!NOTE]
> For now [`build.sh`](build.sh) script works on windows in a git bash shell.

To keep Studio build artifacts separate from other MuJoCo builds, you can also
configure an isolated build directory manually:

```
cmake -B build-studio \
  -DCMAKE_BUILD_TYPE=Release \
  -DUSE_STATIC_LIBCXX=OFF \
  -DBUILD_SHARED_LIB=OFF \
  -DMUJOCO_USE_FILAMENT=ON \
  -DMUJOCO_BUILD_EXAMPLES=OFF \
  -DMUJOCO_BUILD_SIMULATE=OFF \
  -DMUJOCO_BUILD_TESTS=OFF \
  -DMUJOCO_TEST_PYTHON_UTIL=OFF \
  -DMUJOCO_WITH_USD=OFF \
  -DMUJOCO_BUILD_STUDIO=ON \
  -DFILAMENT_SKIP_SAMPLES=ON \
  -DCMAKE_CXX_FLAGS=-Wno-error=deprecated-declarations \
  -DFILAMENT_SHORTEN_MSVC_COMPILATION=OFF
cmake --build build-studio --config Release --target mujoco_studio --parallel
```

Run `mujoco_studio` from the build output directory so it can find the copied
font and Filament assets:

```
cd build-studio/bin
./mujoco_studio --gfx=opengl
```

On Linux, build Filament-backed targets with a single C++ standard library. If
Filament is built with `libc++`, configure MuJoCo with clang and matching
`-stdlib=libc++` compiler/linker flags to avoid mixed-stdlib link failures.

On macOS, Studio defaults to Filament OpenGL. The OpenGL implementation may be
provided by Apple's Metal-backed OpenGL layer, so runtime logs can mention both
OpenGL and Metal/Apple GPU details.

### Troubleshooting

If configure fails while resolving Filament dependencies, check whether CMake is
finding package-manager CMake configs from environments such as Anaconda. In
particular, an unrelated `abslConfig.cmake` can conflict with MuJoCo's fetched
Abseil targets. Remove that prefix from `CMAKE_PREFIX_PATH`, or configure with
`CMAKE_IGNORE_PREFIX_PATH` pointing at the conflicting environment prefix.

## Development

The [`build.sh`](build.sh) script is intended to get you up and running quickly.
If you intend to develop the application you may prefer to work from an IDE:

* [Clion](https://www.jetbrains.com/clion/). You should be able to set this up
  to work with the cmake files we provide.

* [VSCode](https://code.visualstudio.com/). We have found that Microsoft's
  [CMake Tools](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cmake-tools)
  extension works well.

* [Visual Studio](https://visualstudio.microsoft.com/). Follow these
  [instructions](https://learn.microsoft.com/en-us/cpp/build/cmake-projects-in-visual-studio?view=msvc-170).

## Known Bugs

* MuJoCo Studio does not yet work using Wayland on Linux, use X11 instead.

## Future Work

1. **Stability and Robustness**. We need user feedback to find and fix bugs.

1. **UI/UX improvements**. We have ported the simulate UI to make it easier to
users to switch. We will be making further changes to make use of the
flexibility offered by Dear ImGui ([examples](https://github.com/ocornut/imgui/issues/8942)).

1. **Python integration**. As with simulate, we would like to make Studio usable
  via Python.
