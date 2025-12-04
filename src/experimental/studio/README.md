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

## Filament Rendering

Studio uses legacy OpenGL rendering by default but there is an option to use
Physically Based Rendering via [Filament](https://google.github.io/filament/Filament.md.html).
To enable Filament you need to `-DMUJOCO_USE_FILAMENT=ON` during the cmake
configuration step. The Filament renderer has multiple rendering backends,
on Linux OpenGL is the default but Vulkan can be used by also providing
the `-DMUJOCO_USE_FILAMENT_VULKAN=ON` option.

Also note that you will need to run the application from the folder containing
the executable so that the expected materials/assets can be found.

## Known Bugs

* MuJoCo Studio does not yet work using Wayland on Linux, use X11 instead.

## Future Work

1. **Stability and Robustness**. We need user feedback to find and fix bugs.

1. **UI/UX improvements**. We have ported the simulate UI to make it easier to
users to switch. We will be making further changes to make use of the
flexibility offered by Dear ImGui ([examples](https://github.com/ocornut/imgui/issues/8942)).

1. **Python integration**. As with simulate, we would like to make Studio usable
  via Python.
