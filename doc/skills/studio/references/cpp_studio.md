# C++ Studio & Plugin Architecture

In C++, MuJoCo Studio is structured around `mujoco::studio::App` and launched
via `mujoco::studio::LaunchStudio`.

> [!NOTE]
>
> **Architecture Convergence Note:** Unlike the multi-threaded Python Studio
> API, the current C++ Studio application is single-threaded (its main loop runs
> UI, rendering, and physics sequentially on one thread). The Python and C++
> plugin architectures will converge in future updates around a unified
> message-driven lifecycle design.

## 1. C++ Plugin Registry (`mujoco::studio::RegisterPlugin`)

C++ extensions define plugins using structs declared in
`src/experimental/studio/ux/plugin.h`:

| Plugin Struct | Role | Key Callbacks |
| :--- | :--- | :--- |
| **`GuiPlugin`** | Creates an ImGui window listed in the "Plugins" menu | `update(GuiPlugin* self)` |
| **`ScenePlugin`** | Adds custom `mjvGeom` elements to the `mjvScene` | `enhance_scene(self, model, data, scene)` |
| **`ModelPlugin`** | Hooks into model loading and physics updates | `get_model_to_load`, `post_model_loaded`, `pre_step`, `post_step`, `do_update` |
| **`KeyHandlerPlugin`** | Binds keyboard shortcuts via ImGui key chords | `key_chord`, `on_key_pressed(self)` |
| **`SpecEditorPlugin`** | Edits `mjSpec` with pre/post-recompile triggers | `pre_compile(self, spec, m, d, cam)`, `post_compile` |

## 2. Registering and Launching C++ Plugins

### Pattern A: Automatic Self-Registration (`mjPLUGIN_LIB_INIT`)

Author plugins in self-contained translation units using `mjPLUGIN_LIB_INIT`
(from `#include <mujoco/mjplugin.h>`), which defines a static constructor that
calls `mujoco::studio::RegisterPlugin` on library load before `main()` runs.
Linking the plugin library into the Studio binary registers it without modifying
`main()`:

```cpp
#include "imgui.h"
#include <mujoco/mjplugin.h>
#include "src/experimental/studio/ux/plugin.h"

namespace {

void MyGuiUpdate(mujoco::studio::GuiPlugin* self) {
  ImGui::TextUnformatted("Custom Tool Window");
}

}  // namespace

mjPLUGIN_LIB_INIT(my_tool_plugin) {
  mujoco::studio::GuiPlugin gui;
  gui.name = "My Tool";
  gui.active = true;
  gui.update = MyGuiUpdate;
  mujoco::studio::RegisterPlugin(gui);
}
```

### Pattern B: Explicit Registration in a Custom `main()`

When writing a standalone C++ application or custom launcher entrypoint,
register plugins explicitly before calling `mujoco::studio::LaunchStudio`:

```cpp
#include "absl/flags/parse.h"
#include "imgui.h"
#include "src/experimental/studio/launcher.h"
#include "src/experimental/studio/ux/plugin.h"

void MyGuiUpdate(mujoco::studio::GuiPlugin* self) {
  ImGui::TextUnformatted("Custom Tool Window");
}

int main(int argc, char** argv) {
  absl::ParseCommandLine(argc, argv);

  // Register plugins explicitly before LaunchStudio.
  mujoco::studio::GuiPlugin gui;
  gui.name = "My Tool";
  gui.active = true;
  gui.update = MyGuiUpdate;
  mujoco::studio::RegisterPlugin(gui);

  mujoco::studio::LauncherConfig config;
  config.title = "My Project Studio";
  config.model_file = "path/to/robot.xml";

  return mujoco::studio::LaunchStudio(argc, argv, config);
}
```
