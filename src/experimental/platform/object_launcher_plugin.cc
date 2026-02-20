// Copyright 2026 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <random>
#include <string>
#include <vector>

#include <imgui.h>
#include <mujoco/mujoco.h>
#include "experimental/platform/imgui_widgets.h"
#include "experimental/platform/plugin.h"

namespace mujoco::studio {

class ObjectLauncher {
 public:
  ObjectLauncher() : rng_(std::random_device{}()) {}

  void UpdateGui() {
    using platform::ImGui_Input;

    ImGui::Checkbox("Enable Key Binding (Ctrl+Shift+Enter)", &enabled_);
    ImGui_Input("Size", &size_, {0.01f, 1.0f, 0.01, 0.1});
    ImGui_Input("Speed", &speed_, {0.01f, 100.0f, 0.1, 1.0});
    ImGui_Input("Mass", &mass_, {0.01f, 100.0f, 0.01, 0.1});
    ImGui_Input("Life", &lifetime_, {0.0f, 60.0f, 0.1, 1.0});

    int shape = type_ == mjGEOM_BOX ? 0 : 1;
    const char* names[] = {"Box", "Sphere"};
    ImGui::Combo("Shape", &shape, names, 2);
    type_ = shape == 0 ? mjGEOM_BOX : mjGEOM_SPHERE;

    if (ImGui::Button("Launch", ImVec2(-1.0f, 0.0f))) {
      active_ = true;
    }

    if (ImGui::Button("Clear")) {
      for (auto& object : objects_) {
        object.expiration = -1;
      }
    }
  }

  void HandleKeyboardEvent() { if (enabled_) active_ = true; }

  bool UpdateSpecPreCompile(mjSpec* spec, const mjModel* model,
                            const mjData* data, const mjvCamera* camera) {
    // Remove expired objects.
    auto it = std::remove_if(
        objects_.begin(), objects_.end(), [&](const ObjectInfo& o) {
          const bool expired =
              o.body_id >= 0 && o.expiration != 0 && o.expiration < data->time;
          if (expired) {
            mjsBody* body = mjs_findBody(spec, o.name.c_str());
            if (body) {
              mjs_delete(spec, body->element);
            }
          }
          return expired;
        });
    if (it != objects_.end()) {
      objects_.erase(it, objects_.end());
      return true;
    }

    if (!active_) return false;
    active_ = false;

    mjsBody* world = mjs_findBody(spec, "world");
    if (!world) return false;
    mjsBody* body = mjs_addBody(world, nullptr);
    if (!body) return false;
    mjsJoint* joint = mjs_addJoint(body, nullptr);
    if (!joint) return false;
    mjsGeom* geom = mjs_addGeom(body, nullptr);
    if (!geom) return false;

    ObjectInfo& object = objects_.emplace_back();
    object.name = "projectile" + std::to_string(counter_++);;
    object.expiration = data->time + lifetime_;

    mjtNum pos[3];
    mjtNum dir[3];
    mjtNum up[3];
    mjv_cameraFrame(pos, dir, up, nullptr, data, camera);
    mjs_setName(body->element, object.name.c_str());

    joint->type = mjJNT_FREE;
    body->mass = mass_;
    geom->type = type_;
    geom->size[0] = size_;
    geom->size[1] = size_;
    geom->size[2] = size_;
    // Slightly in front of the camera.
    body->pos[0] = pos[0] + (dir[0] * 0.1);
    body->pos[1] = pos[1] + (dir[1] * 0.1);
    body->pos[2] = pos[2] + (dir[2] * 0.1);
    // Randomize the color.
    geom->rgba[0] = std::uniform_real_distribution<float>(0.3f, 1.0f)(rng_);
    geom->rgba[1] = std::uniform_real_distribution<float>(0.3f, 1.0f)(rng_);
    geom->rgba[2] = std::uniform_real_distribution<float>(0.3f, 1.0f)(rng_);
    geom->rgba[3] = 1.0;
    // Launch it slightly upwards to get a nice arc.
    launch_vel_[0] = (dir[0] * speed_) + up[0];
    launch_vel_[1] = (dir[1] * speed_) + up[1];
    launch_vel_[2] = (dir[2] * speed_) + up[2];
    return true;
  }

  void UpdateSpecPostCompile(const mjSpec* spec, const mjModel* model,
                             mjData* data) {
    if (objects_.empty()) {
      return;
    }

    ObjectInfo& object = objects_.back();
    if (object.launched) {
      return;
    }
    object.launched = true;

    const int body_id = mj_name2id(model, mjOBJ_BODY, object.name.c_str());
    if (body_id < 0) {
      return;
    }
    int joint_id = model->body_jntadr[body_id];
    if (joint_id < 0 || model->jnt_type[joint_id] != mjJNT_FREE) {
      return;
    }
    int qvel_addr = model->jnt_dofadr[joint_id];
    if (qvel_addr < 0) {
      return;
    }
    object.body_id = body_id;
    data->qvel[qvel_addr + 0] = launch_vel_[0];
    data->qvel[qvel_addr + 1] = launch_vel_[1];
    data->qvel[qvel_addr + 2] = launch_vel_[2];
  }

 private:
  struct ObjectInfo {
    std::string name;
    int body_id = -1;
    mjtNum expiration = 0;
    bool launched = false;
  };

  std::mt19937 rng_;
  int counter_ = 0;
  bool enabled_ = false;
  bool active_ = false;
  mjtNum size_ = 0.13365;
  mjtNum speed_ = 10.0;
  mjtNum mass_ = 10.0;
  mjtNum lifetime_ = 5.0;
  mjtGeom type_ = mjGEOM_BOX;
  mjtNum launch_vel_[3] = {0, 0, 0};
  std::vector<ObjectInfo> objects_;
};

}  // namespace mujoco::studio

mjPLUGIN_LIB_INIT {
  using mujoco::studio::ObjectLauncher;

  static ObjectLauncher plugin;

  mujoco::platform::GuiPlugin gui;
  gui.data = &plugin;
  gui.name = "ObjectLauncher";
  gui.update = [](mujoco::platform::GuiPlugin* self) {
    auto* plugin = static_cast<mujoco::studio::ObjectLauncher*>(self->data);
    plugin->UpdateGui();
  };
  mujoco::platform::RegisterPlugin(gui);

  mujoco::platform::KeyHandlerPlugin key_handler;
  key_handler.data = &plugin;
  key_handler.name = "ObjectLauncher";
  key_handler.key_chord = ImGuiKey_Enter | ImGuiMod_Ctrl | ImGuiMod_Shift;
  key_handler.on_key_pressed = [](mujoco::platform::KeyHandlerPlugin* self) {
    auto* plugin = static_cast<mujoco::studio::ObjectLauncher*>(self->data);
    plugin->HandleKeyboardEvent();
  };
  mujoco::platform::RegisterPlugin(key_handler);

  mujoco::platform::SpecEditorPlugin spec_editor;
  spec_editor.data = &plugin;
  spec_editor.name = "ObjectLauncher";
  spec_editor.pre_compile = [](mujoco::platform::SpecEditorPlugin* self,
                               mjSpec* spec, const mjModel* model,
                               const mjData* data, const mjvCamera* camera) {
    auto* plugin = static_cast<mujoco::studio::ObjectLauncher*>(self->data);
    return plugin->UpdateSpecPreCompile(spec, model, data, camera);
  };
  spec_editor.post_compile = [](mujoco::platform::SpecEditorPlugin* self,
                                const mjSpec* spec, const mjModel* model,
                                mjData* data) {
    auto* plugin = static_cast<mujoco::studio::ObjectLauncher*>(self->data);
    return plugin->UpdateSpecPostCompile(spec, model, data);
  };
  mujoco::platform::RegisterPlugin(spec_editor);
}
