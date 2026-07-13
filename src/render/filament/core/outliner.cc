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

#include "render/filament/core/outliner.h"

#include <cstdint>
#include <memory>

#include <filament/Camera.h>
#include <filament/Material.h>
#include <filament/MaterialInstance.h>
#include <filament/RenderTarget.h>
#include <filament/RenderableManager.h>
#include <filament/Scene.h>
#include <filament/TextureSampler.h>
#include <filament/View.h>
#include <filament/Viewport.h>
#include <math/vec4.h>
#include <utils/EntityManager.h>
#include <mujoco/mjrfilament.h>
#include <mujoco/mujoco.h>
#include "render/filament/core/object_manager.h"
#include "render/filament/core/render_target.h"
#include "render/filament/core/texture.h"

namespace mujoco {

Outliner::Outliner(ObjectManager* object_mgr, uint8_t layer_mask,
                   filament::math::float4 color, float thickness)
    : object_mgr_(object_mgr),
      engine_(object_mgr->GetEngine()),
      layer_mask_(layer_mask),
      color_(color),
      thickness_(thickness) {}

Outliner::~Outliner() { Reset(); }

void Outliner::Prepare(int width, int height) {
  if (width == width_ && height == height_) {
    return;
  }
  Reset();
  width_ = width;
  height_ = height;
  if (width_ <= 0 || height_ <= 0) {
    return;
  }

  auto& em = utils::EntityManager::get();

  // Sets up a pass with a full-screen quad that renders with a given material.
  auto setup_fullscreen = [&](int pass, ObjectManager::MaterialType type) {
    filament::Material* material = object_mgr_->GetMaterial(type);
    filament::VertexBuffer* quad_vb = object_mgr_->GetQuadVertexBuffer();
    filament::IndexBuffer* quad_ib = object_mgr_->GetQuadIndexBuffer();
    const filament::RenderableManager::PrimitiveType primitive_type =
        filament::RenderableManager::PrimitiveType::TRIANGLES;

    material_instances_[pass] = material->createInstance();
    quads_[pass] = em.create();
    filament::RenderableManager::Builder(1)
        .geometry(0, primitive_type, quad_vb, quad_ib)
        .material(0, material_instances_[pass])
        .culling(false)
        .receiveShadows(false)
        .castShadows(false)
        .build(*engine_, quads_[pass]);
    scenes_[pass] = engine_->createScene();
    scenes_[pass]->addEntity(quads_[pass]);
    views_[pass]->setScene(scenes_[pass]);
  };

  // Sets up a pass to use the given render target as the source and/or output.
  auto bind = [&](int pass, int src, int out) {
    // Set the `source` texture as the color texture from a render target.
    if (src >= 0) {
      const filament::TextureSampler sampler(
          filament::TextureSampler::MinFilter::NEAREST,
          filament::TextureSampler::MagFilter::NEAREST);
      material_instances_[pass]->setParameter(
          "source", targets_[src]->GetColorTexture()->GetFilamentTexture(),
          sampler);
    }
    // Set the output of the view to the given render target.
    if (out >= 0) {
      views_[pass]->setRenderTarget(targets_[out]->GetFilamentRenderTarget());
    }
  };

  // Set up two render targets. We will alternate between the two targets to
  // allow for chaining passes together.
  for (int i = 0; i < 2; ++i) {
    mjrfRenderTargetConfig config;
    mjrf_defaultRenderTargetConfig(&config);
    config.color_format = mjPIXEL_FORMAT_RGBA8;
    config.depth_format = mjPIXEL_FORMAT_DEPTH32F;
    config.width = width;
    config.height = height;
    targets_[i] = std::make_unique<RenderTarget>(engine_, config);
    targets_[i]->Prepare(width, height);
  }

  // Setup orthographic camera for full-screen quad rendering.
  camera_ = engine_->createCamera(em.create());
  camera_->setProjection(filament::Camera::Projection::ORTHO, -1.0, 1.0, -1.0,
                         1.0, -1.0, 1.0);
  camera_->lookAt({0, 0, 1}, {0, 0, 0}, {0, 1, 0});

  // Setup all the views.
  for (auto& view : views_) {
    view = engine_->createView();
    view->setCamera(camera_);
    view->setViewport({0, 0, (uint32_t)width, (uint32_t)height});
    view->setPostProcessingEnabled(false);
    view->setShadowingEnabled(false);
    view->setMultiSampleAntiAliasingOptions({.enabled = false});
  }

  // In the first pass, we will render a given scene, but only render the
  // objects marked as outlines. We assume that the objects have already been
  // assigned the kOutlineFlatten material.
  views_[kPassFlatten]->setVisibleLayers(0xff, layer_mask_);

  // All subsequent passes are full-screen post-processing passes.
  setup_fullscreen(kPassJumpFlood1, ObjectManager::kOutlineJumpFlood);
  setup_fullscreen(kPassJumpFlood2, ObjectManager::kOutlineJumpFlood);
  setup_fullscreen(kPassJumpFlood3, ObjectManager::kOutlineJumpFlood);
  setup_fullscreen(kPassJumpFlood4, ObjectManager::kOutlineJumpFlood);
  setup_fullscreen(kPassJumpFlood5, ObjectManager::kOutlineJumpFlood);
  setup_fullscreen(kPassDrawOutline, ObjectManager::kOutlineComposite);

  // Chain the passes together such that the output of a pass is the input to
  // the next pass. The first pass has no input (we are just rendering the
  // selected objects) and the last pass has no output (we are just rendering
  // the outline to the externally provided target).
  bind(kPassFlatten, -1, 0);
  bind(kPassJumpFlood1, 0, 1);
  bind(kPassJumpFlood2, 1, 0);
  bind(kPassJumpFlood3, 0, 1);
  bind(kPassJumpFlood4, 1, 0);
  bind(kPassJumpFlood5, 0, 1);
  bind(kPassDrawOutline, 1, -1);

  // Bind the parameters for each pass. For the jump flood passes, the step
  // parameter determines how far to propagate the outline in each pass.
  material_instances_[kPassJumpFlood1]->setParameter("step", 16.0f);
  material_instances_[kPassJumpFlood2]->setParameter("step", 8.0f);
  material_instances_[kPassJumpFlood3]->setParameter("step", 4.0f);
  material_instances_[kPassJumpFlood4]->setParameter("step", 2.0f);
  material_instances_[kPassJumpFlood5]->setParameter("step", 1.0f);

  // The final pass renders the actual outline onto a render target.
  material_instances_[kPassDrawOutline]->setParameter("color", color_);
  material_instances_[kPassDrawOutline]->setParameter("width", thickness_);

  // Commit all the material instances to the engine.
  for (auto& material_instance : material_instances_) {
    if (material_instance) {
      material_instance->commit(*engine_);
    }
  }
}

void Outliner::Reset() {
  auto& em = utils::EntityManager::get();

  for (auto& scene : scenes_) {
    if (scene) {
      engine_->destroy(scene);
      scene = nullptr;
    }
  }
  for (auto& quad : quads_) {
    if (quad) {
      engine_->destroy(quad);
      em.destroy(quad);
      quad = utils::Entity();
    }
  }
  for (auto& material_instance : material_instances_) {
    if (material_instance) {
      engine_->destroy(material_instance);
      material_instance = nullptr;
    }
  }
  for (auto& view : views_) {
    if (view) {
      engine_->destroy(view);
      view = nullptr;
    }
  }
  if (camera_) {
    utils::Entity entity = camera_->getEntity();
    engine_->destroyCameraComponent(entity);
    em.destroy(entity);
    camera_ = nullptr;
  }
  for (auto& target : targets_) {
    target.reset();
  }
  width_ = 0;
  height_ = 0;
}

void Outliner::Render(filament::Renderer* renderer, filament::View* view,
                      filament::RenderTarget* render_target) {
  filament::Viewport viewport = view->getViewport();
  Prepare(viewport.width, viewport.height);

  for (auto& view : views_) {
    view->setViewport(viewport);
  }

  // Re-render the view's scene to create the flattened selection mask.
  auto prev_clear_opts = renderer->getClearOptions();
  renderer->setClearOptions({.clearColor = {0, 0, 0, 0}, .clear = true});
  views_[kPassFlatten]->setScene(view->getScene());
  views_[kPassFlatten]->setCamera(&view->getCamera());
  renderer->render(views_[kPassFlatten]);
  renderer->setClearOptions(prev_clear_opts);

  // Run the jump flood steps to expand the selection mask
  for (int i = 0; i < kNumJumpFloodPasses; ++i) {
    renderer->render(views_[kPassJumpFlood1 + i]);
  }

  // Render the final pass as an outline onto the provided render target.
  views_[kPassDrawOutline]->setRenderTarget(render_target);
  renderer->render(views_[kPassDrawOutline]);
  views_[kPassDrawOutline]->setRenderTarget(nullptr);
}
}  // namespace mujoco
