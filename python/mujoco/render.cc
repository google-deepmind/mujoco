// Copyright 2022 DeepMind Technologies Limited
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

#include <array>
#include <cstdint>

#include <Eigen/Core>
#include <mujoco/mjrender.h>
#include <mujoco/mujoco.h>
#include "errors.h"
#include "function_traits.h"
#include "functions.h"
#include "raw.h"
#include "structs.h"
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace mujoco::python {
namespace _impl {
template <>
class MjWrapper<raw::MjrContext> : public WrapperBase<raw::MjrContext> {
 public:
  MjWrapper();
  MjWrapper(const MjModelWrapper& model, int fontscale);
  MjWrapper(const MjWrapper&) = delete;
  MjWrapper(MjWrapper&&) = default;
  ~MjWrapper() = default;

  void Free();

#define X(var) py_array_or_tuple_t<mjtNum> var
  X(fogRGBA);
  X(auxWidth);
  X(auxHeight);
  X(auxSamples);
  X(auxFBO);
  X(auxFBO_r);
  X(auxColor);
  X(auxColor_r);
  X(textureType);
  X(texture);
  X(skinvertVBO);
  X(skinnormalVBO);
  X(skintexcoordVBO);
  X(skinfaceVBO);
  X(charWidth);
  X(charWidthBig);
#undef X
};

using MjrContextWrapper = MjWrapper<raw::MjrContext>;

template <>
struct enable_if_mj_struct<raw::MjrContext> {
  using type = void;
};

static void MjrContextCapsuleDestructor(PyObject* pyobj) {
  auto* ptr =
      static_cast<raw::MjrContext*>(PyCapsule_GetPointer(pyobj, nullptr));
  mjr_freeContext(ptr);
  delete ptr;
}

#define X(var) var(InitPyArray(ptr_->var, owner_))
#define X_SKIN(var) var(InitPyArray(std::array{ptr_->nskin}, ptr_->var, owner_))
MjrContextWrapper::MjWrapper()
    : WrapperBase([]() {
        raw::MjrContext *const ctx = new raw::MjrContext;
        mjr_defaultContext(ctx);
        return ctx;
      }()),
      X(fogRGBA),
      X(auxWidth),
      X(auxHeight),
      X(auxSamples),
      X(auxFBO),
      X(auxFBO_r),
      X(auxColor),
      X(auxColor_r),
      X(textureType),
      X(texture),
      X_SKIN(skinvertVBO),
      X_SKIN(skinnormalVBO),
      X_SKIN(skintexcoordVBO),
      X_SKIN(skinfaceVBO),
      X(charWidth),
      X(charWidthBig) {}

MjrContextWrapper::MjWrapper(const MjModelWrapper& model, int fontscale)
    : WrapperBase([fontscale](const raw::MjModel* m) {
        raw::MjrContext *const ctx = new raw::MjrContext;
        mjr_defaultContext(ctx);
        InterceptMjErrors(mjr_makeContext)(m, ctx, fontscale);
        return ctx;
      }(model.get()), &MjrContextCapsuleDestructor),
      X(fogRGBA),
      X(auxWidth),
      X(auxHeight),
      X(auxSamples),
      X(auxFBO),
      X(auxFBO_r),
      X(auxColor),
      X(auxColor_r),
      X(textureType),
      X(texture),
      X_SKIN(skinvertVBO),
      X_SKIN(skinnormalVBO),
      X_SKIN(skintexcoordVBO),
      X_SKIN(skinfaceVBO),
      X(charWidth),
      X(charWidthBig) {}
#undef X_SKIN
#undef X

void MjrContextWrapper::Free() {
  // mjr_freeContext is safe to call multiple times.
  InterceptMjErrors(mjr_freeContext)(ptr_);
}

}  // namespace _impl

namespace {
PYBIND11_MODULE(_render, pymodule) {
  namespace py = ::pybind11;
  namespace traits = python_traits;

  using _impl::MjModelWrapper;
  using _impl::MjrContextWrapper;

  // Import the _structs module so that pybind11 knows about Python bindings
  // for MjWrapper types and therefore generates prettier docstrings.
  py::module::import("mujoco._structs");

  py::class_<raw::MjrRect> mjrRect(pymodule, "MjrRect");
  mjrRect.def(py::init([](int left, int bottom, int width, int height) {
                return raw::MjrRect{left, bottom, width, height};
              }),
              py::arg("left"), py::arg("bottom"), py::arg("width"),
              py::arg("height"));
  mjrRect.def("__copy__",
              [](const raw::MjrRect& other) { return raw::MjrRect(other); });
  mjrRect.def("__deepcopy__", [](const raw::MjrRect& other, py::dict) {
    return raw::MjrRect(other);
  });
  DefineStructFunctions(mjrRect);
#define X(var) mjrRect.def_readwrite(#var, &raw::MjrRect::var)
  X(left);
  X(bottom);
  X(width);
  X(height);
#undef X

  py::class_<MjrContextWrapper> mjrContext(pymodule, "MjrContext");
  mjrContext.def(py::init<>());
  mjrContext.def(py::init<const MjModelWrapper&, int>());
  mjrContext.def(
      "free", [](MjrContextWrapper& self) { self.Free(); },
      py::doc("Frees resources in current active OpenGL context, sets struct "
              "to default."));
#define X(var)                                                       \
  mjrContext.def_property(                                           \
      #var, [](const MjrContextWrapper& c) { return c.get()->var; }, \
      [](MjrContextWrapper& c, mjtNum rhs) { c.get()->var = rhs; })
  X(lineWidth);
  X(shadowClip);
  X(shadowScale);
  X(fogStart);
  X(fogEnd);
  X(shadowSize);
  X(offWidth);
  X(offHeight);
  X(offSamples);
  X(fontScale);
  X(offFBO);
  X(offFBO_r);
  X(offColor);
  X(offColor_r);
  X(offDepthStencil);
  X(offDepthStencil_r);
  X(shadowFBO);
  X(shadowTex);
  X(ntexture);
  X(basePlane);
  X(baseMesh);
  X(baseHField);
  X(baseBuiltin);
  X(baseFontNormal);
  X(baseFontShadow);
  X(baseFontBig);
  X(rangePlane);
  X(rangeMesh);
  X(rangeHField);
  X(rangeBuiltin);
  X(rangeFont);
  X(nskin);
  X(charHeight);
  X(charHeightBig);
  X(glInitialized);
  X(windowAvailable);
  X(windowSamples);
  X(windowStereo);
  X(windowDoublebuffer);
  X(currentBuffer);
  X(readPixelFormat);
  X(readDepthMap);
#undef X

#define X(var)                      \
  mjrContext.def_property_readonly( \
      #var, [](const MjrContextWrapper& c) { return c.get()->var; })
  X(nskin);
#undef X

#define X(var) DefinePyArray(mjrContext, #var, &MjrContextWrapper::var)
  X(fogRGBA);
  X(auxWidth);
  X(auxHeight);
  X(auxSamples);
  X(auxFBO);
  X(auxFBO_r);
  X(auxColor);
  X(auxColor_r);
  X(textureType);
  X(texture);
  X(skinvertVBO);
  X(skinnormalVBO);
  X(skintexcoordVBO);
  X(skinfaceVBO);
  X(charWidth);
  X(charWidthBig);
#undef X

  using EigenUnsignedCharVectorX = Eigen::Vector<unsigned char, Eigen::Dynamic>;
  using EigenFloatVectorX = Eigen::Vector<float, Eigen::Dynamic>;

  // Skipped: mjr_defaultContext (have MjrContext.__init__)
  // Skipped: mjr_makeContext (have MjrContext.__init__)
  Def<traits::mjr_changeFont>(pymodule);
  Def<traits::mjr_addAux>(pymodule);
  // Skipped: mjr_freeContext (have MjrContext.__del__)
  Def<traits::mjr_resizeOffscreen>(pymodule);
  Def<traits::mjr_uploadTexture>(pymodule);
  Def<traits::mjr_uploadMesh>(pymodule);
  Def<traits::mjr_uploadHField>(pymodule);
  Def<traits::mjr_restoreBuffer>(pymodule);
  Def<traits::mjr_setBuffer>(pymodule);
  DefWithGil<traits::mjr_readPixels>(
      pymodule, [](std::optional<py::array_t<std::uint8_t>> rgb,
                   std::optional<py::array_t<float>> depth,
                   const raw::MjrRect* viewport, const raw::MjrContext* con) {
        std::uint8_t* const rgb_data =
            rgb.has_value() ? rgb->mutable_data() : nullptr;
        float* const depth_data =
            depth.has_value() ? depth->mutable_data() : nullptr;
        {
          py::gil_scoped_release no_gil;
          return InterceptMjErrors(::mjr_readPixels)(rgb_data, depth_data,
                                                     *viewport, con);
        }
      });
  Def<traits::mjr_drawPixels>(
      pymodule,
      [](std::optional<Eigen::Ref<const EigenUnsignedCharVectorX>> rgb,
         std::optional<Eigen::Ref<const EigenFloatVectorX>> depth,
         const raw::MjrRect* viewport, const raw::MjrContext* con) {
        return InterceptMjErrors(::mjr_drawPixels)(
            rgb.has_value() ? rgb->data() : nullptr,
            depth.has_value() ? depth->data() : nullptr, *viewport, con);
      });
  Def<traits::mjr_blitBuffer>(pymodule);
  Def<traits::mjr_setAux>(pymodule);
  Def<traits::mjr_blitAux>(pymodule);
  Def<traits::mjr_text>(pymodule);
  Def<traits::mjr_overlay>(pymodule);
  Def<traits::mjr_maxViewport>(pymodule);
  Def<traits::mjr_rectangle>(pymodule);
  Def<traits::mjr_label>(pymodule);
  Def<traits::mjr_figure>(pymodule);
  Def<traits::mjr_render>(pymodule);
  Def<traits::mjr_finish>(pymodule);
  Def<traits::mjr_getError>(pymodule);
  Def<traits::mjr_findRect>(pymodule);
}  // PYBIND11_MODULE
}  // namespace
}  // namespace mujoco::python
