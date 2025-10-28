// Copyright 2025 DeepMind Technologies Limited
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

#include "experimental/toolbox/helpers.h"

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <ios>
#include <iterator>
#include <string>
#include <vector>

#include "webp/encode.h"
#include "webp/types.h"
#include <mujoco/mjrender.h>
#include <mujoco/mjxmacro.h>
#include <mujoco/mujoco.h>
#include "engine/engine_vis_visualize.h"
#include "xml/xml_api.h"

namespace mujoco::toolbox {

mjModel* LoadMujocoModel(const std::string& model_file, const mjVFS* vfs) {
  mjModel* model = nullptr;

  if (model_file.empty()) {
    auto spec = mj_makeSpec();
    model = mj_compile(spec, 0);
    mj_deleteSpec(spec);
  } else if (model_file.ends_with(".mjb")) {
    model = mj_loadModel(model_file.c_str(), 0);
    if (!model) {
      mju_error("LoadMujocoModel mj_loadModel could not load file '%s'",
                model_file.c_str());
    }
  } else if (model_file.ends_with(".xml")) {
    char error[1000] = "";
    model = mj_loadXML(model_file.c_str(), vfs, error, sizeof(error));
    if (!model) {
      mju_error("LoadMujocoModel mj_loadXML failed with '%s' for file '%s'",
                error, model_file.c_str());
    }
  } else {
    char error[1000] = "";
    auto spec =
        mj_parseXMLString(model_file.c_str(), nullptr, error, sizeof(error));
    if (!spec) {
      mju_error(
          "LoadMujocoModel mj_parseXMLString failed with '%s' for file '%s'",
          error, model_file.c_str());
    }
    model = mj_compile(spec, 0);
    mj_deleteSpec(spec);
  }
  return model;
}

void SaveText(const std::string& contents, const std::string& filename) {
  std::ofstream file(filename);
  file.write(contents.data(), contents.size());
  file.close();
}

std::string LoadText(const std::string& filename) {
  std::ifstream file(filename);
  std::string contents((std::istreambuf_iterator<char>(file)),
                       std::istreambuf_iterator<char>());
  file.close();
  return contents;
}

void SaveColorToWebp(int width, int height, const unsigned char* data,
                     const std::string& filename) {
  uint8_t* webp = nullptr;
  const size_t size =
      WebPEncodeLosslessRGB(data, width, height, width * 3, &webp);

  std::ofstream file(filename, std::ios::binary);
  file.write(reinterpret_cast<const char*>(webp), size);
  file.close();
  WebPFree(webp);
}

void SaveDepthToWebp(int width, int height, const float* data,
                     const std::string& filename) {
  const int size = width * height;

  // Turn the depth buffer into a greyscale color buffer.
  std::vector<unsigned char> byte_buffer;
  byte_buffer.reserve(size * 3);
  for (int i = 0; i < size; ++i) {
    auto byte = static_cast<int>(255.0 * data[i]);
    byte_buffer.push_back(byte);
    byte_buffer.push_back(byte);
    byte_buffer.push_back(byte);
  }
  SaveColorToWebp(width, height, byte_buffer.data(), filename);
}

void SaveScreenshotToWebp(int width, int height, mjrContext* con,
                          const std::string& filename) {
  mjr_setBuffer(mjFB_OFFSCREEN, con);
  auto rgb_buffer = std::vector<unsigned char>(3 * width * height);
  auto depth_buffer = std::vector<float>(width * height, 1.0f);
  mjrRect viewport = {0, 0, width, height};
  mjr_readPixels(rgb_buffer.data(), depth_buffer.data(), viewport, con);
  mjr_setBuffer(mjFB_WINDOW, con);
  SaveColorToWebp(width, height, rgb_buffer.data(), filename);
}

const void* GetValue(const mjModel* model, const mjData* data,
                     const char* field, int index) {
  MJDATA_POINTERS_PREAMBLE(model);
#define X(TYPE, NAME, NR, NC)                                        \
  if (!std::strcmp(#NAME, field) && !std::strcmp(#TYPE, "mjtNum")) { \
    if (index >= 0 && index < model->NR * NC) {                      \
      return &data->NAME[index];                                     \
    } else {                                                         \
      return nullptr;                                                \
    }                                                                \
  }
  MJDATA_POINTERS
#undef X
  return nullptr;  // Invalid field.
}

std::string CameraToString(const mjData* data, const mjvCamera* camera) {
  mjtNum pos[3], forward[3], up[3], right[3];
  mjv_cameraFrame(pos, forward, up, right, data, camera);
  char str[500];
  std::snprintf(str, sizeof(str),
                "<camera pos=\"%.3f %.3f %.3f\" xyaxes=\"%.3f %.3f %.3f %.3f "
                "%.3f %.3f\"/>\n",
                pos[0], pos[1], pos[2], right[0], right[1], right[2],
                up[0], up[1], up[2]);
  return str;
}

std::string KeyframeToString(const mjModel* model, const mjData* data,
                             bool full_precision) {
  const int kStrLen = 5000;

  char buf[200];
  const char p_regular[] = "%g";
  const char p_full[] = "%-22.16g";
  const char* format = full_precision ? p_full : p_regular;

  char str[kStrLen] = "<key\n";

  // time
  std::strncat(str, " time=\"", kStrLen);
  std::snprintf(buf, sizeof(buf), format, data->time);
  std::strncat(str, buf, kStrLen);

  // qpos
  std::strncat(str, "\"\n  qpos=\"", kStrLen);
  for (int i = 0; i < model->nq; i++) {
    std::snprintf(buf, sizeof(buf), format, data->qpos[i]);
    if (i < model->nq - 1) std::strncat(buf, " ", 200);
    std::strncat(str, buf, kStrLen);
  }

  // qvel
  std::strncat(str, "\"\n  qvel=\"", kStrLen);
  for (int i = 0; i < model->nv; i++) {
    std::snprintf(buf, sizeof(buf), format, data->qvel[i]);
    if (i < model->nv - 1) std::strncat(buf, " ", 200);
    std::strncat(str, buf, kStrLen);
  }

  // act
  if (model->na > 0) {
    std::strncat(str, "\"\n  act=\"", kStrLen);
    for (int i = 0; i < model->na; i++) {
      std::snprintf(buf, sizeof(buf), format, data->act[i]);
      if (i < model->na - 1) std::strncat(buf, " ", 200);
      std::strncat(str, buf, kStrLen);
    }
  }

  // ctrl
  if (model->nu > 0) {
    std::strncat(str, "\"\n  ctrl=\"", kStrLen);
    for (int i = 0; i < model->nu; i++) {
      std::snprintf(buf, sizeof(buf), format, data->ctrl[i]);
      if (i < model->nu - 1) std::strncat(buf, " ", 200);
      std::strncat(str, buf, kStrLen);
    }
  }

  if (model->nmocap > 0) {
    std::strncat(str, "\"\n  mpos=\"", kStrLen);
    for (int i = 0; i < 3 * model->nmocap; i++) {
      std::snprintf(buf, sizeof(buf), format, data->mocap_pos[i]);
      if (i < 3 * model->nmocap - 1) std::strncat(buf, " ", 200);
      std::strncat(str, buf, kStrLen);
    }

    // mocap_quat
    std::strncat(str, "\"\n  mquat=\"", kStrLen);
    for (int i = 0; i < 4 * model->nmocap; i++) {
      std::snprintf(buf, sizeof(buf), format, data->mocap_quat[i]);
      if (i < 4 * model->nmocap - 1) std::strncat(buf, " ", 200);
      std::strncat(str, buf, kStrLen);
    }
  }

  std::strncat(str, "\"\n/>", kStrLen);
  return str;
}

}  // namespace mujoco::toolbox
