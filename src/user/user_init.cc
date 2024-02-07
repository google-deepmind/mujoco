// Copyright 2024 DeepMind Technologies Limited
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

#include <cstring>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include "user/user_api.h"
#include "user/user_util.h"


// Default body parameters
void mjm_defaultBody(mjmBody& body) {
  memset(&body, 0, sizeof(mjmBody));

  // set non-zero defaults
  body.fullinertia[0] = mjNAN;
  body.explicitinertial = (mjtByte)false;
  body.mocap = (mjtByte)false;
  body.quat[0] = 1;
  body.iquat[0] = 1;
  body.pos[0] = mjNAN;
  body.ipos[0] = mjNAN;
  body.alt.axisangle[0] = body.alt.xyaxes[0] = body.alt.zaxis[0] = body.alt.euler[0] = mjNAN;
  body.ialt.axisangle[0] = body.ialt.xyaxes[0] = body.ialt.zaxis[0] = body.ialt.euler[0] = mjNAN;
  body.plugin.active = false;
  body.plugin.instance = nullptr;
}



// Default site parameters
void mjm_defaultSite(mjmSite& site) {
  memset(&site, 0, sizeof(mjmSite));

  // set non-zero defaults
  site.type = mjGEOM_SPHERE;
  site.quat[0] = 1;
  site.size[0] = site.size[1] = site.size[2] = 0.005;
  site.fromto[0] = mjNAN;
  site.alt.axisangle[0] = site.alt.xyaxes[0] = site.alt.zaxis[0] = site.alt.euler[0] = mjNAN;
  site.rgba[0] = site.rgba[1] = site.rgba[2] = 0.5f;
  site.rgba[3] = 1.0f;
}
