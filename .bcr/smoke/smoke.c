// Copyright 2026 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdio.h>

#include <mujoco/mujoco.h>

int main(void) {
  if (mj_version() != mjVERSION_HEADER) {
    return 1;
  }
  if (mjui_themeSpacing(0).total <= 0) {
    return 1;
  }
  const char* xml =
      "<mujoco><worldbody><body><joint name='slide' type='slide'/>"
      "<geom size='.1'/></body></worldbody></mujoco>";
#ifdef CHECK_PLUGIN
  if (!mjp_getPlugin("mujoco.pid", NULL)) {
    return 2;
  }
  xml =
      "<mujoco><extension><plugin plugin='mujoco.pid'><instance name='pid'>"
      "<config key='kp' value='4'/></instance></plugin></extension>"
      "<worldbody><body><joint name='slide' type='slide'/><geom size='.1'/>"
      "</body></worldbody><actuator><plugin joint='slide' plugin='mujoco.pid'"
      " instance='pid'/></actuator></mujoco>";
#endif
  char error[1024] = {0};
  mjSpec* spec = mj_parseXMLString(xml, NULL, error, sizeof(error));
  if (!spec) {
    fprintf(stderr, "%s\n", error);
    return 3;
  }
  mjModel* model = mj_compile(spec, NULL);
  if (!model) {
    fprintf(stderr, "%s\n", mjs_getError(spec));
    mj_deleteSpec(spec);
    return 4;
  }
  mjData* data = mj_makeData(model);
#ifdef CHECK_PLUGIN
  data->ctrl[0] = 1;
#endif
  mj_step(model, data);
  int failed = data->time <= 0 || model->nq != 1;
#ifdef CHECK_PLUGIN
  failed |= data->actuator_force[0] != 4;
#endif
  mj_deleteData(data);
  mj_deleteModel(model);
  mj_deleteSpec(spec);
  return failed;
}
