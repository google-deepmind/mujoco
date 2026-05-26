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

#ifndef MUJOCO_MJASSERT_H_
#define MUJOCO_MJASSERT_H_

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjplugin.h>
#include <mujoco/mjrender.h>
#include <mujoco/mjspec.h>
#include <mujoco/mjthread.h>
#include <mujoco/mjtype.h>
#include <mujoco/mjui.h>
#include <mujoco/mjvisualize.h>

#if defined(__cplusplus)
  #define MJ_ASSERT_SIZE(type, size) \
    static_assert(sizeof(type) == (size), #type " must be " #size " bytes for MuJoCo ABI stability")
#elif defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
  #define MJ_ASSERT_SIZE(type, size) \
    _Static_assert(sizeof(type) == (size), #type " must be " #size " bytes for MuJoCo ABI stability")
#else
  #define MJ_ASSERT_SIZE(type, size) \
    typedef char mj_assert_##type[sizeof(type) == (size) ? 1 : -1]
#endif

// primitive types
#if !defined(mjUSESINGLE)
MJ_ASSERT_SIZE(mjtNum, 8);
#else
MJ_ASSERT_SIZE(mjtNum, 4);
#endif
MJ_ASSERT_SIZE(mjtByte, 1);
MJ_ASSERT_SIZE(mjtBool, 1);
MJ_ASSERT_SIZE(mjtSize, 8);

// mjModel enums
MJ_ASSERT_SIZE(mjtDisableBit, 4);
MJ_ASSERT_SIZE(mjtEnableBit, 4);
MJ_ASSERT_SIZE(mjtJoint, 4);
MJ_ASSERT_SIZE(mjtGeom, 4);
MJ_ASSERT_SIZE(mjtProjection, 4);
MJ_ASSERT_SIZE(mjtCamLight, 4);
MJ_ASSERT_SIZE(mjtLightType, 4);
MJ_ASSERT_SIZE(mjtTexture, 4);
MJ_ASSERT_SIZE(mjtTextureRole, 4);
MJ_ASSERT_SIZE(mjtColorSpace, 4);
MJ_ASSERT_SIZE(mjtIntegrator, 4);
MJ_ASSERT_SIZE(mjtCone, 4);
MJ_ASSERT_SIZE(mjtJacobian, 4);
MJ_ASSERT_SIZE(mjtSolver, 4);
MJ_ASSERT_SIZE(mjtEq, 4);
MJ_ASSERT_SIZE(mjtWrap, 4);
MJ_ASSERT_SIZE(mjtTrn, 4);
MJ_ASSERT_SIZE(mjtDyn, 4);
MJ_ASSERT_SIZE(mjtGain, 4);
MJ_ASSERT_SIZE(mjtBias, 4);
MJ_ASSERT_SIZE(mjtObj, 4);
MJ_ASSERT_SIZE(mjtSensor, 4);
MJ_ASSERT_SIZE(mjtStage, 4);
MJ_ASSERT_SIZE(mjtDataType, 4);
MJ_ASSERT_SIZE(mjtConDataField, 4);
MJ_ASSERT_SIZE(mjtRayDataField, 4);
MJ_ASSERT_SIZE(mjtCamOutBit, 4);
MJ_ASSERT_SIZE(mjtSameFrame, 4);
MJ_ASSERT_SIZE(mjtSleepPolicy, 4);
MJ_ASSERT_SIZE(mjtLRMode, 4);
MJ_ASSERT_SIZE(mjtFlexSelf, 4);
MJ_ASSERT_SIZE(mjtSDFType, 4);

// mjData enums
MJ_ASSERT_SIZE(mjtState, 4);
MJ_ASSERT_SIZE(mjtConstraint, 4);
MJ_ASSERT_SIZE(mjtConstraintState, 4);
MJ_ASSERT_SIZE(mjtWarning, 4);
MJ_ASSERT_SIZE(mjtTimer, 4);
MJ_ASSERT_SIZE(mjtSleepState, 4);

// mjspec.h
MJ_ASSERT_SIZE(mjtGeomInertia, 4);
MJ_ASSERT_SIZE(mjtMeshInertia, 4);
MJ_ASSERT_SIZE(mjtMeshBuiltin, 4);
MJ_ASSERT_SIZE(mjtBuiltin, 4);
MJ_ASSERT_SIZE(mjtMark, 4);
MJ_ASSERT_SIZE(mjtLimited, 4);
MJ_ASSERT_SIZE(mjtAlignFree, 4);
MJ_ASSERT_SIZE(mjtInertiaFromGeom, 4);
MJ_ASSERT_SIZE(mjtOrientation, 4);

// mjvisualize.h
MJ_ASSERT_SIZE(mjtCatBit, 4);
MJ_ASSERT_SIZE(mjtMouse, 4);
MJ_ASSERT_SIZE(mjtPertBit, 4);
MJ_ASSERT_SIZE(mjtCamera, 4);
MJ_ASSERT_SIZE(mjtLabel, 4);
MJ_ASSERT_SIZE(mjtFrame, 4);
MJ_ASSERT_SIZE(mjtVisFlag, 4);
MJ_ASSERT_SIZE(mjtRndFlag, 4);
MJ_ASSERT_SIZE(mjtStereo, 4);

// mjrender.h
MJ_ASSERT_SIZE(mjtGridPos, 4);
MJ_ASSERT_SIZE(mjtFramebuffer, 4);
MJ_ASSERT_SIZE(mjtDepthMap, 4);
MJ_ASSERT_SIZE(mjtFontScale, 4);
MJ_ASSERT_SIZE(mjtFont, 4);

// mjui.h
MJ_ASSERT_SIZE(mjtButton, 4);
MJ_ASSERT_SIZE(mjtEvent, 4);
MJ_ASSERT_SIZE(mjtItem, 4);
MJ_ASSERT_SIZE(mjtSection, 4);

// mjthread.h
MJ_ASSERT_SIZE(mjtTaskStatus, 4);

// mjplugin.h
MJ_ASSERT_SIZE(mjtPluginCapabilityBit, 4);

#undef MJ_ASSERT_SIZE

#endif  // MUJOCO_MJASSERT_H_
