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

#ifndef MUJOCO_PYTHON_RAW_H_
#define MUJOCO_PYTHON_RAW_H_

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjrender.h>
#include <mujoco/mjspec.h>
#include <mujoco/mjvisualize.h>

// Type aliases for MuJoCo C structs to allow us refer to consistently refer
// to them under the "raw" namespace.
namespace mujoco::raw {

using MjContact = ::mjContact;
using MjData = ::mjData;
using MjLROpt = ::mjLROpt;
using MjModel = ::mjModel;
using MjSpec = ::mjSpec;
using MjsElement = ::mjsElement;
using MjsOrientation = ::mjsOrientation;
using MjsPlugin = ::mjsPlugin;
using MjsBody = ::mjsBody;
using MjsFrame = ::mjsFrame;
using MjsJoint = ::mjsJoint;
using MjsGeom = ::mjsGeom;
using MjsSite = ::mjsSite;
using MjsCamera = ::mjsCamera;
using MjsLight = ::mjsLight;
using MjsFlex = ::mjsFlex;
using MjsMesh = ::mjsMesh;
using MjsHField = ::mjsHField;
using MjsSkin = ::mjsSkin;
using MjsTexture = ::mjsTexture;
using MjsMaterial = ::mjsMaterial;
using MjsPair = ::mjsPair;
using MjsExclude = ::mjsExclude;
using MjsEquality = ::mjsEquality;
using MjsTendon = ::mjsTendon;
using MjsWrap = ::mjsWrap;
using MjsActuator = ::mjsActuator;
using MjsSensor = ::mjsSensor;
using MjsNumeric = ::mjsNumeric;
using MjsText = ::mjsText;
using MjsTuple = ::mjsTuple;
using MjsKey = ::mjsKey;
using MjsDefault = ::mjsDefault;
using MjsCompiler = ::mjsCompiler;
using MjOption = ::mjOption;
using MjSolverStat = ::mjSolverStat;
using MjStatistic = ::mjStatistic;
using MjTimerStat = ::mjTimerStat;
using MjVisual = ::mjVisual;
using MjVisualGlobal = decltype(::mjVisual::global);
using MjVisualQuality = decltype(::mjVisual::quality);
using MjVisualHeadlight = decltype(::mjVisual::headlight);
using MjVisualMap = decltype(::mjVisual::map);
using MjVisualScale = decltype(::mjVisual::scale);
using MjVisualRgba = decltype(::mjVisual::rgba);
using MjWarningStat = ::mjWarningStat;

// From mjrender.h
using MjrRect = ::mjrRect;
using MjrContext = ::mjrContext;

// From mjvisualize.h
using MjvPerturb = ::mjvPerturb;
using MjvCamera = ::mjvCamera;
using MjvGLCamera = ::mjvGLCamera;
using MjvGeom = ::mjvGeom;
using MjvLight = ::mjvLight;
using MjvOption = ::mjvOption;
using MjvScene = ::mjvScene;
using MjvFigure = ::mjvFigure;

}  // namespace mujoco::raw

#endif  // MUJOCO_PYTHON_RAW_H_
