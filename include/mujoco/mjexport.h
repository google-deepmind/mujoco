// Copyright 2021 DeepMind Technologies Limited
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

#ifndef MUJOCO_MJEXPORT_H_
#define MUJOCO_MJEXPORT_H_

#if defined _WIN32 || defined __CYGWIN__
  #define MUJOCO_HELPER_DLL_IMPORT __declspec(dllimport)
  #define MUJOCO_HELPER_DLL_EXPORT __declspec(dllexport)
  #define MUJOCO_HELPER_DLL_LOCAL
#else
  #if __GNUC__ >= 4
    #define MUJOCO_HELPER_DLL_IMPORT __attribute__ ((visibility ("default")))
    #define MUJOCO_HELPER_DLL_EXPORT __attribute__ ((visibility ("default")))
    #define MUJOCO_HELPER_DLL_LOCAL  __attribute__ ((visibility ("hidden")))
  #else
    #define MUJOCO_HELPER_DLL_IMPORT
    #define MUJOCO_HELPER_DLL_EXPORT
    #define MUJOCO_HELPER_DLL_LOCAL
  #endif
#endif

#ifdef MJ_STATIC
  // static library
  #define MJAPI
  #define MJLOCAL
#else
  #ifdef MUJOCO_DLL_EXPORTS
    #define MJAPI MUJOCO_HELPER_DLL_EXPORT
  #else
    #define MJAPI MUJOCO_HELPER_DLL_IMPORT
  #endif
  #define MJLOCAL MUJOCO_HELPER_DLL_LOCAL
#endif

#endif  // MUJOCO_MJEXPORT_H_
