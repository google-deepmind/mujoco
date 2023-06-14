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

#ifndef MUJOCO_SRC_XML_XML_API_H_
#define MUJOCO_SRC_XML_XML_API_H_

#include <mujoco/mjexport.h>
#include <mujoco/mjmodel.h>

#ifdef __cplusplus
extern "C" {
#endif



// parse XML file in MJCF or URDF format, compile it, return low-level model
//  if vfs is not NULL, look up files in vfs before reading from disk
//  error can be NULL; otherwise assumed to have size error_sz
MJAPI mjModel* mj_loadXML(const char* filename, const mjVFS* vfs, char* error, int error_sz);

// update XML data structures with info from low-level model, save as MJCF
MJAPI int mj_saveLastXML(const char* filename, const mjModel* m, char* error, int error_sz);

// free last XML model if loaded; called internally at each load
MJAPI void mj_freeLastXML(void);

// print internal XML schema as plain text or HTML, with style-padding or &nbsp;
MJAPI int mj_printSchema(const char* filename, char* buffer, int buffer_sz,
                         int flg_html, int flg_pad);


#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_XML_XML_API_H_
