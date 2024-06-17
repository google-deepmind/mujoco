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

#ifndef MUJOCO_SRC_USER_USER_API_H_
#define MUJOCO_SRC_USER_USER_API_H_

#include <math.h>
#include <stddef.h>
#include <mujoco/mjdata.h>
#include <mujoco/mjexport.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjspec.h>
#include <mujoco/mjtnum.h>


// this is a C-API
#ifdef __cplusplus
#include <string>
#include <vector>

extern "C" {
#endif

#define mjNAN NAN                  // used to mark undefined fields

//---------------------------------- Top-level spec manipulation -----------------------------------

// Create spec.
MJAPI mjSpec* mj_makeSpec(void);

// Compile spec to model.
MJAPI mjModel* mj_compile(mjSpec* s, const mjVFS* vfs);

// Recompile spec to model preserving the current state.
MJAPI void mj_recompile(mjSpec* s, const mjVFS* vfs, mjModel* m, mjData* d);

// Copy spec.
MJAPI mjSpec* mj_copySpec(const mjSpec* s);

// Get compiler error message from spec.
MJAPI const char* mjs_getError(mjSpec* s);

// Return 1 if compiler error is a warning.
MJAPI int mjs_isWarning(mjSpec* s);

// Copy model fields back into spec.
MJAPI void mj_copyBack(mjSpec* s, const mjModel* m);

// Delete spec.
MJAPI void mj_deleteSpec(mjSpec* s);


//---------------------------------- Attachment ----------------------------------------------------

// Attach child body to a parent frame, return 0 on success.
MJAPI int mjs_attachBody(mjsFrame* parent, const mjsBody* child,
                         const char* prefix, const char* suffix);

// Attach child frame to a parent body, return 0 on success.
MJAPI int mjs_attachFrame(mjsBody* parent, const mjsFrame* child,
                          const char* prefix, const char* suffix);

// Detach body from mjSpec, remove all references and delete the body, return 0 on success.
MJAPI int mjs_detachBody(mjSpec* s, mjsBody* b);


//---------------------------------- Add tree elements ---------------------------------------------

// Add child body to body, return child.
MJAPI mjsBody* mjs_addBody(mjsBody* body, mjsDefault* def);

// Add site to body, return site spec.
MJAPI mjsSite* mjs_addSite(mjsBody* body, mjsDefault* def);

// Add joint to body.
MJAPI mjsJoint* mjs_addJoint(mjsBody* body, mjsDefault* def);

// Add freejoint to body.
MJAPI mjsJoint* mjs_addFreeJoint(mjsBody* body);

// Add geom to body.
MJAPI mjsGeom* mjs_addGeom(mjsBody* body, mjsDefault* def);

// Add camera to body.
MJAPI mjsCamera* mjs_addCamera(mjsBody* body, mjsDefault* def);

// Add light to body.
MJAPI mjsLight* mjs_addLight(mjsBody* body, mjsDefault* def);

// Add frame to body.
MJAPI mjsFrame* mjs_addFrame(mjsBody* body, mjsFrame* parentframe);

// Delete object corresponding to the given element.
MJAPI void mjs_delete(mjsElement* element);


//---------------------------------- Add non-tree elements -----------------------------------------

// Add actuator.
MJAPI mjsActuator* mjs_addActuator(mjSpec* s, mjsDefault* def);

// Add sensor.
MJAPI mjsSensor* mjs_addSensor(mjSpec* s);

// Add flex.
MJAPI mjsFlex* mjs_addFlex(mjSpec* s);

// Add contact pair.
MJAPI mjsPair* mjs_addPair(mjSpec* s, mjsDefault* def);

// Add excluded body pair.
MJAPI mjsExclude* mjs_addExclude(mjSpec* s);

// Add equality.
MJAPI mjsEquality* mjs_addEquality(mjSpec* s, mjsDefault* def);

// Add tendon.
MJAPI mjsTendon* mjs_addTendon(mjSpec* s, mjsDefault* def);

// Wrap site using tendon.
MJAPI mjsWrap* mjs_wrapSite(mjsTendon* tendon, const char* name);

// Wrap geom using tendon.
MJAPI mjsWrap* mjs_wrapGeom(mjsTendon* tendon, const char* name, const char* sidesite);

// Wrap joint using tendon.
MJAPI mjsWrap* mjs_wrapJoint(mjsTendon* tendon, const char* name, double coef);

// Wrap pulley using tendon.
MJAPI mjsWrap* mjs_wrapPulley(mjsTendon* tendon, double divisor);

// Add numeric.
MJAPI mjsNumeric* mjs_addNumeric(mjSpec* s);

// Add text.
MJAPI mjsText* mjs_addText(mjSpec* s);

// Add tuple.
MJAPI mjsTuple* mjs_addTuple(mjSpec* s);

// Add keyframe.
MJAPI mjsKey* mjs_addKey(mjSpec* s);

// Add plugin.
MJAPI mjsPlugin* mjs_addPlugin(mjSpec* s);

// Add default.
MJAPI mjsDefault* mjs_addDefault(mjSpec* s, const char* classname, int parentid, int* id);


//---------------------------------- Add assets ----------------------------------------------------

// Add mesh.
MJAPI mjsMesh* mjs_addMesh(mjSpec* s, mjsDefault* def);

// Add height field.
MJAPI mjsHField* mjs_addHField(mjSpec* s);

// Add skin.
MJAPI mjsSkin* mjs_addSkin(mjSpec* s);

// Add texture.
MJAPI mjsTexture* mjs_addTexture(mjSpec* s);

// Add material.
MJAPI mjsMaterial* mjs_addMaterial(mjSpec* s, mjsDefault* def);


//---------------------------------- Find/get utilities --------------------------------------------

// Get spec from body.
MJAPI mjSpec* mjs_getSpec(mjsBody* body);

// Find body in model by name.
MJAPI mjsBody* mjs_findBody(mjSpec* s, const char* name);

// Find child body by name.
MJAPI mjsBody* mjs_findChild(mjsBody* body, const char* name);

// Find mesh by name.
MJAPI mjsMesh* mjs_findMesh(mjSpec* s, const char* name);

// Find frame by name.
MJAPI mjsFrame* mjs_findFrame(mjSpec* s, const char* name);

// Get default corresponding to an element.
MJAPI mjsDefault* mjs_getDefault(mjsElement* element);

// Find default in model by class name.
MJAPI mjsDefault* mjs_findDefault(mjSpec* s, const char* classname);

// Get global default from model.
MJAPI mjsDefault* mjs_getSpecDefault(mjSpec* s);

// Get element id.
MJAPI int mjs_getId(mjsElement* element);


//---------------------------------- Tree traversal ------------------------------------------------

// Return body's first child of given type.
MJAPI mjsElement* mjs_firstChild(mjsBody* body, mjtObj type);

// Return body's next child of the same type; return NULL if child is last.
MJAPI mjsElement* mjs_nextChild(mjsBody* body, mjsElement* child);


//---------------------------------- Attribute setters ---------------------------------------------

// Copy text to string.
MJAPI void mjs_setString(mjString* dest, const char* text);

// Split text to entries and copy to string vector.
MJAPI void mjs_setStringVec(mjStringVec* dest, const char* text);

// Set entry in string vector.
MJAPI mjtByte mjs_setInStringVec(mjStringVec* dest, int i, const char* text);

// Append text entry to string vector.
MJAPI void mjs_appendString(mjStringVec* dest, const char* text);

// Copy int array to vector.
MJAPI void mjs_setInt(mjIntVec* dest, const int* array, int size);

// Append int array to vector of arrays.
MJAPI void mjs_appendIntVec(mjIntVecVec* dest, const int* array, int size);

// Copy float array to vector.
MJAPI void mjs_setFloat(mjFloatVec* dest, const float* array, int size);

// Append float array to vector of arrays.
MJAPI void mjs_appendFloatVec(mjFloatVecVec* dest, const float* array, int size);

// Copy double array to vector.
MJAPI void mjs_setDouble(mjDoubleVec* dest, const double* array, int size);

// Set plugin attributes.
MJAPI void mjs_setPluginAttributes(mjsPlugin* plugin, void* attributes);


//---------------------------------- Attribute getters ---------------------------------------------

// Get string contents.
MJAPI const char* mjs_getString(const mjString* source);

// Get double array contents and optionally its size.
MJAPI const double* mjs_getDouble(const mjDoubleVec* source, int* size);


//---------------------------------- Other utilities -----------------------------------------------

// Set active plugins.
MJAPI void mjs_setActivePlugins(mjSpec* s, void* activeplugins);

// Set element's default.
MJAPI void mjs_setDefault(mjsElement* element, mjsDefault* def);

// Set element's enlcosing frame.
MJAPI void mjs_setFrame(mjsElement* dest, mjsFrame* frame);

// Resolve alternative orientations to quat, return error if any.
MJAPI const char* mjs_resolveOrientation(double quat[4], mjtByte degree, const char* sequence,
                                         const mjsOrientation* orientation);

// Compute quat and diag inertia from full inertia matrix, return error if any.
MJAPI const char* mjs_fullInertia(double quat[4], double inertia[3], const double fullinertia[6]);


//---------------------------------- Initialization  -----------------------------------------------

// Default spec attributes.
MJAPI void mjs_defaultSpec(mjSpec* spec);

// Default orientation attributes.
MJAPI void mjs_defaultOrientation(mjsOrientation* orient);

// Default body attributes.
MJAPI void mjs_defaultBody(mjsBody* body);

// Default frame attributes.
MJAPI void mjs_defaultFrame(mjsFrame* frame);

// Default joint attributes.
MJAPI void mjs_defaultJoint(mjsJoint* joint);

// Default geom attributes.
MJAPI void mjs_defaultGeom(mjsGeom* geom);

// Default site attributes.
MJAPI void mjs_defaultSite(mjsSite* site);

// Default camera attributes.
MJAPI void mjs_defaultCamera(mjsCamera* camera);

// Default light attributes.
MJAPI void mjs_defaultLight(mjsLight* light);

// Default flex attributes.
MJAPI void mjs_defaultFlex(mjsFlex* flex);

// Default mesh attributes.
MJAPI void mjs_defaultMesh(mjsMesh* mesh);

// Default height field attributes.
MJAPI void mjs_defaultHField(mjsHField* hfield);

// Default skin attributes.
MJAPI void mjs_defaultSkin(mjsSkin* skin);

// Default texture attributes.
MJAPI void mjs_defaultTexture(mjsTexture* texture);

// Default material attributes.
MJAPI void mjs_defaultMaterial(mjsMaterial* material);

// Default pair attributes.
MJAPI void mjs_defaultPair(mjsPair* pair);

// Default equality attributes.
MJAPI void mjs_defaultEquality(mjsEquality* equality);

// Default tendon attributes.
MJAPI void mjs_defaultTendon(mjsTendon* tendon);

// Default actuator attributes.
MJAPI void mjs_defaultActuator(mjsActuator* actuator);

// Default sensor attributes.
MJAPI void mjs_defaultSensor(mjsSensor* sensor);

// Default numeric attributes.
MJAPI void mjs_defaultNumeric(mjsNumeric* numeric);

// Default text attributes.
MJAPI void mjs_defaultText(mjsText* text);

// Default tuple attributes.
MJAPI void mjs_defaultTuple(mjsTuple* tuple);

// Default keyframe attributes.
MJAPI void mjs_defaultKey(mjsKey* key);

// Default plugin attributes.
MJAPI void mjs_defaultPlugin(mjsPlugin* plugin);


//---------------------------------- Compiler cache ------------------------------------------------

typedef struct mjCache_* mjCache;

// Set the size of the cache in bytes.
MJAPI void mj_setCacheSize(mjCache cache, size_t size);

// Get internal global cache context.
MJAPI mjCache mj_globalCache(void);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // MUJOCO_SRC_USER_USER_API_H_
