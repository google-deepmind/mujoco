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

// Recompile spec to model, preserving the state, return 0 on success.
MJAPI int mj_recompile(mjSpec* s, const mjVFS* vfs, mjModel* m, mjData* d);

// Copy spec.
MJAPI mjSpec* mj_copySpec(const mjSpec* s);

// Get compiler error message from spec.
MJAPI const char* mjs_getError(mjSpec* s);

// Return 1 if compiler error is a warning.
MJAPI int mjs_isWarning(mjSpec* s);

// Delete spec.
MJAPI void mj_deleteSpec(mjSpec* s);

// Add spec (model asset) to spec.
MJAPI void mjs_addSpec(mjSpec* s, mjSpec* child);

// Activate plugin, return 0 on success.
MJAPI int mjs_activatePlugin(mjSpec* s, const char* name);

// Turn deep copy on or off attach. Returns 0 on success.
MJAPI int mjs_setDeepCopy(mjSpec* s, int deepcopy);

// Copy real-valued arrays from model to spec, returns 1 on success.
MJAPI int mj_copyBack(mjSpec* s, const mjModel* m);


//---------------------------------- Attachment ----------------------------------------------------

// Attach child to a parent, return the attached element if success or NULL otherwise.
MJAPI mjsElement* mjs_attach(mjsElement* parent, const mjsElement* child,
                             const char* prefix, const char* suffix);

// Detach body from mjSpec, remove all references and delete the body, return 0 on success.
MJAPI int mjs_detachBody(mjSpec* s, mjsBody* b);

// Detach default from mjSpec, remove all references and delete the default, return 0 on success.
MJAPI int mjs_detachDefault(mjSpec* s, mjsDefault* d);

//---------------------------------- Add tree elements ---------------------------------------------

// Add child body to body, return child.
MJAPI mjsBody* mjs_addBody(mjsBody* body, const mjsDefault* def);

// Add site to body, return site spec.
MJAPI mjsSite* mjs_addSite(mjsBody* body, const mjsDefault* def);

// Add joint to body.
MJAPI mjsJoint* mjs_addJoint(mjsBody* body, const mjsDefault* def);

// Add freejoint to body.
MJAPI mjsJoint* mjs_addFreeJoint(mjsBody* body);

// Add geom to body.
MJAPI mjsGeom* mjs_addGeom(mjsBody* body, const mjsDefault* def);

// Add camera to body.
MJAPI mjsCamera* mjs_addCamera(mjsBody* body, const mjsDefault* def);

// Add light to body.
MJAPI mjsLight* mjs_addLight(mjsBody* body, const mjsDefault* def);

// Add frame to body.
MJAPI mjsFrame* mjs_addFrame(mjsBody* body, mjsFrame* parentframe);

// Delete object corresponding to the given element, return 0 on success.
MJAPI int mjs_delete(mjsElement* element);


//---------------------------------- Add non-tree elements -----------------------------------------

// Add actuator.
MJAPI mjsActuator* mjs_addActuator(mjSpec* s, const mjsDefault* def);

// Add sensor.
MJAPI mjsSensor* mjs_addSensor(mjSpec* s);

// Add flex.
MJAPI mjsFlex* mjs_addFlex(mjSpec* s);

// Add contact pair.
MJAPI mjsPair* mjs_addPair(mjSpec* s, const mjsDefault* def);

// Add excluded body pair.
MJAPI mjsExclude* mjs_addExclude(mjSpec* s);

// Add equality.
MJAPI mjsEquality* mjs_addEquality(mjSpec* s, const mjsDefault* def);

// Add tendon.
MJAPI mjsTendon* mjs_addTendon(mjSpec* s, const mjsDefault* def);

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
MJAPI mjsDefault* mjs_addDefault(mjSpec* s, const char* classname, const mjsDefault* parent);


//---------------------------------- Set actuator parameters ---------------------------------------

// Set actuator to motor, return error on failure.
MJAPI const char* mjs_setToMotor(mjsActuator* actuator);

// Set actuator to position, return error on failure.
MJAPI const char* mjs_setToPosition(mjsActuator* actuator, double kp, double kv[1],
                                    double dampratio[1], double timeconst[1], double inheritrange);

// Set actuator to integrated velocity, return error on failure.
MJAPI const char* mjs_setToIntVelocity(mjsActuator* actuator, double kp, double kv[1],
                                       double dampratio[1], double timeconst[1], double inheritrange);

// Set actuator to velocity, return error on failure.
MJAPI const char* mjs_setToVelocity(mjsActuator* actuator, double kv);

// Set actuator to damper, return error on failure.
MJAPI const char* mjs_setToDamper(mjsActuator* actuator, double kv);

// Set actuator to cylinder actuator, return error on failure.
MJAPI const char* mjs_setToCylinder(mjsActuator* actuator, double timeconst,
                                    double bias, double area, double diameter);

// Set actuator to muscle, return error on failure.
MJAPI const char* mjs_setToMuscle(mjsActuator* actuator, double timeconst[2], double tausmooth,
                                  double range[2], double force, double scale, double lmin,
                                  double lmax, double vmax, double fpmax, double fvmax);

// Set actuator to adhesion, return error on failure.
MJAPI const char* mjs_setToAdhesion(mjsActuator* actuator, double gain);

//---------------------------------- Add assets ----------------------------------------------------

// Add mesh.
MJAPI mjsMesh* mjs_addMesh(mjSpec* s, const mjsDefault* def);

// Add height field.
MJAPI mjsHField* mjs_addHField(mjSpec* s);

// Add skin.
MJAPI mjsSkin* mjs_addSkin(mjSpec* s);

// Add texture.
MJAPI mjsTexture* mjs_addTexture(mjSpec* s);

// Add material.
MJAPI mjsMaterial* mjs_addMaterial(mjSpec* s, const mjsDefault* def);


//---------------------------------- Find/get utilities --------------------------------------------

// Get spec from body.
MJAPI mjSpec* mjs_getSpec(mjsElement* element);

// Find spec (model asset) by name.
MJAPI mjSpec* mjs_findSpec(mjSpec* spec, const char* name);

// Find body in spec by name.
MJAPI mjsBody* mjs_findBody(mjSpec* s, const char* name);

// Find element in spec by name.
MJAPI mjsElement* mjs_findElement(mjSpec* s, mjtObj type, const char* name);

// Find child body by name.
MJAPI mjsBody* mjs_findChild(mjsBody* body, const char* name);

// Get parent body.
MJAPI mjsBody* mjs_getParent(mjsElement* element);

// Get parent frame.
MJAPI mjsFrame* mjs_getFrame(mjsElement* element);

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

// Return body's first child of given type. If recurse is nonzero, also search the body's subtree.
MJAPI mjsElement* mjs_firstChild(mjsBody* body, mjtObj type, int recurse);

// Return body's next child of the same type; return NULL if child is last.
// If recurse is nonzero, also search the body's subtree.
MJAPI mjsElement* mjs_nextChild(mjsBody* body, mjsElement* child, int recurse);

// Return spec's first element of selected type.
MJAPI mjsElement* mjs_firstElement(mjSpec* s, mjtObj type);

// Return spec's next element; return NULL if element is last.
MJAPI mjsElement* mjs_nextElement(mjSpec* s, mjsElement* element);

// Safely cast an element as mjsBody, or return NULL if the element is not an mjsBody.
MJAPI mjsBody* mjs_asBody(mjsElement* element);

// Safely cast an element as mjsGeom, or return NULL if the element is not an mjsGeom.
MJAPI mjsGeom* mjs_asGeom(mjsElement* element);

// Safely cast an element as mjsJoint, or return NULL if the element is not an mjsJoint.
MJAPI mjsJoint* mjs_asJoint(mjsElement* element);

// Safely cast an element as mjsSite, or return NULL if the element is not an mjsSite.
MJAPI mjsSite* mjs_asSite(mjsElement* element);

// Safely cast an element as mjsCamera, or return NULL if the element is not an mjsCamera.
MJAPI mjsCamera* mjs_asCamera(mjsElement* element);

// Safely cast an element as mjsLight, or return NULL if the element is not an mjsLight.
MJAPI mjsLight* mjs_asLight(mjsElement* element);

// Safely cast an element as mjsFrame, or return NULL if the element is not an mjsFrame.
MJAPI mjsFrame* mjs_asFrame(mjsElement* element);

// Safely cast an element as mjsActuator, or return NULL if the element is not an mjsActuator.
MJAPI mjsActuator* mjs_asActuator(mjsElement* element);

// Safely cast an element as mjsSensor, or return NULL if the element is not an mjsSensor.
MJAPI mjsSensor* mjs_asSensor(mjsElement* element);

// Safely cast an element as mjsFlex, or return NULL if the element is not an mjsFlex.
MJAPI mjsFlex* mjs_asFlex(mjsElement* element);

// Safely cast an element as mjsPair, or return NULL if the element is not an mjsPair.
MJAPI mjsPair* mjs_asPair(mjsElement* element);

// Safely cast an element as mjsEquality, or return NULL if the element is not an mjsEquality.
MJAPI mjsEquality* mjs_asEquality(mjsElement* element);

// Safely cast an element as mjsExclude, or return NULL if the element is not an mjsExclude.
MJAPI mjsExclude* mjs_asExclude(mjsElement* element);

// Safely cast an element as mjsTendon, or return NULL if the element is not an mjsTendon.
MJAPI mjsTendon* mjs_asTendon(mjsElement* element);

// Safely cast an element as mjsNumeric, or return NULL if the element is not an mjsNumeric.
MJAPI mjsNumeric* mjs_asNumeric(mjsElement* element);

// Safely cast an element as mjsText, or return NULL if the element is not an mjsText.
MJAPI mjsText* mjs_asText(mjsElement* element);

// Safely cast an element as mjsTuple, or return NULL if the element is not an mjsTuple.
MJAPI mjsTuple* mjs_asTuple(mjsElement* element);

// Safely cast an element as mjsKey, or return NULL if the element is not an mjsKey.
MJAPI mjsKey* mjs_asKey(mjsElement* element);

// Safely cast an element as mjsMesh, or return NULL if the element is not an mjsMesh.
MJAPI mjsMesh* mjs_asMesh(mjsElement* element);

// Safely cast an element as mjsHField, or return NULL if the element is not an mjsHField.
MJAPI mjsHField* mjs_asHField(mjsElement* element);

// Safely cast an element as mjsSkin, or return NULL if the element is not an mjsSkin.
MJAPI mjsSkin* mjs_asSkin(mjsElement* element);

// Safely cast an element as mjsTexture, or return NULL if the element is not an mjsTexture.
MJAPI mjsTexture* mjs_asTexture(mjsElement* element);

// Safely cast an element as mjsMaterial, or return NULL if the element is not an mjsMaterial.
MJAPI mjsMaterial* mjs_asMaterial(mjsElement* element);

// Safely cast an element as mjsPlugin, or return NULL if the element is not an mjsPlugin.
MJAPI mjsPlugin* mjs_asPlugin(mjsElement* element);


//---------------------------------- Attribute setters ---------------------------------------------

// Copy buffer.
MJAPI void mjs_setBuffer(mjByteVec* dest, const void* array, int size);

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

// Get plugin attributes.
MJAPI const void* mjs_getPluginAttributes(const mjsPlugin* plugin);


//---------------------------------- Other utilities -----------------------------------------------

// Set element's default.
MJAPI void mjs_setDefault(mjsElement* element, const mjsDefault* def);

// Set element's enclosing frame, return 0 on success.
MJAPI int mjs_setFrame(mjsElement* dest, mjsFrame* frame);

// Resolve alternative orientations to quat, return error if any.
MJAPI const char* mjs_resolveOrientation(double quat[4], mjtByte degree, const char* sequence,
                                         const mjsOrientation* orientation);

// Transform body into a frame.
MJAPI mjsFrame* mjs_bodyToFrame(mjsBody** body);

// Set user payload.
MJAPI void mjs_setUserValue(mjsElement* element, const char* key, const void* data);

// Set user payload.
MJAPI void mjs_setUserValueWithCleanup(mjsElement* element, const char* key,
                                       const void* data,
                                       void (*cleanup)(const void*));

// Return user payload or NULL if none found.
MJAPI const void* mjs_getUserValue(mjsElement* element, const char* key);

// Delete user payload.
MJAPI void mjs_deleteUserValue(mjsElement* element, const char* key);


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
