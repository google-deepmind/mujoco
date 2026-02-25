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

#include "experimental/platform/gui_spec.h"

#include <string>
#include <string_view>

#include <imgui.h>
#include <mujoco/mjxmacro.h>
#include <mujoco/mujoco.h>
#include "experimental/platform/imgui_widgets.h"

// Define the mujoco X macros to add fields to the ImGui_DataTable.
// We limit the fields to the ones with a matching element by comparing the
// array size field (e.g. nbody) with the MATCH constexpr value.
#define X(TYPE, NAME, NELEM, SIZE) \
  if constexpr (#NELEM == MATCH) table.DataPtr(#NAME, ptr->NAME, index, SIZE);

// Simple wrapper around MJMODEL_POINTERS that prepares values we need for the
// X macro above.
#define MJMODEL_POINTERS_X(M)           \
  constexpr std::string_view MATCH(#M); \
  const auto* ptr = model;              \
  MJMODEL_POINTERS

// Simple wrapper around MJDATA_POINTERS that prepares values we need for the
// X macro above.
#define MJDATA_POINTERS_X(M)            \
  constexpr std::string_view MATCH(#M); \
  const auto* ptr = data;               \
  MJDATA_POINTERS

namespace mujoco::platform {

// Returns the index of the element in the spec. This is different from
// mjs_getId which returns the runtime ID of an element.
static int GetElementIndexInSpec(mjsElement* element) {
  int n = 0;
  mjSpec* spec = mjs_getSpec(element);
  mjsElement* iter = mjs_firstElement(spec, element->elemtype);
  while (iter) {
    if (iter == element) {
      return n;
    }
    iter = mjs_nextElement(spec, iter);
    ++n;
  }
  return -1;
}

// Returns a name for the element; either the element has a name, or we
// construct a unique name from the element's id (using mjs_getId) or index
// (using GetElementIndexInSpec).
static std::string ElementName(mjsElement* element) {
  const mjString* name = mjs_getName(element);
  std::string label = *name;
  if (label.empty()) {
    int id = mjs_getId(element);
    if (id == -1) {
      id = GetElementIndexInSpec(element);
    }
    const char* type_name = mju_type2Str(element->elemtype);
    label = "(" + std::string(type_name) + " " + std::to_string(id) + ")";
  }
  return label;
}

static void AddDeleteButton(mjsElement* element,
                            const SpecElementCallbackFn& on_delete) {
  if (on_delete) {
    // Right-align the delete button.
    const float button_width = ImGui::CalcTextSize(ICON_FA_TRASH_CAN).x +
                               ImGui::GetStyle().FramePadding.x * 2.0f;
    ImGui::SameLine(ImGui::GetWindowContentRegionMax().x - button_width);
    if (ImGui::SmallButton(ICON_FA_TRASH_CAN)) {
      on_delete(element);
    }
  }
}

static void SelectableElement(mjsElement* element,
                              mjsElement** selected_element,
                              const SpecElementCallbackFn& on_delete) {
  constexpr ImGuiSelectableFlags flags = ImGuiSelectableFlags_AllowOverlap;

  const std::string name = ElementName(element);
  const bool selected = (element == *selected_element);
  if (ImGui::Selectable(name.c_str(), selected, flags)) {
    *selected_element = element;
  }
  if (selected) {
    AddDeleteButton(element, on_delete);
  }
}

static void BodyChildrenGui(const char* heading, mjtObj type,
                            mjsElement** element, mjsBody* body,
                            const SpecElementCallbackFn& on_delete) {
  mjsElement* iter = mjs_firstChild(body, type, 0);
  if (!iter) {
    return;
  }

  constexpr ImGuiTreeNodeFlags tree_flags =
      ImGuiTreeNodeFlags_SpanAvailWidth | ImGuiTreeNodeFlags_DrawLinesFull;
  if (ImGui::TreeNodeEx(heading, tree_flags)) {
    while (iter) {
      SelectableElement(iter, element, on_delete);
      iter = mjs_nextChild(body, iter, 0);
    }
    ImGui::TreePop();
  }
}

static void ElementListGui(const char* heading, mjtObj type,
                           mjsElement** element, mjSpec* spec,
                           const SpecElementCallbackFn& on_delete) {
  mjsElement* iter = mjs_firstElement(spec, type);
  if (!iter) {
    return;
  }

  constexpr ImGuiTreeNodeFlags tree_flags =
      ImGuiTreeNodeFlags_SpanAvailWidth | ImGuiTreeNodeFlags_Framed;
  if (ImGui::TreeNodeEx(heading, tree_flags)) {
    while (iter) {
      SelectableElement(iter, element, on_delete);
      iter = mjs_nextElement(spec, iter);
    }
    ImGui::TreePop();
  }
}

static void BodyTreeGuiRecursive(mjsElement** element, mjsBody* body,
                                 const SpecElementCallbackFn& on_delete) {
  const std::string label = ElementName(body->element);

  ImGui::PushID(body);

  ImGuiTreeNodeFlags flags =
      ImGuiTreeNodeFlags_SpanAvailWidth | ImGuiTreeNodeFlags_Framed |
      ImGuiTreeNodeFlags_DrawLinesFull | ImGuiTreeNodeFlags_AllowOverlap;

  if (*element == body->element) {
    flags |= ImGuiTreeNodeFlags_Selected;
  }

  const bool tree_open = ImGui::TreeNodeEx(label.c_str(), flags);
  if (ImGui::IsItemClicked()) {
    *element = body->element;
  }
  if (*element == body->element) {
    AddDeleteButton(body->element, on_delete);
  }

  if (tree_open) {
    mjsElement* iter = mjs_firstChild(body, mjOBJ_BODY, 0);
    while (iter) {
      BodyTreeGuiRecursive(element, mjs_asBody(iter), on_delete);
      iter = mjs_nextChild(body, iter, 0);
    }

    BodyChildrenGui("Frames", mjOBJ_FRAME, element, body, on_delete);
    BodyChildrenGui("Sites", mjOBJ_SITE, element, body, on_delete);
    BodyChildrenGui("Joints", mjOBJ_JOINT, element, body, on_delete);
    BodyChildrenGui("Geoms", mjOBJ_GEOM, element, body, on_delete);
    BodyChildrenGui("Lights", mjOBJ_LIGHT, element, body, on_delete);
    BodyChildrenGui("Cameras", mjOBJ_CAMERA, element, body, on_delete);

    ImGui::TreePop();
  }

  ImGui::PopID();
}

void SpecExplorerGui(mjsElement** element, mjSpec* spec,
                     const SpecElementCallbackFn& on_delete) {
  const ImGuiTreeNodeFlags flags =
      ImGuiTreeNodeFlags_SpanAvailWidth | ImGuiTreeNodeFlags_Framed;

  if (ImGui::TreeNodeEx("Body Tree", flags)) {
    mjsElement* root = mjs_firstElement(spec, mjOBJ_BODY);
    if (root) {
      mjsBody* body = mjs_asBody(root);
      if (body) {
        BodyTreeGuiRecursive(element, body, on_delete);
      }
    }
    ImGui::TreePop();
  }

  auto list = [&](const char* heading, mjtObj type) {
    ElementListGui(heading, type, element, spec, on_delete);
  };

  ImGui::PushID(spec);

  // Non-tree elements.
  if (ImGui::TreeNodeEx("Elements", flags)) {
    list("Actuators", mjOBJ_ACTUATOR);
    list("Sensors", mjOBJ_SENSOR);
    list("Flexes", mjOBJ_FLEX);
    list("Tendons", mjOBJ_TENDON);
    list("Pair", mjOBJ_PAIR);
    list("Exclude", mjOBJ_EXCLUDE);
    list("Equality", mjOBJ_EQUALITY);
    list("Numeric", mjOBJ_NUMERIC);
    list("Text", mjOBJ_TEXT);
    list("Tuple", mjOBJ_TUPLE);
    list("Key", mjOBJ_KEY);
    list("Default", mjOBJ_DEFAULT);
    ImGui::TreePop();
  }

  // Assets.
  if (ImGui::TreeNodeEx("Assets", flags)) {
    list("Meshes", mjOBJ_MESH);
    list("Height Fields", mjOBJ_HFIELD);
    list("Skins", mjOBJ_SKIN);
    list("Textures", mjOBJ_TEXTURE);
    list("Materials", mjOBJ_MATERIAL);
    ImGui::TreePop();
  }

  ImGui::PopID();
}

void ElementSpecGui(const mjSpec* spec, mjsElement* element) {
  if (element == nullptr) {
    return;
  }

  ImGui_SpecElementTable table;
  switch (element->elemtype) {
    case mjOBJ_BODY: {
      mjsBody* body = mjs_asBody(element);
      table("childclass", body->childclass, "childclass name");
      table("pos", body->pos, "frame position");
      table("quat", body->quat, "alt", body->alt, "frame orientation");
      table("ipos", body->ipos, "inertial frame position");
      table("iquat", body->iquat, "ialt", body->ialt, "inertial frame orientation");
      table("mass", body->mass, "mass");
      table("inertia", body->inertia, "diagonal inertia (in i-frame)");
      table("fullinertia", body->fullinertia, "non-axis-aligned inertia matrix");
      table("mocap", body->mocap, "is this a mocap body");
      table("gravcomp", body->gravcomp, "gravity compensation");
      table("explicitinertial", body->explicitinertial, "whether to save the body with explicit inertial clause");
      table("sleep", body->sleep, "sleep policy");
      table("info", body->info, "message appended to compiler errors");
      break;
    }
    case mjOBJ_JOINT: {
      mjsJoint* joint = mjs_asJoint(element);
      table("pos", joint->pos, "anchor position");
      table("axis", joint->axis, "joint axis");
      table("ref", joint->ref, "value at reference configuration: qpos0");
      table("align", joint->align, "align free joint with body com (mjtAlignFree)");
      table("stiffness", joint->stiffness, "stiffness coefficient");
      table("springref", joint->springref, "spring reference value: qpos_spring");
      table("springdamper", joint->springdamper, "timeconst, dampratio");
      table("limited", joint->limited, "does joint have limits (mjtLimited)");
      table("range", joint->range, "joint limits");
      table("margin", joint->margin, "margin value for joint limit detection");
      table("solref_limit", joint->solref_limit, "solver reference: joint limits");
      table("solimp_limit", joint->solimp_limit, "solver impedance: joint limits");
      table("actfrclimited", joint->actfrclimited, "are actuator forces on joint limited (mjtLimited)");
      table("actfrcrange", joint->actfrcrange, "actuator force limits");
      table("armature", joint->armature, "armature inertia (mass for slider)");
      table("damping", joint->damping, "damping coefficient");
      table("frictionloss", joint->frictionloss, "friction loss");
      table("solref_friction", joint->solref_friction, "solver reference: dof friction");
      table("solimp_friction", joint->solimp_friction, "solver impedance: dof friction");
      table("group", joint->group, "group");
      table("actgravcomp", joint->actgravcomp, "is gravcomp force applied via actuators");
      table("info", joint->info, "message appended to compiler errors");
      break;
    }
    case mjOBJ_ACTUATOR: {
      mjsActuator* actuator = mjs_asActuator(element);
      table("gaintype", actuator->gaintype, "gain type");
      table("gainprm", actuator->gainprm, "gain parameters");
      table("biastype", actuator->biastype, "bias type");
      table("biasprm", actuator->biasprm, "bias parameters");
      table("dyntype", actuator->dyntype, "dynamics type");
      table("dynprm", actuator->dynprm, "dynamics parameters");
      table("actdim", actuator->actdim, "number of activation variables");
      table("actearly", actuator->actearly, "apply next activations to qfrc");
      table("trntype", actuator->trntype, "transmission type");
      table("gear", actuator->gear, "length and transmitted force scaling");
      table("target", actuator->target, "name of transmission target");
      table("refsite", actuator->refsite, "reference site, for site transmission");
      table("slidersite", actuator->slidersite, "site defining cylinder, for slider-crank");
      table("cranklength", actuator->cranklength, "crank length, for slider-crank");
      table("lengthrange", actuator->lengthrange, "transmission length range");
      table("inheritrange", actuator->inheritrange, "automatic range setting for position and intvelocity");
      table("ctrllimited", actuator->ctrllimited, "are control limits defined (mjtLimited)");
      table("ctrlrange", actuator->ctrlrange, "control range");
      table("forcelimited", actuator->forcelimited, "are force limits defined (mjtLimited)");
      table("forcerange", actuator->forcerange, "force range");
      table("actlimited", actuator->actlimited, "are activation limits defined (mjtLimited)");
      table("actrange", actuator->actrange, "activation range");
      table("group", actuator->group, "group");
      table("nsample", actuator->nsample, "number of samples in history buffer");
      table("interp", actuator->interp, "interpolation order (0=ZOH, 1=linear, 2=cubic)");
      table("delay", actuator->delay, "delay time in seconds; 0: no delay");
      table("info", actuator->info, "message appended to compiler errors");
      break;
    }
    case mjOBJ_SENSOR: {
      mjsSensor* sensor = mjs_asSensor(element);
      table("type", sensor->type, "type of sensor");
      table("objtype", sensor->objtype, "type of sensorized object");
      table("objname", sensor->objname, "name of sensorized object");
      table("reftype", sensor->reftype, "type of referenced object");
      table("refname", sensor->refname, "name of referenced object");
      table("intprm", sensor->intprm, "integer parameters");
      table("datatype", sensor->datatype, "data type for sensor measurement");
      table("needstage", sensor->needstage, "compute stage needed to simulate sensor");
      table("dim", sensor->dim, "number of scalar outputs");
      table("cutoff", sensor->cutoff, "cutoff for real and positive datatypes");
      table("noise", sensor->noise, "noise stdev");
      table("nsample", sensor->nsample, "number of samples in history buffer");
      table("interp", sensor->interp, "interpolation order (0=ZOH, 1=linear, 2=cubic)");
      table("delay", sensor->delay, "delay time in seconds");
      table("interval", sensor->interval, "[period, time_prev] in seconds");
      table("info", sensor->info, "message appended to compiler errors");
      break;
    }
    case mjOBJ_SITE: {
      mjsSite* site = mjs_asSite(element);
      table("pos", site->pos, "position");
      table("quat", site->quat, "alt", site->alt, "orientation");
      table("fromto", site->fromto, "alternative for capsule, cylinder, box, ellipsoid");
      table("size", site->size, "geom size");
      table("type", site->type, "geom type");
      table("material", site->material, "name of material");
      table("group", site->group, "group");
      table("rgba", site->rgba, "rgba when material is omitted");
      table("info", site->info, "message appended to compiler errors");
      break;
    }
    case mjOBJ_FRAME: {
      mjsFrame* frame = mjs_asFrame(element);
      table("childclass", frame->childclass, "childclass name");
      table("pos", frame->pos, "position");
      table("quat", frame->quat, "alt", frame->alt, "orientation");
      table("info", frame->info, "message appended to compiler errors");
      break;
    }
    case mjOBJ_GEOM: {
      mjsGeom* geom = mjs_asGeom(element);
      table("type", geom->type, "geom type");
      table("pos", geom->pos, "position");
      table("quat", geom->quat, "alt", geom->alt, "orientation");
      table("fromto", geom->fromto, "alternative for capsule, cylinder, box, ellipsoid");
      table("size", geom->size, "type-specific size");
      table("contype", geom->contype, "contact type");
      table("conaffinity", geom->conaffinity, "contact affinity");
      table("condim", geom->condim, "contact dimensionality");
      table("priority", geom->priority, "contact priority");
      table("friction", geom->friction, "one-sided friction coefficients: slide, roll, spin");
      table("solmix", geom->solmix, "solver mixing for contact pairs");
      table("solref", geom->solref, "solver reference");
      table("solimp", geom->solimp, "solver impedance");
      table("margin", geom->margin, "margin for contact detection");
      table("gap", geom->gap, "include in solver if dist < margin-gap");
      table("mass", geom->mass, "used to compute density");
      table("density", geom->density, "used to compute mass and inertia from volume or surface");
      table("typeinertia", geom->typeinertia, "selects between surface and volume inertia");
      table("fluid_ellipsoid", geom->fluid_ellipsoid, "whether ellipsoid-fluid model is active");
      table("fluid_coefs", geom->fluid_coefs, "ellipsoid-fluid interaction coefs");
      table("material", geom->material, "name of material");
      table("rgba", geom->rgba, "rgba when material is omitted");
      table("group", geom->group, "group");
      table("hfieldname", geom->hfieldname, "heightfield attached to geom");
      table("meshname", geom->meshname, "mesh attached to geom");
      table("fitscale", geom->fitscale, "scale mesh uniformly");
      table("info", geom->info, "message appended to compiler errors");
      break;
    }
    case mjOBJ_LIGHT: {
      mjsLight* light = mjs_asLight(element);
      table("pos", light->pos, "position");
      table("dir", light->dir, "direction");
      table("mode", light->mode, "tracking mode");
      table("targetbody", light->targetbody, "target body for targeting");
      table("active", light->active, "is light active");
      table("type", light->type, "type of light");
      table("texture", light->texture, "texture name for image lights");
      table("castshadow", light->castshadow, "does light cast shadows");
      table("bulbradius", light->bulbradius, "bulb radius, for soft shadows");
      table("intensity", light->intensity, "intensity, in candelas");
      table("range", light->range, "range of effectiveness");
      table("attenuation", light->attenuation, "OpenGL attenuation (quadratic model)");
      table("cutoff", light->cutoff, "OpenGL cutoff");
      table("exponent", light->exponent, "OpenGL exponent");
      table("ambient", light->ambient, "ambient color");
      table("diffuse", light->diffuse, "diffuse color");
      table("specular", light->specular, "specular color");
      table("info", light->info, "message appended to compiler errorsx");
      break;
    }
    case mjOBJ_CAMERA: {
      mjsCamera* camera = mjs_asCamera(element);
      table("pos", camera->pos, "position");
      table("quat", camera->quat, "alt", camera->alt, "orientation");
      table("mode", camera->mode, "tracking mode");
      table("targetbody", camera->targetbody, "target body for tracking/targeting");
      table("proj", camera->proj, "camera projection type");
      table("resolution", camera->resolution, "resolution (pixel)");
      table("output", camera->output, "bit flags for output type");
      table("fovy", camera->fovy, "y-field of view");
      table("ipd", camera->ipd, "inter-pupillary distance");
      table("intrinsic", camera->intrinsic, "camera intrinsics (length)");
      table("sensor_size", camera->sensor_size, "sensor size (length)");
      table("focal_length", camera->focal_length, "focal length (length)");
      table("focal_pixel", camera->focal_pixel, "focal length (pixel)");
      table("principal_length", camera->principal_length, "principal point (length)");
      table("principal_pixel", camera->principal_pixel, "principal point (pixel)");
      table("info", camera->info, "message appended to compiler errors");
      break;
    }
    case mjOBJ_MESH: {
      mjsMesh* mesh = mjs_asMesh(element);
      table("content_type", mesh->content_type, "content type of file");
      table("file", mesh->file, "mesh file");
      table("refpos", mesh->refpos, "reference position");
      table("refquat", mesh->refquat, "reference orientation");
      table("scale", mesh->scale, "rescale mesh");
      table("inertia", mesh->inertia, "inertia type (convex, legacy, exact, shell)");
      table("smoothnormal", mesh->smoothnormal, "do not exclude large-angle faces from normals");
      table("needsdf", mesh->needsdf, "compute sdf from mesh");
      table("maxhullvert", mesh->maxhullvert, "maximum vertex count for the convex hull");
      table("material", mesh->material, "name of material");
      table("info", mesh->info, "message appended to compiler errors");
      break;
    }
    case mjOBJ_HFIELD: {
      mjsHField* hfield = mjs_asHField(element);
      table("content_type", hfield->content_type, "content type of file");
      table("file", hfield->file, "file: (nrow, ncol, [elevation data])");
      table("size", hfield->size, "hfield size (ignore referencing geom size)");
      table("nrow", hfield->nrow, "number of rows");
      table("ncol", hfield->ncol, "number of columns");
      table("info", hfield->info, "message appended to compiler errors");
      break;
    }
    case mjOBJ_SKIN: {
      mjsSkin* skin = mjs_asSkin(element);
      table("file", skin->file, "skin file");
      table("material", skin->material, "name of material used for rendering");
      table("rgba", skin->rgba, "rgba when material is omitted");
      table("inflate", skin->inflate, "inflate in normal direction");
      table("group", skin->group, "group for visualization");
      table("info", skin->info, "message appended to compiler errors");
      break;
    }
    case mjOBJ_FLEX: {
      mjsFlex* flex = mjs_asFlex(element);
      table("contype", flex->contype, "contact type");
      table("conaffinity", flex->conaffinity, "contact affinity");
      table("condim", flex->condim, "contact dimensionality");
      table("priority", flex->priority, "contact priority");
      table("friction", flex->friction, "one-sided friction coefficients: slide, roll, spin");
      table("solmix", flex->solmix, "solver mixing for contact pairs");
      table("solref", flex->solref, "solver reference");
      table("solimp", flex->solimp, "solver impedance");
      table("margin", flex->margin, "margin for contact detection");
      table("gap", flex->gap, "include in solver if dist<margin-gap");
      table("dim", flex->dim, "element dimensionality");
      table("radius", flex->radius, "radius around primitive element");
      table("size", flex->size, "vertex bounding box half sizes in qpos0");
      table("internal", flex->internal, "enable internal collisions");
      table("flatskin", flex->flatskin, "render flex skin with flat shading");
      table("selfcollide", flex->selfcollide, "mode for flex self collision");
      table("vertcollide", flex->vertcollide, "mode for vertex collision");
      table("passive", flex->passive, "mode for passive collisions");
      table("activelayers", flex->activelayers, "number of active element layers in 3D");
      table("group", flex->group, "group for visualization");
      table("edgestiffness", flex->edgestiffness, "edge stiffness");
      table("edgedamping", flex->edgedamping, "edge damping");
      table("rgba", flex->rgba, "rgba when material is omitted");
      table("material", flex->material, "name of material used for rendering");
      table("young", flex->young, "Young's modulus");
      table("poisson", flex->poisson, "Poisson's ratio");
      table("damping", flex->damping, "Rayleigh's damping");
      table("thickness", flex->thickness, "thickness (2D only)");
      table("elastic2d", flex->elastic2d, "2D passive forces; 0: none, 1: bending, 2: stretching, 3: both");
      table("info", flex->info, "message appended to compiler errors");
      break;
    }
    case mjOBJ_TENDON: {
      mjsTendon* tendon = mjs_asTendon(element);
      table("stiffness", tendon->stiffness, "stiffness coefficient");
      table("springlength", tendon->springlength, "spring resting length; {-1, -1}: use qpos_spring");
      table("damping", tendon->damping, "damping coefficient");
      table("frictionloss", tendon->frictionloss, "friction loss");
      table("solref_friction", tendon->solref_friction, "solver reference: tendon friction");
      table("solimp_friction", tendon->solimp_friction, "solver impedance: tendon friction");
      table("armature", tendon->armature, "inertia associated with tendon velocity");
      table("limited", tendon->limited, "does tendon have limits (mjtLimited)");
      table("actfrclimited", tendon->actfrclimited, "does tendon have actuator force limits");
      table("range", tendon->range, "length limits");
      table("actfrcrange", tendon->actfrcrange, "actuator force limits");
      table("margin", tendon->margin, "margin value for tendon limit detection");
      table("solref_limit", tendon->solref_limit, "solver reference: tendon limits");
      table("solimp_limit", tendon->solimp_limit, "solver impedance: tendon limits");
      table("material", tendon->material, "name of material for rendering");
      table("width", tendon->width, "width for rendering");
      table("rgba", tendon->rgba, "rgba when material is omitted");
      table("group", tendon->group, "group");
      table("info", tendon->info, "message appended to errors");
      break;
    }
    case mjOBJ_TEXTURE: {
      mjsTexture* texture = mjs_asTexture(element);
      table("type", texture->type, "texture type");
      table("colorspace", texture->colorspace, "colorspace");
      table("builtin", texture->builtin, "builtin type (mjtBuiltin)");
      table("mark", texture->mark, "mark type (mjtMark)");
      table("rgb1", texture->rgb1, "first color for builtin");
      table("rgb2", texture->rgb2, "second color for builtin");
      table("markrgb", texture->markrgb, "mark color");
      table("random", texture->random, "probability of random dots");
      table("height", texture->height, "height in pixels (square for cube and skybox)");
      table("width", texture->width, "width in pixels");
      table("nchannel", texture->nchannel, "number of channels");
      table("content_type", texture->content_type, "content type of file");
      table("file", texture->file, "png file to load; use for all sides of cube");
      table("gridsize", texture->gridsize, "size of grid for composite file; (1,1)-repeat");
      table("gridlayout", texture->gridlayout, "row-major: L,R,F,B,U,D for faces; . for unused");
      table("cubefiles", texture->cubefiles, "different file for each side of the cube");
      table("hflip", texture->hflip, "horizontal flip");
      table("vflip", texture->vflip, "vertical flip");
      table("info", texture->info, "message appended to compiler errors");
      break;
    }
    case mjOBJ_MATERIAL: {
      mjsMaterial* material = mjs_asMaterial(element);
      table("textures", material->textures, "names of textures (empty: none)");
      table("texuniform", material->texuniform, "make texture cube uniform");
      table("texrepeat", material->texrepeat, "texture repetition for 2D mapping");
      table("emission", material->emission, "emission");
      table("specular", material->specular, "specular");
      table("shininess", material->shininess, "shininess");
      table("reflectance", material->reflectance, "reflectance");
      table("metallic", material->metallic, "metallic");
      table("roughness", material->roughness, "roughness");
      table("rgba", material->rgba, "rgba");
      table("info", material->info, "message appended to compiler errors");
      break;
    }
    case mjOBJ_PAIR: {
      mjsPair* pair = mjs_asPair(element);
      table("geomname1", pair->geomname1, "name of geom 1");
      table("geomname2", pair->geomname2, "name of geom 2");
      table("condim", pair->condim, "contact dimensionality");
      table("solref", pair->solref, "solver reference, normal direction");
      table("solreffriction", pair->solreffriction, "solver reference, frictional directions");
      table("solimp", pair->solimp, "solver impedance");
      table("margin", pair->margin, "margin for contact detection");
      table("gap", pair->gap, "include in solver if dist<margin-gap");
      table("friction", pair->friction, "full contact friction");
      table("info", pair->info, "message appended to errors");
      break;
    }
    case mjOBJ_EQUALITY: {
      mjsEquality* equality = mjs_asEquality(element);
      table("type", equality->type, "constraint type");
      table("data", equality->data, "type-dependent data");
      table("active", equality->active, "is equality initially active");
      table("name1", equality->name1, "name of object 1");
      table("name2", equality->name2, "name of object 2");
      table("objtype", equality->objtype, "type of both objects");
      table("solref", equality->solref, "solver reference");
      table("solimp", equality->solimp, "solver impedance");
      table("info", equality->info, "message appended to errors");
      break;
    }
    case mjOBJ_EXCLUDE: {
      mjsExclude* exclude = mjs_asExclude(element);
      table("bodyname1", exclude->bodyname1, "name of geom 1");
      table("bodyname2", exclude->bodyname2, "name of geom 2");
      table("info", exclude->info, "message appended to errors");
      break;
    }
    case mjOBJ_NUMERIC: {
      mjsNumeric* numeric = mjs_asNumeric(element);
      table("data", numeric->data, "initialization data");
      table("size", numeric->size, "array size, can be bigger than data size");
      table("info", numeric->info, "message appended to errors");
      break;
    }
    case mjOBJ_TEXT: {
      mjsText* text = mjs_asText(element);
      table("data", text->data, "text string");
      table("info", text->info, "message appended to compiler errors");
      break;
    }
    case mjOBJ_TUPLE: {
      mjsTuple* tuple = mjs_asTuple(element);
      table("objtype", tuple->objtype, "object types");
      table("objname", tuple->objname, "object names");
      table("objprm", tuple->objprm, "object parameters");
      table("info", tuple->info, "message appended to compiler errors");
      break;
    }
    case mjOBJ_KEY: {
      mjsKey* key = mjs_asKey(element);
      table("time", key->time, "time");
      table("qpos", key->qpos, "qpos");
      table("qvel", key->qvel, "qvel");
      table("act", key->act, "act");
      table("mpos", key->mpos, "mocap pos");
      table("mquat", key->mquat, "mocap quat");
      table("ctrl", key->ctrl, "ctrl");
      table("info", key->info, "message appended to compiler errors");
      break;
    }
    case mjOBJ_PLUGIN: {
      mjsPlugin* plugin = mjs_asPlugin(element);
      table("name", plugin->name, "instance name");
      table("plugin_name", plugin->plugin_name, "plugin name");
      table("active", plugin->active, "is the plugin active");
      table("info", plugin->info, "message appended to compiler errors");
      break;
    }
    default:
      // ignore other types
      break;
  }
}

void ElementModelGui(const mjModel* model, mjsElement* element) {
  if (element == nullptr) {
    return;
  }

  ImGui_DataPtrTable table;
  const int index = mjs_getId(element);

  MJMODEL_POINTERS_PREAMBLE(model);
  switch (element->elemtype) {
    case mjOBJ_BODY: {
      table.SetPrefix("body_");
      MJMODEL_POINTERS_X(nbody)
      break;
    }
    case mjOBJ_JOINT: {
      table.SetPrefix("jnt_");
      MJMODEL_POINTERS_X(njnt)
      break;
    }
    case mjOBJ_ACTUATOR: {
      table.SetPrefix("actuator_");
      MJMODEL_POINTERS_X(nu)
      break;
    }
    case mjOBJ_SENSOR: {
      table.SetPrefix("sensor_");
      MJMODEL_POINTERS_X(nsensor)
      break;
    }
    case mjOBJ_SITE: {
      table.SetPrefix("site_");
      MJMODEL_POINTERS_X(nsite)
      break;
    }
    case mjOBJ_GEOM: {
      table.SetPrefix("geom_");
      MJMODEL_POINTERS_X(ngeom)
      break;
    }
    case mjOBJ_LIGHT: {
      table.SetPrefix("light_");
      MJMODEL_POINTERS_X(nlight)
      break;
    }
    case mjOBJ_CAMERA: {
      table.SetPrefix("cam_");
      MJMODEL_POINTERS_X(ncam)
      break;
    }
    case mjOBJ_MESH: {
      table.SetPrefix("mesh_");
      MJMODEL_POINTERS_X(nmesh)
      break;
    }
    case mjOBJ_HFIELD: {
      table.SetPrefix("hfield_");
      MJMODEL_POINTERS_X(nhfield)
      break;
    }
    case mjOBJ_SKIN: {
      table.SetPrefix("skin_");
      MJMODEL_POINTERS_X(nskin)
      break;
    }
    case mjOBJ_FLEX: {
      table.SetPrefix("flex_");
      MJMODEL_POINTERS_X(nflex)
      break;
    }
    case mjOBJ_TENDON: {
      table.SetPrefix("tendon_");
      MJMODEL_POINTERS_X(ntendon)
      break;
    }
    case mjOBJ_TEXTURE: {
      table.SetPrefix("tex_");
      MJMODEL_POINTERS_X(ntex)
      break;
    }
    case mjOBJ_MATERIAL: {
      table.SetPrefix("mat_");
      MJMODEL_POINTERS_X(nmat)
      break;
    }
    default:
      // ignore other types
      break;
  }
}

void ElementDataGui(const mjData* data, mjsElement* element) {
  if (element == nullptr) {
    return;
  }

  ImGui_DataPtrTable table;
  const int index = mjs_getId(element);

  switch (element->elemtype) {
    case mjOBJ_BODY: {
      MJDATA_POINTERS_X(nbody)
      break;
    }
    case mjOBJ_JOINT: {
      MJDATA_POINTERS_X(njnt)
      break;
    }
    case mjOBJ_SITE: {
      table.SetPrefix("site_");
      MJDATA_POINTERS_X(nsite)
      break;
    }
    case mjOBJ_GEOM: {
      table.SetPrefix("geom_");
      MJDATA_POINTERS_X(ngeom)
      break;
    }
    case mjOBJ_CAMERA: {
      table.SetPrefix("cam_");
      MJDATA_POINTERS_X(ncam)
      break;
    }
    case mjOBJ_LIGHT: {
      table.SetPrefix("light_");
      MJDATA_POINTERS_X(nlight)
      break;
    }
    case mjOBJ_SENSOR: {
      table.SetPrefix("sensor_");
      MJDATA_POINTERS_X(nsensor)
      break;
    }
    case mjOBJ_MESH: {
      table.SetPrefix("mesh_");
      MJDATA_POINTERS_X(nmesh)
      break;
    }
    case mjOBJ_HFIELD: {
      table.SetPrefix("hfield_");
      MJDATA_POINTERS_X(nhfield)
      break;
    }
    case mjOBJ_SKIN: {
      table.SetPrefix("skin_");
      MJDATA_POINTERS_X(nskin)
      break;
    }
    case mjOBJ_FLEX: {
      table.SetPrefix("flex_");
      MJDATA_POINTERS_X(nflex)
      break;
    }
    case mjOBJ_TENDON: {
      table.SetPrefix("ten_");
      MJDATA_POINTERS_X(ntendon)
      break;
    }
    case mjOBJ_TEXTURE: {
      table.SetPrefix("tex_");
      MJDATA_POINTERS_X(ntex)
      break;
    }
    case mjOBJ_MATERIAL: {
      table.SetPrefix("mat_");
      MJDATA_POINTERS_X(nmat)
      break;
    }
    default:
      break;
  }
}
}  // namespace mujoco::platform
