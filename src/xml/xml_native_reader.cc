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

#include "xml/xml_native_reader.h"

#include <array>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <functional>
#include <limits>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <mujoco/mujoco.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjplugin.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>
#include "engine/engine_plugin.h"
#include "engine/engine_support.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include <mujoco/mjspec.h>
#include "user/user_api.h"
#include "user/user_composite.h"
#include "user/user_flexcomp.h"
#include "user/user_util.h"
#include "xml/xml_base.h"
#include "xml/xml_util.h"
#include "tinyxml2.h"
#ifdef mjUSEUSD
#include <mujoco/experimental/usd/usd.h>
#endif  // mjUSEUSD

namespace {
using std::string;
using std::string_view;
using std::vector;
using mujoco::user::FilePath;
using tinyxml2::XMLElement;

void ReadPluginConfigs(tinyxml2::XMLElement* elem, mjsPlugin* p) {
  std::map<string, string, std::less<> > config_attribs;
  XMLElement* child = FirstChildElement(elem);
  while (child) {
    string_view name = child->Value();
    if (name == "config") {
      string key, value;
      mjXUtil::ReadAttrTxt(child, "key", key, /* required = */ true);
      if (config_attribs.find(key) != config_attribs.end()) {
        string err = "duplicate config key: " + key;
        throw mjXError(child, "%s", err.c_str());
      }
      mjXUtil::ReadAttrTxt(child, "value", value, /* required = */ true);
      config_attribs[key] = value;
    }
    child = NextSiblingElement(child);
  }

  if (!p && !config_attribs.empty()) {
    throw mjXError(elem,
                   "plugin configuration attributes cannot be used in an "
                   "element that references a predefined plugin instance");
  } else if (p) {
    mjs_setPluginAttributes(p, &config_attribs);
  }
}

static void UpdateString(string& psuffix, int count, int i) {
  int ndigits = std::to_string(count).length();
  string i_string = std::to_string(i);
  string prefix = "";
  while (ndigits-- > i_string.length()) {
    prefix += '0';
  }
  psuffix += prefix + i_string;
}
}  // namespace


//---------------------------------- MJCF schema ---------------------------------------------------

std::vector<const char*> MJCF[nMJCF] = {
{"mujoco", "!", "model"},
{"<"},
    {"compiler", "*", "autolimits", "boundmass", "boundinertia", "settotalmass",
        "balanceinertia", "strippath", "coordinate", "angle", "fitaabb", "eulerseq",
        "meshdir", "texturedir", "discardvisual", "usethread", "fusestatic", "inertiafromgeom",
        "inertiagrouprange", "saveinertial", "assetdir", "alignfree"},
    {"<"},
        {"lengthrange", "?", "mode", "useexisting", "uselimit",
            "accel", "maxforce", "timeconst", "timestep",
            "inttotal", "interval", "tolrange"},
    {">"},

    {"option", "*",
        "timestep", "impratio", "tolerance", "ls_tolerance", "noslip_tolerance",
        "ccd_tolerance", "sleep_tolerance", "gravity", "wind", "magnetic", "density", "viscosity",
        "o_margin", "o_solref", "o_solimp", "o_friction",
        "integrator", "cone", "jacobian",
        "solver", "iterations", "ls_iterations", "noslip_iterations", "ccd_iterations",
        "sdf_iterations", "sdf_initpoints", "actuatorgroupdisable"},
    {"<"},
        {"flag", "?", "constraint", "equality", "frictionloss", "limit", "contact",
            "spring", "damper", "gravity", "clampctrl", "warmstart", "filterparent", "actuation",
            "refsafe", "sensor", "midphase", "eulerdamp", "autoreset", "nativeccd", "island",
            "override", "energy", "fwdinv", "invdiscrete", "multiccd", "sleep"},
    {">"},

    {"size", "*", "memory", "njmax", "nconmax", "nstack", "nuserdata", "nkey",
        "nuser_body", "nuser_jnt", "nuser_geom", "nuser_site", "nuser_cam",
        "nuser_tendon", "nuser_actuator", "nuser_sensor"},

    {"visual", "*"},
    {"<"},
        {"global", "?", "cameraid", "orthographic", "fovy", "ipd", "azimuth", "elevation",
            "linewidth", "glow", "offwidth", "offheight", "realtime", "ellipsoidinertia",
            "bvactive"},
        {"quality", "?", "shadowsize", "offsamples", "numslices", "numstacks",
            "numquads"},
        {"headlight", "?", "ambient", "diffuse", "specular", "active"},
        {"map", "?", "stiffness", "stiffnessrot", "force", "torque", "alpha",
            "fogstart", "fogend", "znear", "zfar", "haze", "shadowclip", "shadowscale",
            "actuatortendon"},
        {"scale", "?", "forcewidth", "contactwidth", "contactheight", "connect", "com",
            "camera", "light", "selectpoint", "jointlength", "jointwidth", "actuatorlength",
            "actuatorwidth", "framelength", "framewidth", "constraint", "slidercrank", "frustum"},
        {"rgba", "?", "fog", "haze", "force", "inertia", "joint",
            "actuator", "actuatornegative", "actuatorpositive", "com",
            "camera", "light", "selectpoint", "connect", "contactpoint", "contactforce",
            "contactfriction", "contacttorque", "contactgap", "rangefinder",
            "constraint", "slidercrank", "crankbroken", "frustum", "bv", "bvactive"},
    {">"},

    {"statistic", "*", "meaninertia", "meanmass", "meansize", "extent", "center"},

    {"default", "R", "class"},
    {"<"},
        {"mesh", "?", "scale", "maxhullvert", "inertia"},
        {"material", "?", "texture", "emission", "specular", "shininess",
            "reflectance", "metallic", "roughness", "rgba", "texrepeat", "texuniform"},
        {"<"},
            {"layer", "*", "texture", "role"},
        {">"},
        {"joint", "?", "type", "group", "pos", "axis", "springdamper",
            "limited", "actuatorfrclimited", "solreflimit", "solimplimit",
            "solreffriction", "solimpfriction", "stiffness", "range", "actuatorfrcrange",
            "actuatorgravcomp", "margin", "ref", "springref", "armature", "damping",
            "frictionloss", "user"},
        {"geom", "?", "type", "pos", "quat", "contype", "conaffinity", "condim",
            "group", "priority", "size", "material", "friction", "mass", "density",
            "shellinertia", "solmix", "solref", "solimp",
            "margin", "gap", "fromto", "axisangle", "xyaxes", "zaxis", "euler",
            "hfield", "mesh", "fitscale", "rgba", "fluidshape", "fluidcoef", "user"},
        {"site", "?", "type", "group", "pos", "quat", "material",
            "size", "fromto", "axisangle", "xyaxes", "zaxis", "euler", "rgba", "user"},
        {"camera", "?", "projection", "fovy", "ipd", "resolution", "pos", "quat",
            "axisangle", "xyaxes", "zaxis", "euler", "mode", "focal", "focalpixel",
            "principal", "principalpixel", "sensorsize", "user"},
        {"light", "?", "pos", "dir", "bulbradius", "intensity", "range",
            "directional", "type", "castshadow", "active", "attenuation", "cutoff", "exponent",
            "ambient", "diffuse", "specular", "mode"},
        {"pair", "?", "condim", "friction", "solref", "solreffriction", "solimp",
         "gap", "margin"},
        {"equality", "?", "active", "solref", "solimp"},
        {"tendon", "?", "group", "limited", "range",
            "solreflimit", "solimplimit", "solreffriction", "solimpfriction",
            "frictionloss", "springlength", "width", "material",
            "margin", "stiffness", "damping", "rgba", "user"},
        {"general", "?", "ctrllimited", "forcelimited", "actlimited", "ctrlrange",
            "forcerange", "actrange", "gear", "cranklength", "user", "group", "actdim",
            "dyntype", "gaintype", "biastype", "dynprm", "gainprm", "biasprm", "actearly"},
        {"motor", "?", "ctrllimited", "forcelimited", "ctrlrange", "forcerange",
            "gear", "cranklength", "user", "group"},
        {"position", "?", "ctrllimited", "forcelimited", "ctrlrange", "inheritrange",
            "forcerange", "gear", "cranklength", "user", "group", "kp", "kv", "dampratio", "timeconst"},
        {"velocity", "?", "ctrllimited", "forcelimited", "ctrlrange", "forcerange",
            "gear", "cranklength", "user", "group", "kv"},
        {"intvelocity", "?", "ctrllimited", "forcelimited",
            "ctrlrange", "forcerange", "actrange", "inheritrange",
            "gear", "cranklength", "user", "group",
            "kp", "kv", "dampratio"},
        {"damper", "?", "forcelimited", "ctrlrange", "forcerange",
            "gear", "cranklength", "user", "group", "kv"},
        {"cylinder", "?", "ctrllimited", "forcelimited", "ctrlrange", "forcerange",
            "gear", "cranklength", "user", "group",
            "timeconst", "area", "diameter", "bias"},
        {"muscle", "?", "ctrllimited", "forcelimited", "ctrlrange", "forcerange",
            "gear", "cranklength", "user", "group",
            "timeconst", "range", "force", "scale",
            "lmin", "lmax", "vmax", "fpmax", "fvmax"},
        {"adhesion", "?", "forcelimited", "ctrlrange", "forcerange",
            "gain", "user", "group"},
    {">"},

    {"extension", "*"},
    {"<"},
        {"plugin", "*", "plugin"},
        {"<"},
            {"instance", "*", "name"},
            {"<"},
                {"config", "*", "key", "value"},
            {">"},
        {">"},
    {">"},

    {"custom", "*"},
    {"<"},
        {"numeric", "*",  "name", "size", "data"},
        {"text", "*", "name", "data"},
        {"tuple", "*", "name"},
        {"<"},
            {"element", "*",  "objtype", "objname", "prm"},
        {">"},
    {">"},

    {"asset", "*"},
    {"<"},
        {"mesh", "*", "name", "class", "content_type", "file", "vertex", "normal",
            "texcoord", "face", "refpos", "refquat", "scale", "smoothnormal",
            "maxhullvert", "inertia", "builtin", "params", "material"},
        {"<"},
          {"plugin", "*", "plugin", "instance"},
          {"<"},
            {"config", "*", "key", "value"},
          {">"},
        {">"},
        {"hfield", "*", "name", "content_type", "file", "nrow", "ncol", "size", "elevation"},
        {"skin", "*", "name", "file", "material", "rgba", "inflate",
            "vertex", "texcoord", "face", "group"},
        {"<"},
            {"bone", "*", "body", "bindpos", "bindquat", "vertid", "vertweight"},
        {">"},
        {"texture", "*", "name", "type", "colorspace", "content_type", "file", "gridsize",
            "gridlayout", "fileright", "fileleft", "fileup", "filedown", "filefront", "fileback",
            "builtin", "rgb1", "rgb2", "mark", "markrgb", "random", "width", "height",
            "hflip", "vflip", "nchannel"},
        {"material", "*", "name", "class", "texture",  "texrepeat", "texuniform",
            "emission", "specular", "shininess", "reflectance", "metallic", "roughness", "rgba"},
        {"<"},
            {"layer", "*", "texture", "role"},
        {">"},
        {"model", "*", "name", "file", "content_type"},
    {">"},

    {"body", "R", "name", "childclass", "pos", "quat", "mocap",
        "axisangle", "xyaxes", "zaxis", "euler", "gravcomp", "sleep", "user"},
    {"<"},
        {"inertial", "?", "pos", "quat", "mass", "diaginertia",
            "axisangle", "xyaxes", "zaxis", "euler", "fullinertia"},
        {"joint", "*", "name", "class", "type", "group", "pos", "axis",
            "springdamper", "limited", "actuatorfrclimited",
            "solreflimit", "solimplimit", "solreffriction", "solimpfriction",
            "stiffness", "range", "actuatorfrcrange", "actuatorgravcomp", "margin", "ref",
            "springref", "armature", "damping", "frictionloss", "user"},
        {"freejoint", "*",  "name", "group", "align"},
        {"geom", "*", "name", "class", "type", "contype", "conaffinity", "condim",
            "group", "priority", "size", "material", "friction", "mass", "density",
            "shellinertia", "solmix", "solref", "solimp",
            "margin", "gap", "fromto", "pos", "quat", "axisangle", "xyaxes", "zaxis", "euler",
            "hfield", "mesh", "fitscale", "rgba", "fluidshape", "fluidcoef", "user"},
        {"<"},
            {"plugin", "*", "plugin", "instance"},
            {"<"},
              {"config", "*", "key", "value"},
            {">"},
        {">"},
        {"attach", "*", "model", "body", "prefix"},
        {"site", "*",  "name", "class", "type", "group", "pos", "quat",
            "material", "size", "fromto", "axisangle", "xyaxes", "zaxis", "euler", "rgba", "user"},
        {"camera", "*", "name", "class", "projection", "fovy", "ipd", "resolution", "pos",
            "quat", "axisangle", "xyaxes", "zaxis", "euler", "mode", "target",
            "focal", "focalpixel", "principal", "principalpixel", "sensorsize", "user"},
        {"light", "*", "name", "class", "directional", "type", "castshadow", "active",
            "pos", "dir", "bulbradius", "intensity", "range", "attenuation", "cutoff",
            "exponent", "ambient", "diffuse", "specular", "mode", "target", "texture"},
        {"plugin", "*", "plugin", "instance"},
        {"<"},
          {"config", "*", "key", "value"},
        {">"},
        {"composite", "*", "prefix", "type", "count", "offset",
            "vertex", "initial", "curve", "size", "quat"},
        {"<"},
            {"joint", "*", "kind", "group", "stiffness", "damping", "armature",
                "solreffix", "solimpfix", "type", "axis",
                "limited", "range", "margin", "solreflimit", "solimplimit",
                "frictionloss", "solreffriction", "solimpfriction"},
            {"skin", "?", "texcoord", "material", "group", "rgba", "inflate", "subgrid"},
            {"geom", "?", "type", "contype", "conaffinity", "condim",
                "group", "priority", "size", "material", "rgba", "friction", "mass",
                "density", "solmix", "solref", "solimp", "margin", "gap"},
            {"site", "?", "group", "size", "material", "rgba"},
            {"plugin", "*", "plugin", "instance"},
            {"<"},
              {"config", "*", "key", "value"},
            {">"},
        {">"},
        {"flexcomp", "*", "name", "type", "group", "dim", "dof",
            "count", "spacing", "radius", "rigid", "mass", "inertiabox",
            "scale", "file", "point", "element", "texcoord", "material", "rgba",
            "flatskin", "pos", "quat", "axisangle", "xyaxes", "zaxis", "euler", "origin"},
        {"<"},
            {"edge", "?", "equality", "solref", "solimp", "stiffness", "damping"},
            {"elasticity", "?", "young", "poisson", "damping", "thickness", "elastic2d"},
            {"contact", "?",  "contype", "conaffinity", "condim", "priority",
                "friction", "solmix", "solref", "solimp", "margin", "gap",
                "internal", "selfcollide", "activelayers", "vertcollide", "passive"},
            {"pin", "*", "id", "range", "grid", "gridrange"},
            {"plugin", "*", "plugin", "instance"},
            {"<"},
              {"config", "*", "key", "value"},
            {">"},
        {">"},
    {">"},

    {"deformable", "*"},
    {"<"},
        {"flex", "*", "name", "group", "dim", "radius", "material",
            "rgba", "flatskin", "body", "vertex", "element", "texcoord", "elemtexcoord", "node"},
        {"<"},
            {"contact", "?",  "contype", "conaffinity", "condim", "priority",
                "friction", "solmix", "solref", "solimp", "margin", "gap",
                "internal", "selfcollide", "activelayers", "vertcollide", "passive"},
            {"edge", "?", "stiffness", "damping"},
            {"elasticity", "?", "young", "poisson", "damping", "thickness", "elastic2d"},
        {">"},
        {"skin", "*", "name", "file", "material", "rgba", "inflate",
            "vertex", "texcoord", "face", "group"},
        {"<"},
            {"bone", "*", "body", "bindpos", "bindquat", "vertid", "vertweight"},
        {">"},
    {">"},

    {"contact", "*"},
    {"<"},
        {"pair", "*", "name", "class", "geom1", "geom2", "condim", "friction",
            "solref", "solreffriction", "solimp", "gap", "margin"},
        {"exclude", "*", "name", "body1", "body2"},
    {">"},

    {"equality", "*"},
    {"<"},
        {"connect", "*",  "name", "class", "body1", "body2", "anchor",
            "site1", "site2", "active", "solref", "solimp"},
        {"weld", "*", "name", "class", "body1", "body2", "relpose", "anchor",
            "site1", "site2", "active", "solref", "solimp", "torquescale"},
        {"joint", "*", "name", "class", "joint1", "joint2", "polycoef",
            "active", "solref", "solimp"},
        {"tendon", "*", "name", "class", "tendon1", "tendon2", "polycoef",
            "active", "solref", "solimp"},
        {"flex", "*", "name", "class", "flex",
            "active", "solref", "solimp"},
    {">"},

    {"tendon", "*"},
    {"<"},
        {"spatial", "*", "name", "class", "group", "limited", "actuatorfrclimited", "range",
            "actuatorfrcrange", "solreflimit", "solimplimit", "solreffriction", "solimpfriction",
            "frictionloss", "springlength", "width", "material",
            "margin", "stiffness", "damping", "armature", "rgba", "user"},
        {"<"},
            {"site", "*", "site"},
            {"geom", "*", "geom", "sidesite"},
            {"pulley", "*", "divisor"},
        {">"},
        {"fixed", "*", "name", "class", "group", "limited", "actuatorfrclimited", "range",
            "actuatorfrcrange","solreflimit", "solimplimit", "solreffriction", "solimpfriction",
            "frictionloss", "springlength", "margin", "stiffness", "damping", "armature", "user"},
        {"<"},
            {"joint", "*", "joint", "coef"},
        {">"},
    {">"},

    {"actuator", "*"},
    {"<"},
        {"general", "*", "name", "class", "group",
            "ctrllimited", "forcelimited", "actlimited", "ctrlrange", "forcerange", "actrange",
            "lengthrange", "gear", "cranklength", "user",
            "joint", "jointinparent", "tendon", "slidersite", "cranksite", "site", "refsite",
            "body", "actdim", "dyntype", "gaintype", "biastype", "dynprm", "gainprm", "biasprm",
            "actearly"},
        {"motor", "*", "name", "class", "group",
            "ctrllimited", "forcelimited", "ctrlrange", "forcerange",
            "lengthrange", "gear", "cranklength", "user",
            "joint", "jointinparent", "tendon", "slidersite", "cranksite", "site", "refsite"},
        {"position", "*", "name", "class", "group",
            "ctrllimited", "forcelimited", "ctrlrange", "inheritrange", "forcerange",
            "lengthrange", "gear", "cranklength", "user",
            "joint", "jointinparent", "tendon", "slidersite", "cranksite", "site", "refsite",
            "kp", "kv", "dampratio", "timeconst"},
        {"velocity", "*", "name", "class", "group",
            "ctrllimited", "forcelimited", "ctrlrange", "forcerange",
            "lengthrange", "gear", "cranklength", "user",
            "joint", "jointinparent", "tendon", "slidersite", "cranksite", "site", "refsite",
            "kv"},
        {"intvelocity", "*", "name", "class", "group",
            "ctrllimited", "forcelimited",
            "ctrlrange", "forcerange", "actrange", "inheritrange", "lengthrange",
            "gear", "cranklength", "user",
            "joint", "jointinparent", "tendon", "slidersite", "cranksite", "site", "refsite",
            "kp", "kv", "dampratio"},
        {"damper", "*", "name", "class", "group",
            "forcelimited", "ctrlrange", "forcerange",
            "lengthrange", "gear", "cranklength", "user",
            "joint", "jointinparent", "tendon", "slidersite", "cranksite", "site", "refsite",
            "kv"},
        {"cylinder", "*", "name", "class", "group",
            "ctrllimited", "forcelimited", "ctrlrange", "forcerange",
            "lengthrange", "gear", "cranklength", "user",
            "joint", "jointinparent", "tendon", "slidersite", "cranksite", "site", "refsite",
            "timeconst", "area", "diameter", "bias"},
        {"muscle", "*",  "name", "class", "group",
            "ctrllimited", "forcelimited", "ctrlrange", "forcerange",
            "lengthrange", "gear", "cranklength", "user",
            "joint", "jointinparent", "tendon", "slidersite", "cranksite",
            "timeconst", "tausmooth", "range", "force", "scale",
            "lmin", "lmax", "vmax", "fpmax", "fvmax"},
        {"adhesion", "*", "name", "class", "group",
            "forcelimited", "ctrlrange", "forcerange", "user", "body", "gain"},
        {"plugin", "*", "name", "class",  "plugin", "instance", "group",
            "ctrllimited", "forcelimited", "actlimited", "ctrlrange", "forcerange", "actrange",
            "lengthrange", "gear", "cranklength", "joint", "jointinparent",
            "site", "actdim", "dyntype", "dynprm", "tendon", "cranksite", "slidersite", "user",
            "actearly"},
        {"<"},
          {"config", "*", "key", "value"},
        {">"},
    {">"},

    {"sensor", "*"},
    {"<"},
        {"touch", "*", "name", "site", "cutoff", "noise", "user"},
        {"accelerometer", "*", "name", "site", "cutoff", "noise", "user"},
        {"velocimeter", "*", "name", "site", "cutoff", "noise", "user"},
        {"gyro", "*", "name", "site", "cutoff", "noise", "user"},
        {"force", "*", "name", "site", "cutoff", "noise", "user"},
        {"torque", "*", "name", "site", "cutoff", "noise", "user"},
        {"magnetometer", "*", "name", "site", "cutoff", "noise", "user"},
        {"camprojection", "*", "name", "site", "camera", "cutoff", "noise", "user"},
        {"rangefinder", "*", "name", "site", "cutoff", "noise", "user"},
        {"jointpos", "*", "name", "joint", "cutoff", "noise", "user"},
        {"jointvel", "*", "name", "joint", "cutoff", "noise", "user"},
        {"tendonpos", "*", "name", "tendon", "cutoff", "noise", "user"},
        {"tendonvel", "*", "name", "tendon", "cutoff", "noise", "user"},
        {"actuatorpos", "*", "name", "actuator", "cutoff", "noise", "user"},
        {"actuatorvel", "*", "name", "actuator", "cutoff", "noise", "user"},
        {"actuatorfrc", "*", "name", "actuator", "cutoff", "noise", "user"},
        {"jointactuatorfrc", "*", "name", "joint", "cutoff", "noise", "user"},
        {"tendonactuatorfrc", "*", "name", "tendon", "cutoff", "noise", "user"},
        {"ballquat", "*", "name", "joint", "cutoff", "noise", "user"},
        {"ballangvel", "*", "name", "joint", "cutoff", "noise", "user"},
        {"jointlimitpos", "*", "name", "joint", "cutoff", "noise", "user"},
        {"jointlimitvel", "*", "name", "joint", "cutoff", "noise", "user"},
        {"jointlimitfrc", "*", "name", "joint", "cutoff", "noise", "user"},
        {"tendonlimitpos", "*", "name", "tendon", "cutoff", "noise", "user"},
        {"tendonlimitvel", "*", "name", "tendon", "cutoff", "noise", "user"},
        {"tendonlimitfrc", "*", "name", "tendon", "cutoff", "noise", "user"},
        {"framepos", "*", "name", "objtype", "objname", "reftype", "refname", "cutoff", "noise", "user"},
        {"framequat", "*", "name", "objtype", "objname", "reftype", "refname", "cutoff", "noise", "user"},
        {"framexaxis", "*", "name", "objtype", "objname", "reftype", "refname", "cutoff", "noise", "user"},
        {"frameyaxis", "*", "name", "objtype", "objname", "reftype", "refname", "cutoff", "noise", "user"},
        {"framezaxis", "*", "name", "objtype", "objname", "reftype", "refname", "cutoff", "noise", "user"},
        {"framelinvel", "*", "name", "objtype", "objname", "reftype", "refname", "cutoff", "noise", "user"},
        {"frameangvel", "*", "name", "objtype", "objname", "reftype", "refname", "cutoff", "noise", "user"},
        {"framelinacc", "*", "name", "objtype", "objname", "cutoff", "noise", "user"},
        {"frameangacc", "*", "name", "objtype", "objname", "cutoff", "noise", "user"},
        {"subtreecom", "*", "name", "body", "cutoff", "noise", "user"},
        {"subtreelinvel", "*", "name", "body", "cutoff", "noise", "user"},
        {"subtreeangmom", "*", "name", "body", "cutoff", "noise", "user"},
        {"insidesite", "*", "name", "site", "objtype", "objname", "cutoff", "noise", "user"},
        {"distance", "*", "name", "geom1", "geom2", "body1", "body2", "cutoff", "noise", "user"},
        {"normal", "*", "name", "geom1", "geom2", "body1", "body2", "cutoff", "noise", "user"},
        {"fromto", "*", "name", "geom1", "geom2", "body1", "body2", "cutoff", "noise", "user"},
        {"contact", "*", "name", "geom1", "geom2", "body1", "body2", "subtree1", "subtree2", "site",
            "num", "data", "reduce", "cutoff", "noise", "user"},
        {"e_potential", "*", "name", "cutoff", "noise", "user"},
        {"e_kinetic", "*", "name", "cutoff", "noise", "user"},
        {"clock", "*", "name", "cutoff", "noise", "user"},
        {"user", "*", "name", "objtype", "objname", "datatype", "needstage",
            "dim", "cutoff", "noise", "user"},
    {"tactile", "*", "name", "geom", "mesh", "user"},
        {"plugin", "*", "name", "plugin", "instance", "cutoff", "objtype", "objname", "reftype", "refname",
            "user"},
        {"<"},
          {"config", "*", "key", "value"},
        {">"},
    {">"},

    {"keyframe", "*"},
    {"<"},
        {"key", "*", "name", "time", "qpos", "qvel", "act", "mpos", "mquat", "ctrl"},
    {">"},
{">"}
};



//---------------------------------- MJCF keywords used in attributes ------------------------------

// coordinate type
const mjMap coordinate_map[2] = {
  {"local",   0},
  {"global",  1}
};


// angle type
const mjMap angle_map[2] = {
  {"radian",  0},
  {"degree",  1}
};


// bool type
const mjMap bool_map[2] = {
  {"false",   0},
  {"true",    1}
};


// fluidshape type
const mjMap fluid_map[2] = {
  {"none",      0},
  {"ellipsoid", 1}
};


// enable type
const mjMap enable_map[2] = {
  {"disable", 0},
  {"enable",  1}
};


// TFAuto type
const mjMap TFAuto_map[3] = {
  {"false",   0},
  {"true",    1},
  {"auto",    2}
};


// body sleep type
const int bodysleep_sz = 4;
const mjMap bodysleep_map[bodysleep_sz] = {
  {"auto",          mjSLEEP_AUTO},
  {"never",         mjSLEEP_NEVER},
  {"allowed",       mjSLEEP_ALLOWED},
  {"init",          mjSLEEP_INIT}
};

// joint type
const int joint_sz = 4;
const mjMap joint_map[joint_sz] = {
  {"free",          mjJNT_FREE},
  {"ball",          mjJNT_BALL},
  {"slide",         mjJNT_SLIDE},
  {"hinge",         mjJNT_HINGE}
};


// geom type
const mjMap geom_map[mjNGEOMTYPES] = {
  {"plane",         mjGEOM_PLANE},
  {"hfield",        mjGEOM_HFIELD},
  {"sphere",        mjGEOM_SPHERE},
  {"capsule",       mjGEOM_CAPSULE},
  {"ellipsoid",     mjGEOM_ELLIPSOID},
  {"cylinder",      mjGEOM_CYLINDER},
  {"box",           mjGEOM_BOX},
  {"mesh",          mjGEOM_MESH},
  {"sdf",           mjGEOM_SDF}
};


// projection type
const int projection_sz = 2;
const mjMap projection_map[projection_sz] = {
  {"perspective",   mjPROJ_PERSPECTIVE},
  {"orthographic",  mjPROJ_ORTHOGRAPHIC}
};

// camlight type
const int camlight_sz = 5;
const mjMap camlight_map[camlight_sz] = {
  {"fixed",         mjCAMLIGHT_FIXED},
  {"track",         mjCAMLIGHT_TRACK},
  {"trackcom",      mjCAMLIGHT_TRACKCOM},
  {"targetbody",    mjCAMLIGHT_TARGETBODY},
  {"targetbodycom", mjCAMLIGHT_TARGETBODYCOM}
};


// light type
const int lighttype_sz = 4;
const mjMap lighttype_map[lighttype_sz] = {
  {"spot",          mjLIGHT_SPOT},
  {"directional",   mjLIGHT_DIRECTIONAL},
  {"point",         mjLIGHT_POINT},
  {"image",         mjLIGHT_IMAGE}
};


// texmat role type
const int texrole_sz = mjNTEXROLE - 1;
const mjMap texrole_map[texrole_sz] = {
  {"rgb",           mjTEXROLE_RGB},
  {"occlusion",     mjTEXROLE_OCCLUSION},
  {"roughness",     mjTEXROLE_ROUGHNESS},
  {"metallic",      mjTEXROLE_METALLIC},
  {"normal",        mjTEXROLE_NORMAL},
  {"opacity",       mjTEXROLE_OPACITY},
  {"emissive",      mjTEXROLE_EMISSIVE},
  {"rgba",          mjTEXROLE_RGBA},
  {"orm",           mjTEXROLE_ORM},
};


// integrator type
const int integrator_sz = 4;
const mjMap integrator_map[integrator_sz] = {
  {"Euler",         mjINT_EULER},
  {"RK4",           mjINT_RK4},
  {"implicit",      mjINT_IMPLICIT},
  {"implicitfast",  mjINT_IMPLICITFAST}
};


// cone type
const int cone_sz = 2;
const mjMap cone_map[cone_sz] = {
  {"pyramidal",     mjCONE_PYRAMIDAL},
  {"elliptic",      mjCONE_ELLIPTIC}
};


// Jacobian type
const int jac_sz = 3;
const mjMap jac_map[jac_sz] = {
  {"dense",         mjJAC_DENSE},
  {"sparse",        mjJAC_SPARSE},
  {"auto",          mjJAC_AUTO}
};


// solver type
const int solver_sz = 3;
const mjMap solver_map[solver_sz] = {
  {"PGS",           mjSOL_PGS},
  {"CG",            mjSOL_CG},
  {"Newton",        mjSOL_NEWTON}
};


// constraint type
const int equality_sz = 6;
const mjMap equality_map[equality_sz] = {
  {"connect",       mjEQ_CONNECT},
  {"weld",          mjEQ_WELD},
  {"joint",         mjEQ_JOINT},
  {"tendon",        mjEQ_TENDON},
  {"flex",          mjEQ_FLEX},
  {"distance",      mjEQ_DISTANCE}
};


// type for texture
const int texture_sz = 3;
const mjMap texture_map[texture_sz] = {
  {"2d",            mjTEXTURE_2D},
  {"cube",          mjTEXTURE_CUBE},
  {"skybox",        mjTEXTURE_SKYBOX}
};


// colorspace for texture
const int colorspace_sz = 3;
const mjMap colorspace_map[colorspace_sz] = {
  {"auto",          mjCOLORSPACE_AUTO},
  {"linear",        mjCOLORSPACE_LINEAR},
  {"sRGB",          mjCOLORSPACE_SRGB}
};


// builtin type for texture
const int builtin_sz = 4;
const mjMap builtin_map[builtin_sz] = {
  {"none",          mjBUILTIN_NONE},
  {"gradient",      mjBUILTIN_GRADIENT},
  {"checker",       mjBUILTIN_CHECKER},
  {"flat",          mjBUILTIN_FLAT}
};


// mark type for texture
const int mark_sz = 4;
const mjMap mark_map[mark_sz] = {
  {"none",          mjMARK_NONE},
  {"edge",          mjMARK_EDGE},
  {"cross",         mjMARK_CROSS},
  {"random",        mjMARK_RANDOM}
};


// dyn type
const int dyn_sz = 6;
const mjMap dyn_map[dyn_sz] = {
  {"none",          mjDYN_NONE},
  {"integrator",    mjDYN_INTEGRATOR},
  {"filter",        mjDYN_FILTER},
  {"filterexact",   mjDYN_FILTEREXACT},
  {"muscle",        mjDYN_MUSCLE},
  {"user",          mjDYN_USER}
};


// gain type
const int gain_sz = 4;
const mjMap gain_map[gain_sz] = {
  {"fixed",         mjGAIN_FIXED},
  {"affine",        mjGAIN_AFFINE},
  {"muscle",        mjGAIN_MUSCLE},
  {"user",          mjGAIN_USER}
};


// bias type
const int bias_sz = 4;
const mjMap bias_map[bias_sz] = {
  {"none",          mjBIAS_NONE},
  {"affine",        mjBIAS_AFFINE},
  {"muscle",        mjBIAS_MUSCLE},
  {"user",          mjBIAS_USER}
};


// stage type
const int stage_sz = 4;
const mjMap stage_map[stage_sz] = {
  {"none",          mjSTAGE_NONE},
  {"pos",           mjSTAGE_POS},
  {"vel",           mjSTAGE_VEL},
  {"acc",           mjSTAGE_ACC}
};


// data type
const int datatype_sz = 4;
const mjMap datatype_map[datatype_sz] = {
  {"real",          mjDATATYPE_REAL},
  {"positive",      mjDATATYPE_POSITIVE},
  {"axis",          mjDATATYPE_AXIS},
  {"quaternion",    mjDATATYPE_QUATERNION}
};


// contact data type
const mjMap condata_map[mjNCONDATA] = {
  {"found",         mjCONDATA_FOUND},
  {"force",         mjCONDATA_FORCE},
  {"torque",        mjCONDATA_TORQUE},
  {"dist",          mjCONDATA_DIST},
  {"pos",           mjCONDATA_POS},
  {"normal",        mjCONDATA_NORMAL},
  {"tangent",       mjCONDATA_TANGENT}
};


// contact reduction type
const int reduce_sz = 4;
const mjMap reduce_map[reduce_sz] = {
  {"none",          0},
  {"mindist",       1},
  {"maxforce",      2},
  {"netforce",      3}
};


// LR mode
const int lrmode_sz = 4;
const mjMap lrmode_map[lrmode_sz] = {
  {"none",          mjLRMODE_NONE},
  {"muscle",        mjLRMODE_MUSCLE},
  {"muscleuser",    mjLRMODE_MUSCLEUSER},
  {"all",           mjLRMODE_ALL}
};


// composite type
const mjMap comp_map[mjNCOMPTYPES] = {
  {"particle",      mjCOMPTYPE_PARTICLE},
  {"grid",          mjCOMPTYPE_GRID},
  {"rope",          mjCOMPTYPE_ROPE},
  {"loop",          mjCOMPTYPE_LOOP},
  {"cable",         mjCOMPTYPE_CABLE},
  {"cloth",         mjCOMPTYPE_CLOTH}
};


// composite joint kind
const mjMap jkind_map[1] = {
  {"main",          mjCOMPKIND_JOINT}
};


// composite rope shape
const mjMap shape_map[mjNCOMPSHAPES] = {
  {"s",             mjCOMPSHAPE_LINE},
  {"cos(s)",        mjCOMPSHAPE_COS},
  {"sin(s)",        mjCOMPSHAPE_SIN},
  {"0",             mjCOMPSHAPE_ZERO}
};


// mesh type
const mjMap meshtype_map[2] = {
  {"false",         mjINERTIA_VOLUME},
  {"true",          mjINERTIA_SHELL},
};


// mesh inertia type
const mjMap meshinertia_map[4] = {
  {"convex",        mjMESH_INERTIA_CONVEX},
  {"legacy",        mjMESH_INERTIA_LEGACY},
  {"exact",         mjMESH_INERTIA_EXACT},
  {"shell",         mjMESH_INERTIA_SHELL}
};


// mesh builtin type
const int meshbuiltin_sz = 8;
const mjMap meshbuiltin_map[meshbuiltin_sz] = {
  {"none",          mjMESH_BUILTIN_NONE},
  {"sphere",        mjMESH_BUILTIN_SPHERE},
  {"hemisphere",    mjMESH_BUILTIN_HEMISPHERE},
  {"cone",          mjMESH_BUILTIN_CONE},
  {"supertorus",    mjMESH_BUILTIN_SUPERTORUS},
  {"supersphere",   mjMESH_BUILTIN_SUPERSPHERE},
  {"wedge",         mjMESH_BUILTIN_WEDGE},
  {"plate",         mjMESH_BUILTIN_PLATE}
};


// flexcomp type
const mjMap fcomp_map[mjNFCOMPTYPES] = {
  {"grid",          mjFCOMPTYPE_GRID},
  {"box",           mjFCOMPTYPE_BOX},
  {"cylinder",      mjFCOMPTYPE_CYLINDER},
  {"ellipsoid",     mjFCOMPTYPE_ELLIPSOID},
  {"square",        mjFCOMPTYPE_SQUARE},
  {"disc",          mjFCOMPTYPE_DISC},
  {"circle",        mjFCOMPTYPE_CIRCLE},
  {"mesh",          mjFCOMPTYPE_MESH},
  {"gmsh",          mjFCOMPTYPE_GMSH},
  {"direct",        mjFCOMPTYPE_DIRECT}
};


// flexcomp dof type
const mjMap fdof_map[mjNFCOMPDOFS] = {
  {"full",          mjFCOMPDOF_FULL},
  {"radial",        mjFCOMPDOF_RADIAL},
  {"trilinear",     mjFCOMPDOF_TRILINEAR},
  {"quadratic",     mjFCOMPDOF_QUADRATIC}
};


// flex selfcollide type
const mjMap flexself_map[5] = {
  {"none",          mjFLEXSELF_NONE},
  {"narrow",        mjFLEXSELF_NARROW},
  {"bvh",           mjFLEXSELF_BVH},
  {"sap",           mjFLEXSELF_SAP},
  {"auto",          mjFLEXSELF_AUTO},
};


// flex elastic 2d type
const mjMap elastic2d_map[5] = {
  {"none",          0},
  {"bend",          1},
  {"stretch",       2},
  {"both",          3},
};



//---------------------------------- class mjXReader implementation --------------------------------

// constructor
mjXReader::mjXReader() : schema(MJCF, nMJCF) {
  readingdefaults = false;
}



// print schema
void mjXReader::PrintSchema(std::stringstream& str, bool html, bool pad) {
  if (html) {
    schema.PrintHTML(str, 0, pad);
  } else {
    schema.Print(str, 0);
  }
}



// main entry point for XML parser
//  mjCModel is allocated here; caller is responsible for deallocation
void mjXReader::Parse(XMLElement* root, const mjVFS* vfs) {
  // check schema
  if (!schema.GetError().empty()) {
    throw mjXError(0, "XML Schema Construction Error: %s", schema.GetError().c_str());
  }

  // validate
  XMLElement* bad = 0;
  if ((bad = schema.Check(root, 0))) {
    throw mjXError(bad, "Schema violation: %s", schema.GetError().c_str());
  }

  // get model name
  string modelname;
  if (ReadAttrTxt(root, "model", modelname)) {
    mjs_setString(spec->modelname, modelname.c_str());
  }

  // get comment
  if (root->FirstChild() && root->FirstChild()->ToComment()) {
    mjs_setString(spec->comment, root->FirstChild()->Value());
  } else {
    mjs_setString(spec->comment, "");
  }

  //------------------- parse MuJoCo sections embedded in all XML formats

  for (XMLElement* section = FirstChildElement(root, "compiler"); section;
       section = NextSiblingElement(section, "compiler")) {
    Compiler(section, spec);
  }

  for (XMLElement* section = FirstChildElement(root, "option"); section;
       section = NextSiblingElement(section, "option")) {
    Option(section, &spec->option);
  }

  for (XMLElement* section = FirstChildElement(root, "size"); section;
       section = NextSiblingElement(section, "size")) {
    Size(section, spec);
  }

  //------------------ parse MJCF-specific sections

  for (XMLElement* section = FirstChildElement(root, "visual"); section;
       section = NextSiblingElement(section, "visual")) {
    Visual(section);
  }

  for (XMLElement* section = FirstChildElement(root, "statistic"); section;
       section = NextSiblingElement(section, "statistic")) {
    Statistic(section);
  }

  readingdefaults = true;
  for (XMLElement* section = FirstChildElement(root, "default"); section;
       section = NextSiblingElement(section, "default")) {
    Default(section, nullptr, vfs);
  }
  readingdefaults = false;

  for (XMLElement* section = FirstChildElement(root, "extension"); section;
       section = NextSiblingElement(section, "extension")) {
    Extension(section);
  }

  for (XMLElement* section = FirstChildElement(root, "custom"); section;
       section = NextSiblingElement(section, "custom")) {
    Custom(section);
  }

  for (XMLElement* section = FirstChildElement(root, "asset"); section;
       section = NextSiblingElement(section, "asset")) {
    Asset(section, vfs);
  }

  for (XMLElement* section = FirstChildElement(root, "contact"); section;
       section = NextSiblingElement(section, "contact")) {
    Contact(section);
  }

  for (XMLElement* section = FirstChildElement(root, "deformable"); section;
       section = NextSiblingElement(section, "deformable")) {
    Deformable(section, vfs);
  }

  for (XMLElement* section = FirstChildElement(root, "equality"); section;
       section = NextSiblingElement(section, "equality")) {
    Equality(section);
  }

  for (XMLElement* section = FirstChildElement(root, "tendon"); section;
       section = NextSiblingElement(section, "tendon")) {
    Tendon(section);
  }

  for (XMLElement* section = FirstChildElement(root, "actuator"); section;
       section = NextSiblingElement(section, "actuator")) {
    Actuator(section);
  }

  for (XMLElement* section = FirstChildElement(root, "sensor"); section;
       section = NextSiblingElement(section, "sensor")) {
    Sensor(section);
  }

  for (XMLElement* section = FirstChildElement(root, "keyframe"); section;
       section = NextSiblingElement(section, "keyframe")) {
    Keyframe(section);
  }

  // set deepcopy flag to true to copy child specs during attach calls
  mjs_setDeepCopy(spec, true);

  for (XMLElement* section = FirstChildElement(root, "worldbody"); section;
       section = NextSiblingElement(section, "worldbody")) {
    Body(section, mjs_findBody(spec, "world"), nullptr, vfs);
  }

  // set deepcopy flag to false to disable copying during attach in all future calls
  mjs_setDeepCopy(spec, false);
}



// compiler section parser
void mjXReader::Compiler(XMLElement* section, mjSpec* s) {
  string text;
  int n;

  // top-level attributes
  if (MapValue(section, "autolimits", &n, bool_map, 2)) {
    s->compiler.autolimits = (n == 1);
  }
  ReadAttr(section, "boundmass", 1, &s->compiler.boundmass, text);
  ReadAttr(section, "boundinertia", 1, &s->compiler.boundinertia, text);
  ReadAttr(section, "settotalmass", 1, &s->compiler.settotalmass, text);
  if (MapValue(section, "balanceinertia", &n, bool_map, 2)) {
    s->compiler.balanceinertia = (n == 1);
  }
  if (MapValue(section, "strippath", &n, bool_map, 2)) {
    s->strippath = (n == 1);
  }
  if (MapValue(section, "fitaabb", &n, bool_map, 2)) {
    s->compiler.fitaabb = (n == 1);
  }
  if (MapValue(section, "coordinate", &n, coordinate_map, 2)) {
    if (n == 1) {
      throw mjXError(section, "global coordinates no longer supported. To convert existing models, "
                     "load and save them in MuJoCo 2.3.3 or older");
    }
  }
  if (MapValue(section, "angle", &n, angle_map, 2)) {
    s->compiler.degree = (n == 1);
  }
  if (ReadAttrTxt(section, "eulerseq", text)) {
    if (text.size() != 3) {
      throw mjXError(section, "euler format must have length 3");
    }
    memcpy(s->compiler.eulerseq, text.c_str(), 3);
  }
  if (ReadAttrTxt(section, "assetdir", text)) {
    mjs_setString(s->compiler.meshdir, text.c_str());
    mjs_setString(s->compiler.texturedir, text.c_str());
  }
  // meshdir and texturedir take precedence over assetdir
  string meshdir, texturedir;
  if (ReadAttrTxt(section, "meshdir", meshdir)) {
    mjs_setString(s->compiler.meshdir, meshdir.c_str());
  };
  if (ReadAttrTxt(section, "texturedir", texturedir)) {
    mjs_setString(s->compiler.texturedir, texturedir.c_str());
  }
  if (MapValue(section, "discardvisual", &n, bool_map, 2)) {
    s->compiler.discardvisual = (n == 1);
  }
  if (MapValue(section, "usethread", &n, bool_map, 2)) {
    s->compiler.usethread = (n == 1);
  }
  if (MapValue(section, "fusestatic", &n, bool_map, 2)) {
    s->compiler.fusestatic = (n == 1);
  }
  MapValue(section, "inertiafromgeom", &s->compiler.inertiafromgeom, TFAuto_map, 3);
  ReadAttr(section, "inertiagrouprange", 2, s->compiler.inertiagrouprange, text);
  if (MapValue(section, "alignfree", &n, bool_map, 2)) {
    s->compiler.alignfree = (n == 1);
  }
  if (MapValue(section, "saveinertial", &n, bool_map, 2)) {
    s->compiler.saveinertial = (n == 1);
  }

  // lengthrange subelement
  XMLElement* elem = FindSubElem(section, "lengthrange");
  if (elem) {
    mjLROpt* opt = &(s->compiler.LRopt);

    // flags
    MapValue(elem, "mode", &opt->mode, lrmode_map, lrmode_sz);
    if (MapValue(elem, "useexisting", &n, bool_map, 2)) {
      opt->useexisting = (n == 1);
    }
    if (MapValue(elem, "uselimit", &n, bool_map, 2)) {
      opt->uselimit = (n == 1);
    }

    // algorithm parameters
    ReadAttr(elem, "accel", 1, &opt->accel, text);
    ReadAttr(elem, "maxforce", 1, &opt->maxforce, text);
    ReadAttr(elem, "timeconst", 1, &opt->timeconst, text);
    ReadAttr(elem, "timestep", 1, &opt->timestep, text);
    ReadAttr(elem, "inttotal", 1, &opt->inttotal, text);
    ReadAttr(elem, "interval", 1, &opt->interval, text);
    ReadAttr(elem, "tolrange", 1, &opt->tolrange, text);
  }
}



// option section parser
void mjXReader::Option(XMLElement* section, mjOption* opt) {
  string text;
  int n;

  // read options
  ReadAttr(section, "timestep", 1, &opt->timestep, text);
  ReadAttr(section, "impratio", 1, &opt->impratio, text);
  ReadAttr(section, "tolerance", 1, &opt->tolerance, text);
  ReadAttr(section, "ls_tolerance", 1, &opt->ls_tolerance, text);
  ReadAttr(section, "noslip_tolerance", 1, &opt->noslip_tolerance, text);
  ReadAttr(section, "ccd_tolerance", 1, &opt->ccd_tolerance, text);
  ReadAttr(section, "sleep_tolerance", 1, &opt->sleep_tolerance, text);
  ReadAttr(section, "gravity", 3, opt->gravity, text);
  ReadAttr(section, "wind", 3, opt->wind, text);
  ReadAttr(section, "magnetic", 3, opt->magnetic, text);
  ReadAttr(section, "density", 1, &opt->density, text);
  ReadAttr(section, "viscosity", 1, &opt->viscosity, text);

  ReadAttr(section, "o_margin", 1, &opt->o_margin, text);
  ReadAttr(section, "o_solref", mjNREF, opt->o_solref, text, false, false);
  ReadAttr(section, "o_solimp", mjNIMP, opt->o_solimp, text, false, false);
  ReadAttr(section, "o_friction", 5, opt->o_friction, text, false, false);

  MapValue(section, "integrator", &opt->integrator, integrator_map, integrator_sz);
  MapValue(section, "cone", &opt->cone, cone_map, cone_sz);
  MapValue(section, "jacobian", &opt->jacobian, jac_map, jac_sz);
  MapValue(section, "solver", &opt->solver, solver_map, solver_sz);
  ReadAttrInt(section, "iterations", &opt->iterations);
  ReadAttrInt(section, "ls_iterations", &opt->ls_iterations);
  ReadAttrInt(section, "noslip_iterations", &opt->noslip_iterations);
  ReadAttrInt(section, "ccd_iterations", &opt->ccd_iterations);
  ReadAttrInt(section, "sdf_iterations", &opt->sdf_iterations);
  ReadAttrInt(section, "sdf_initpoints", &opt->sdf_initpoints);

  // actuatorgroupdisable
  constexpr int num_bitflags = 31;
  int disabled_act_groups[num_bitflags];
  int num_found = ReadAttr(section, "actuatorgroupdisable", num_bitflags, disabled_act_groups,
                           text, false, false);
  for (int i=0; i < num_found; i++) {
    int group = disabled_act_groups[i];
    if (group < 0) {
      throw mjXError(section, "disabled actuator group value must be non-negative");
    }
    if (group > num_bitflags - 1) {
      throw mjXError(section, "disabled actuator group value cannot exceed 30");
    }
    opt->disableactuator |= (1 << group);
  }

  // read disable sub-element
  XMLElement* elem = FindSubElem(section, "flag");
  if (elem) {
#define READDSBL(NAME, MASK) \
        if (MapValue(elem, NAME, &n, enable_map, 2)) { \
            opt->disableflags ^= (opt->disableflags & MASK); \
            opt->disableflags |= (n ? 0 : MASK); }

    READDSBL("constraint",   mjDSBL_CONSTRAINT)
    READDSBL("equality",     mjDSBL_EQUALITY)
    READDSBL("frictionloss", mjDSBL_FRICTIONLOSS)
    READDSBL("limit",        mjDSBL_LIMIT)
    READDSBL("contact",      mjDSBL_CONTACT)
    READDSBL("spring",       mjDSBL_SPRING)
    READDSBL("damper",       mjDSBL_DAMPER)
    READDSBL("gravity",      mjDSBL_GRAVITY)
    READDSBL("clampctrl",    mjDSBL_CLAMPCTRL)
    READDSBL("warmstart",    mjDSBL_WARMSTART)
    READDSBL("filterparent", mjDSBL_FILTERPARENT)
    READDSBL("actuation",    mjDSBL_ACTUATION)
    READDSBL("refsafe",      mjDSBL_REFSAFE)
    READDSBL("sensor",       mjDSBL_SENSOR)
    READDSBL("midphase",     mjDSBL_MIDPHASE)
    READDSBL("eulerdamp",    mjDSBL_EULERDAMP)
    READDSBL("autoreset",    mjDSBL_AUTORESET)
    READDSBL("nativeccd",    mjDSBL_NATIVECCD)
    READDSBL("island",       mjDSBL_ISLAND)
#undef READDSBL

#define READENBL(NAME, MASK) \
        if (MapValue(elem, NAME, &n, enable_map, 2)) { \
            opt->enableflags ^= (opt->enableflags & MASK); \
            opt->enableflags |= (n ? MASK : 0); }

    READENBL("override",    mjENBL_OVERRIDE)
    READENBL("energy",      mjENBL_ENERGY)
    READENBL("fwdinv",      mjENBL_FWDINV)
    READENBL("invdiscrete", mjENBL_INVDISCRETE)
    READENBL("multiccd",    mjENBL_MULTICCD)
    READENBL("sleep",       mjENBL_SLEEP)
#undef READENBL
  }
}



// size section parser
void mjXReader::Size(XMLElement* section, mjSpec* s) {
  // read memory bytes
  {
    constexpr char err_msg[] =
      "unsigned integer with an optional suffix {K,M,G,T,P,E} is expected in "
      "attribute 'memory' (or the size specified is too big)";

    auto memory = [&]() -> std::optional<std::size_t> {
      const char* pstr = section->Attribute("memory");
      if (!pstr) {
        return std::nullopt;
      }

      // trim entire string
      string trimmed;
      {
        std::istringstream strm((string(pstr)));
        strm >> trimmed;
        string trailing;
        strm >> trailing;
        if (!trailing.empty() || !strm.eof()) {
          throw mjXError(section, "%s", err_msg);
        }

        // allow explicit specification of the default "-1" value
        if (trimmed == "-1") {
          return std::nullopt;
        }
      }

      std::istringstream strm(trimmed);

      // check that the number is not negative
      if (strm.peek() == '-') {
        throw mjXError(section, "%s", err_msg);
      }

      std::size_t base_size;
      strm >> base_size;
      if (strm.fail()) {
        // either not an integer or the number without the suffix is already bigger than size_t
        throw mjXError(section, "%s", err_msg);
      }

      // parse the multiplier suffix
      int multiplier_bit = 0;
      if (!strm.eof()) {
        char suffix = strm.get();
        if (suffix == 'K' || suffix == 'k') {
          multiplier_bit = 10;
        } else if (suffix == 'M' || suffix == 'm') {
          multiplier_bit = 20;
        } else if (suffix == 'G' || suffix == 'g') {
          multiplier_bit = 30;
        } else if (suffix == 'T' || suffix == 't') {
          multiplier_bit = 40;
        } else if (suffix == 'P' || suffix == 'p') {
          multiplier_bit = 50;
        } else if (suffix == 'E' || suffix == 'e') {
          multiplier_bit = 60;
        }

        // check for invalid suffix, or suffix longer than one character
        strm.get();
        if (!multiplier_bit || !strm.eof()) {
          throw mjXError(section, "%s", err_msg);
        }
      }

      // check that the specified suffix isn't bigger than size_t
      if (multiplier_bit + 1 > std::numeric_limits<std::size_t>::digits) {
        throw mjXError(section, "%s", err_msg);
      }

      // check that the suffix won't take the total size beyond size_t
      const std::size_t max_base_size =
          (std::numeric_limits<std::size_t>::max() << multiplier_bit) >> multiplier_bit;
      if (base_size > max_base_size) {
        throw mjXError(section, "%s", err_msg);
      }

      const std::size_t total_size = base_size << multiplier_bit;
      return total_size;
    }();

    if (memory.has_value()) {
      if (*memory / sizeof(mjtNum) > std::numeric_limits<std::size_t>::max()) {
        throw mjXError(section, "%s", err_msg);
      }
      s->memory = *memory;
    }
  }

  // read sizes
  ReadAttrInt(section, "nuserdata", &s->nuserdata);
  ReadAttrInt(section, "nkey", &s->nkey);

  ReadAttrInt(section, "nconmax", &s->nconmax);
  if (s->nconmax < -1) throw mjXError(section, "nconmax must be >= -1");

  {
    int nstack = -1;
    const bool has_nstack = ReadAttrInt(section, "nstack", &nstack);
    if (has_nstack) {
      if (s->nstack < -1) {
        throw mjXError(section, "nstack must be >= -1");
      }
      if (s->memory != -1 && nstack != -1) {
        throw mjXError(section,
                       "either 'memory' and 'nstack' attribute can be specified, not both");
      }
      s->nstack = nstack;
    }
  }
  {
    int njmax = -1;
    const bool has_njmax = ReadAttrInt(section, "njmax", &njmax);
    if (has_njmax) {
      if (s->njmax < -1) {
        throw mjXError(section, "njmax must be >= -1");
      }
      if (s->memory != -1 && njmax != -1) {
        throw mjXError(section,
                       "either 'memory' and 'njmax' attribute can be specified, not both");
      }
      s->njmax = njmax;
    }
  }

  ReadAttrInt(section, "nuser_body", &s->nuser_body);
  if (s->nuser_body < -1) throw mjXError(section, "nuser_body must be >= -1");

  ReadAttrInt(section, "nuser_jnt", &s->nuser_jnt);
  if (s->nuser_jnt < -1) throw mjXError(section, "nuser_jnt must be >= -1");

  ReadAttrInt(section, "nuser_geom", &s->nuser_geom);
  if (s->nuser_geom < -1) throw mjXError(section, "nuser_geom must be >= -1");

  ReadAttrInt(section, "nuser_site", &s->nuser_site);
  if (s->nuser_site < -1) throw mjXError(section, "nuser_site must be >= -1");

  ReadAttrInt(section, "nuser_cam", &s->nuser_cam);
  if (s->nuser_cam < -1) throw mjXError(section, "nuser_cam must be >= -1");

  ReadAttrInt(section, "nuser_tendon", &s->nuser_tendon);
  if (s->nuser_tendon < -1) throw mjXError(section, "nuser_tendon must be >= -1");

  ReadAttrInt(section, "nuser_actuator", &s->nuser_actuator);
  if (s->nuser_actuator < -1) throw mjXError(section, "nuser_actuator must be >= -1");

  ReadAttrInt(section, "nuser_sensor", &s->nuser_sensor);
  if (s->nuser_sensor < -1) throw mjXError(section, "nuser_sensor must be >= -1");
}



// statistic section parser
void mjXReader::Statistic(XMLElement* section) {
  string text;

  // read statistics
  ReadAttr(section, "meaninertia", 1, &spec->stat.meaninertia, text);
  ReadAttr(section, "meanmass", 1, &spec->stat.meanmass, text);
  ReadAttr(section, "meansize", 1, &spec->stat.meansize, text);
  ReadAttr(section, "extent", 1, &spec->stat.extent, text);
  if (mjuu_defined(spec->stat.extent) && spec->stat.extent <= 0) {
    throw mjXError(section, "extent must be strictly positive");
  }
  ReadAttr(section, "center", 3, spec->stat.center, text);
}



//---------------------------------- one-element parsers -------------------------------------------

// flex element parser
void mjXReader::OneFlex(XMLElement* elem, mjsFlex* flex) {
  string text, name, material, nodebody;
  int n;

  // read attributes
  if (ReadAttrTxt(elem, "name", name)) {
    if (mjs_setName(flex->element, name.c_str())) {
      throw mjXError(elem, "%s", mjs_getError(spec));
    }
  }
  if (ReadAttrTxt(elem, "material", material)) {
    mjs_setString(flex->material, material.c_str());
  }

  ReadAttr(elem, "radius", 1, &flex->radius, text);
  ReadAttr(elem, "rgba", 4, flex->rgba, text);
  if (MapValue(elem, "flatskin", &n, bool_map, 2)) {
    flex->flatskin = (n == 1);
  }
  ReadAttrInt(elem, "dim", &flex->dim);
  ReadAttrInt(elem, "group", &flex->group);

  // read data vectors
  if (ReadAttrTxt(elem, "body", text, true)) {
    mjs_setStringVec(flex->vertbody, text.c_str());
  }
  if (ReadAttrTxt(elem, "node", nodebody)) {
    mjs_setStringVec(flex->nodebody, nodebody.c_str());
  }
  auto vert = ReadAttrVec<double>(elem, "vertex");
  if (vert.has_value()) {
    mjs_setDouble(flex->vert, vert->data(), vert->size());
  }
  auto element = ReadAttrVec<int>(elem, "element", true);
  if (element.has_value()) {
    mjs_setInt(flex->elem, element->data(), element->size());
  }
  auto texcoord = ReadAttrVec<float>(elem, "texcoord");
  if (texcoord.has_value()) {
    mjs_setFloat(flex->texcoord, texcoord->data(), texcoord->size());
  }
  auto elemtexcoord = ReadAttrVec<int>(elem, "elemtexcoord");
  if (elemtexcoord.has_value()) {
    mjs_setInt(flex->elemtexcoord, elemtexcoord->data(), elemtexcoord->size());
  }

  // contact subelement
  XMLElement* cont = FirstChildElement(elem, "contact");
  if (cont) {
    ReadAttrInt(cont, "contype", &flex->contype);
    ReadAttrInt(cont, "conaffinity", &flex->conaffinity);
    ReadAttrInt(cont, "condim", &flex->condim);
    ReadAttrInt(cont, "priority", &flex->priority);
    ReadAttr(cont, "friction", 3, flex->friction, text, false, false);
    ReadAttr(cont, "solmix", 1, &flex->solmix, text);
    ReadAttr(cont, "solref", mjNREF, flex->solref, text, false, false);
    ReadAttr(cont, "solimp", mjNIMP, flex->solimp, text, false, false);
    ReadAttr(cont, "margin", 1, &flex->margin, text);
    ReadAttr(cont, "gap", 1, &flex->gap, text);
    if (MapValue(cont, "internal", &n, bool_map, 2)) {
      flex->internal = (n == 1);
    }
    MapValue(cont, "selfcollide", &flex->selfcollide, flexself_map, 5);
    if (MapValue(cont, "vertcollide", &flex->vertcollide, bool_map, 2)) {
      flex->vertcollide = (n == 1);
    }
    if (MapValue(cont, "passive", &flex->passive, bool_map, 2)) {
      flex->passive = (n == 1);
    }
    ReadAttrInt(cont, "activelayers", &flex->activelayers);
  }

  // edge subelement
  XMLElement* edge = FirstChildElement(elem, "edge");
  if (edge) {
    ReadAttr(edge, "stiffness", 1, &flex->edgestiffness, text);
    ReadAttr(edge, "damping", 1, &flex->edgedamping, text);
  }

  // elasticity subelement
  XMLElement* elasticity = FirstChildElement(elem, "elasticity");
  if (elasticity) {
    ReadAttr(elasticity, "young", 1, &flex->young, text);
    ReadAttr(elasticity, "poisson", 1, &flex->poisson, text);
    ReadAttr(elasticity, "thickness", 1, &flex->thickness, text);
    ReadAttr(elasticity, "damping", 1, &flex->damping, text);
    MapValue(elasticity, "elastic2d", &flex->elastic2d, elastic2d_map, 4);
  }

  // write error info
  mjs_setString(flex->info, ("line " + std::to_string(elem->GetLineNum())).c_str());
}



// mesh element parser
void mjXReader::OneMesh(XMLElement* elem, mjsMesh* mesh, const mjVFS* vfs) {
  int n;
  string text, name, content_type;

  // read attributes
  if (ReadAttrTxt(elem, "name", name)) {
    if (mjs_setName(mesh->element, name.c_str())) {
      throw mjXError(elem, "%s", mjs_getError(spec));
    }
  }
  if (ReadAttrTxt(elem, "content_type", content_type)) {
    *mesh->content_type = content_type;
  }
  auto file = ReadAttrFile(elem, "file", vfs, MeshDir());
  if (file) {
    mjs_setString(mesh->file, file->c_str());
  }
  ReadAttr(elem, "refpos", 3, mesh->refpos, text);
  ReadAttr(elem, "refquat", 4, mesh->refquat, text);
  ReadAttr(elem, "scale", 3, mesh->scale, text);
  if (MapValue(elem, "inertia", &n, meshinertia_map, 4)) {
    mesh->inertia = (mjtMeshInertia)n;
  }

  XMLElement* eplugin = FirstChildElement(elem, "plugin");
  if (eplugin) {
    OnePlugin(eplugin, &mesh->plugin);
  }

  if (MapValue(elem, "smoothnormal", &n, bool_map, 2)) {
    mesh->smoothnormal = (n == 1);
  }

  if (ReadAttrInt(elem, "maxhullvert", &n)) {
    if (n != -1 && n < 4) throw mjXError(elem, "maxhullvert must be larger than 3");
    mesh->maxhullvert = n;
  }

  // read user vertex data
  if (ReadAttrTxt(elem, "vertex", text)) {
    auto uservert = ReadAttrVec<float>(elem, "vertex");
    if (uservert.has_value()) {
      mjs_setFloat(mesh->uservert, uservert->data(), uservert->size());
    }
  }

  // read user normal data
  if (ReadAttrTxt(elem, "normal", text)) {
    auto usernormal = ReadAttrVec<float>(elem, "normal");
    if (usernormal.has_value()) {
      mjs_setFloat(mesh->usernormal, usernormal->data(), usernormal->size());
    }
  }

  // read user texcoord data
  if (ReadAttrTxt(elem, "texcoord", text)) {
    auto usertexcoord = ReadAttrVec<float>(elem, "texcoord");
    if (usertexcoord.has_value()) {
      mjs_setFloat(mesh->usertexcoord, usertexcoord->data(), usertexcoord->size());
    }
  }

  // read user face data
  if (ReadAttrTxt(elem, "face", text)) {
    auto userface = ReadAttrVec<int>(elem, "face");
    if (userface.has_value()) {
      mjs_setInt(mesh->userface, userface->data(), userface->size());
    }
  }

  // read builtin options
  if (MapValue(elem, "builtin", &n, meshbuiltin_map, meshbuiltin_sz)) {
    std::vector<double> params;
    int nparams = ReadVector(elem, "params", params, text, /*required*/ true);
    if (file) {
      throw mjXError(elem, "builtin cannot be used with a mesh file");
    }
    if (!mesh->uservert->empty()) {
      throw mjXError(elem, "builtin mesh cannot be used with user vertex data");
    }
    if (mjs_makeMesh(mesh, (mjtMeshBuiltin)n, params.data(), nparams)) {
      throw mjXError(elem, "%s", mjs_getError(spec));
    }
  }

  std::string material;
  if (ReadAttrTxt(elem, "material", material)) {
    mjs_setString(mesh->material, material.c_str());
  }

  // write error info
  mjs_setString(mesh->info, ("line " + std::to_string(elem->GetLineNum())).c_str());
}



// skin element parser
void mjXReader::OneSkin(XMLElement* elem, mjsSkin* skin, const mjVFS* vfs) {
  string text, name, material;
  float data[4];

  // read attributes
  if (ReadAttrTxt(elem, "name", name)) {
    if (mjs_setName(skin->element, name.c_str())) {
      throw mjXError(elem, "%s", mjs_getError(spec));
    }
  }
  auto file = ReadAttrFile(elem, "file", vfs, AssetDir());
  if (file.has_value()) {
    mjs_setString(skin->file, file->c_str());
  }
  if (ReadAttrTxt(elem, "material", material)) {
    mjs_setString(skin->material, material.c_str());
  }
  ReadAttrInt(elem, "group", &skin->group);
  if (skin->group < 0 || skin->group >= mjNGROUP) {
    throw mjXError(elem, "skin group must be between 0 and 5");
  }
  ReadAttr(elem, "rgba", 4, skin->rgba, text);
  ReadAttr(elem, "inflate", 1, &skin->inflate, text);

  // read vertex data
  auto vertex = ReadAttrVec<float>(elem, "vertex");
  if (vertex.has_value()) {
    mjs_setFloat(skin->vert, vertex->data(), vertex->size());
  }

  // read texcoord data
  auto texcoord = ReadAttrVec<float>(elem, "texcoord");
  if (texcoord.has_value()) {
    mjs_setFloat(skin->texcoord, texcoord->data(), texcoord->size());
  }

  // read user face data
  auto face = ReadAttrVec<int>(elem, "face");
  if (face.has_value()) {
    mjs_setInt(skin->face, face->data(), face->size());
  }

  // read bones
  XMLElement* bone = FirstChildElement(elem, "bone");
  std::vector<float> bindpos;
  std::vector<float> bindquat;

  while (bone) {
    // read body
    ReadAttrTxt(bone, "body", text, true);
    mjs_appendString(skin->bodyname, text.c_str());

    // read bindpos
    ReadAttr(bone, "bindpos", 3, data, text, true);
    bindpos.push_back(data[0]);
    bindpos.push_back(data[1]);
    bindpos.push_back(data[2]);

    // read bindquat
    ReadAttr(bone, "bindquat", 4, data, text, true);
    bindquat.push_back(data[0]);
    bindquat.push_back(data[1]);
    bindquat.push_back(data[2]);
    bindquat.push_back(data[3]);

    // read vertid
    auto tempid = ReadAttrVec<int>(bone, "vertid", true);
    if (tempid.has_value()) {
      mjs_appendIntVec(skin->vertid, tempid->data(), tempid->size());
    }

    // read vertweight
    auto tempweight = ReadAttrVec<float>(bone, "vertweight", true);
    if (tempweight.has_value()) {
      mjs_appendFloatVec(skin->vertweight, tempweight->data(), tempweight->size());
    }

    // advance to next bone
    bone = NextSiblingElement(bone, "bone");
  }

  // set bind vectors
  mjs_setFloat(skin->bindpos, bindpos.data(), bindpos.size());
  mjs_setFloat(skin->bindquat, bindquat.data(), bindquat.size());

  // write error info
  mjs_setString(skin->info, ("line " + std::to_string(elem->GetLineNum())).c_str());
}



// material element parser
void mjXReader::OneMaterial(XMLElement* elem, mjsMaterial* material) {
  string text, name, texture;
  int n;

  // read attributes
  if (ReadAttrTxt(elem, "name", name)) {
    if (mjs_setName(material->element, name.c_str())) {
      throw mjXError(elem, "%s", mjs_getError(spec));
    }
  }

  bool tex_attributes_found = false;
  if (ReadAttrTxt(elem, "texture", texture)) {
    mjs_setInStringVec(material->textures, mjTEXROLE_RGB, texture.c_str());
    tex_attributes_found = true;
  }

  XMLElement* layer = FirstChildElement(elem);
  while (layer) {
    if (tex_attributes_found) {
      throw mjXError(layer, "A material with a texture attribute cannot have layer sub-elements");
    }

    // layer sub-element
    ReadAttrTxt(layer, "role", text, true);
    int role = FindKey(texrole_map, texrole_sz, text);
    ReadAttrTxt(layer, "texture", text, true);
    mjs_setInStringVec(material->textures, role, text.c_str());
    layer = NextSiblingElement(layer);
  }

  if (MapValue(elem, "texuniform", &n, bool_map, 2)) {
    material->texuniform = (n == 1);
  }
  ReadAttr(elem, "texrepeat", 2, material->texrepeat, text);
  ReadAttr(elem, "emission", 1, &material->emission, text);
  ReadAttr(elem, "specular", 1, &material->specular, text);
  ReadAttr(elem, "shininess", 1, &material->shininess, text);
  ReadAttr(elem, "reflectance", 1, &material->reflectance, text);
  ReadAttr(elem, "metallic", 1, &material->metallic, text);
  ReadAttr(elem, "roughness", 1, &material->roughness, text);
  ReadAttr(elem, "rgba", 4, material->rgba, text);

  // write error info
  mjs_setString(material->info, ("line " + std::to_string(elem->GetLineNum())).c_str());
}



// joint element parser
void mjXReader::OneJoint(XMLElement* elem, mjsJoint* joint) {
  string text, name;
  std::vector<double> userdata;
  int n;

  // read attributes
  if (ReadAttrTxt(elem, "name", name)) {
    if (mjs_setName(joint->element, name.c_str())) {
      throw mjXError(elem, "%s", mjs_getError(spec));
    }
  }
  if (MapValue(elem, "type", &n, joint_map, joint_sz)) {
    joint->type = (mjtJoint)n;
  }
  MapValue(elem, "limited", &joint->limited, TFAuto_map, 3);
  MapValue(elem, "actuatorfrclimited", &joint->actfrclimited, TFAuto_map, 3);
  ReadAttrInt(elem, "group", &joint->group);
  ReadAttr(elem, "solreflimit", mjNREF, joint->solref_limit, text, false, false);
  ReadAttr(elem, "solimplimit", mjNIMP, joint->solimp_limit, text, false, false);
  ReadAttr(elem, "solreffriction", mjNREF, joint->solref_friction, text, false, false);
  ReadAttr(elem, "solimpfriction", mjNIMP, joint->solimp_friction, text, false, false);
  ReadAttr(elem, "pos", 3, joint->pos, text);
  ReadAttr(elem, "axis", 3, joint->axis, text);
  ReadAttr(elem, "springdamper", 2, joint->springdamper, text);
  ReadAttr(elem, "stiffness", 1, &joint->stiffness, text);
  ReadAttr(elem, "range", 2, joint->range, text);
  ReadAttr(elem, "actuatorfrcrange", 2, joint->actfrcrange, text);
  ReadAttr(elem, "margin", 1, &joint->margin, text);
  ReadAttr(elem, "ref", 1, &joint->ref, text);
  ReadAttr(elem, "springref", 1, &joint->springref, text);
  ReadAttr(elem, "armature", 1, &joint->armature, text);
  ReadAttr(elem, "damping", 1, &joint->damping, text);
  ReadAttr(elem, "frictionloss", 1, &joint->frictionloss, text);
  if (MapValue(elem, "actuatorgravcomp", &n, bool_map, 2)) {
    joint->actgravcomp = (n == 1);
  }

  // read userdata
  if (ReadVector(elem, "user", userdata, text)) {
    mjs_setDouble(joint->userdata, userdata.data(), userdata.size());
  }

  // write error info
  mjs_setString(joint->info, ("line " + std::to_string(elem->GetLineNum())).c_str());
}



// geom element parser
void mjXReader::OneGeom(XMLElement* elem, mjsGeom* geom) {
  string text, name;
  std::vector<double> userdata;
  string hfieldname, meshname, material;
  int n;

  // read attributes
  if (ReadAttrTxt(elem, "name", name)) {
    if (mjs_setName(geom->element, name.c_str())) {
      throw mjXError(elem, "%s", mjs_getError(spec));
    }
  }
  if (MapValue(elem, "type", &n, geom_map, mjNGEOMTYPES)) {
    geom->type = (mjtGeom)n;
  }
  ReadAttr(elem, "size", 3, geom->size, text, false, false);
  ReadAttrInt(elem, "contype", &geom->contype);
  ReadAttrInt(elem, "conaffinity", &geom->conaffinity);
  ReadAttrInt(elem, "condim", &geom->condim);
  ReadAttrInt(elem, "group", &geom->group);
  ReadAttrInt(elem, "priority", &geom->priority);
  ReadAttr(elem, "friction", 3, geom->friction, text, false, false);
  ReadAttr(elem, "solmix", 1, &geom->solmix, text);
  ReadAttr(elem, "solref", mjNREF, geom->solref, text, false, false);
  ReadAttr(elem, "solimp", mjNIMP, geom->solimp, text, false, false);
  ReadAttr(elem, "margin", 1, &geom->margin, text);
  ReadAttr(elem, "gap", 1, &geom->gap, text);
  if (ReadAttrTxt(elem, "hfield", hfieldname)) {
    mjs_setString(geom->hfieldname, hfieldname.c_str());
  }
  if (ReadAttrTxt(elem, "mesh", meshname)) {
    mjs_setString(geom->meshname, meshname.c_str());
  }
  ReadAttr(elem, "fitscale", 1, &geom->fitscale, text);
  if (ReadAttrTxt(elem, "material", material)) {
    mjs_setString(geom->material, material.c_str());
  }
  ReadAttr(elem, "rgba", 4, geom->rgba, text);
  if (MapValue(elem, "fluidshape", &n, fluid_map, 2)) {
    geom->fluid_ellipsoid = (n == 1);
  }
  ReadAttr(elem, "fluidcoef", 5, geom->fluid_coefs, text, false, false);

  // read userdata
  if (ReadVector(elem, "user", userdata, text)) {
    mjs_setDouble(geom->userdata, userdata.data(), userdata.size());
  }

  // plugin sub-element
  XMLElement* eplugin = FirstChildElement(elem, "plugin");
  if (eplugin) {
    OnePlugin(eplugin, &geom->plugin);
  }

  // remaining attributes
  ReadAttr(elem, "mass", 1, &geom->mass, text);
  ReadAttr(elem, "density", 1, &geom->density, text);
  ReadAttr(elem, "fromto", 6, geom->fromto, text);
  ReadAttr(elem, "pos", 3, geom->pos, text);
  ReadQuat(elem, "quat", geom->quat, text);
  ReadAlternative(elem, geom->alt);

  // compute inertia using either solid or shell geometry
  if (MapValue(elem, "shellinertia", &n, meshtype_map, 2)) {
    geom->typeinertia = (mjtGeomInertia)n;
  }

  // write error info
  mjs_setString(geom->info, ("line " + std::to_string(elem->GetLineNum())).c_str());
}



// site element parser
void mjXReader::OneSite(XMLElement* elem, mjsSite* site) {
  int n;
  string text, name;
  std::vector<double> userdata;
  string material;

  // read attributes
  if (ReadAttrTxt(elem, "name", name)) {
    if (mjs_setName(site->element, name.c_str())) {
      throw mjXError(elem, "%s", mjs_getError(spec));
    }
  }
  if (MapValue(elem, "type", &n, geom_map, mjNGEOMTYPES)) {
    site->type = (mjtGeom)n;
  }
  ReadAttr(elem, "size", 3, site->size, text, false, false);
  ReadAttrInt(elem, "group", &site->group);
  ReadAttr(elem, "pos", 3, site->pos, text);
  ReadQuat(elem, "quat", site->quat, text);
  if (ReadAttrTxt(elem, "material", material)) {
    mjs_setString(site->material, material.c_str());
  }
  ReadAttr(elem, "rgba", 4, site->rgba, text);
  ReadAttr(elem, "fromto", 6, site->fromto, text);
  ReadAlternative(elem, site->alt);
  if (ReadVector(elem, "user", userdata, text)) {
    mjs_setDouble(site->userdata, userdata.data(), userdata.size());
  }

  // write error info
  mjs_setString(site->info, ("line " + std::to_string(elem->GetLineNum())).c_str());
}



// camera element parser
void mjXReader::OneCamera(XMLElement* elem, mjsCamera* camera) {
  int n;
  string text, name, targetbody;
  std::vector<double> userdata;

  // read attributes
  if (ReadAttrTxt(elem, "name", name)) {
    if (mjs_setName(camera->element, name.c_str())) {
      throw mjXError(elem, "%s", mjs_getError(spec));
    }
  }
  if (ReadAttrTxt(elem, "target", targetbody)) {
    mjs_setString(camera->targetbody, targetbody.c_str());
  }
  if (MapValue(elem, "mode", &n, camlight_map, camlight_sz)) {
    camera->mode = (mjtCamLight)n;
  }
  ReadAttr(elem, "pos", 3, camera->pos, text);
  ReadQuat(elem, "quat", camera->quat, text);
  ReadAlternative(elem, camera->alt);
  ReadAttr(elem, "ipd", 1, &camera->ipd, text);

  if (MapValue(elem, "projection", &n, projection_map, 2)) {
    camera->proj = (mjtProjection)n;
  }

  bool has_principal = ReadAttr(elem, "principalpixel", 2, camera->principal_pixel, text) ||
                       ReadAttr(elem, "principal", 2, camera->principal_length, text);
  bool has_focal = ReadAttr(elem, "focalpixel", 2, camera->focal_pixel, text) ||
                   ReadAttr(elem, "focal", 2, camera->focal_length, text);
  bool needs_sensorsize = has_principal || has_focal;
  bool has_sensorsize = ReadAttr(elem, "sensorsize", 2, camera->sensor_size, text, needs_sensorsize);
  bool has_fovy = ReadAttr(elem, "fovy", 1, &camera->fovy, text);
  bool needs_resolution = has_focal || has_sensorsize;
  ReadAttr(elem, "resolution", 2, camera->resolution, text, needs_resolution);

  if (camera->resolution[0] < 0 || camera->resolution[1] < 0) {
    throw mjXError(elem, "camera resolution cannot be negative");
  }

  if (has_fovy && has_sensorsize) {
    throw mjXError(
            elem,
            "either 'fovy' or 'sensorsize' attribute can be specified, not both");
  }

  // read userdata
  ReadVector(elem, "user", userdata, text);
  mjs_setDouble(camera->userdata, userdata.data(), userdata.size());

  // write error info
  mjs_setString(camera->info, ("line " + std::to_string(elem->GetLineNum())).c_str());
}



// light element parser
void mjXReader::OneLight(XMLElement* elem, mjsLight* light) {
  int n;
  bool has_directional = false;
  string text, name, texture, targetbody;

  // read attributes
  if (ReadAttrTxt(elem, "name", name)) {
    if (mjs_setName(light->element, name.c_str())) {
      throw mjXError(elem, "%s", mjs_getError(spec));
    }
  }
  if (ReadAttrTxt(elem, "texture", texture)) {
    mjs_setString(light->texture, texture.c_str());
  }
  if (ReadAttrTxt(elem, "target", targetbody)) {
    mjs_setString(light->targetbody, targetbody.c_str());
  }
  if (MapValue(elem, "mode", &n, camlight_map, camlight_sz)) {
    light->mode = (mjtCamLight)n;
  }
  if (MapValue(elem, "directional", &n, bool_map, 2)) {
    light->type = (n == 1) ? mjLIGHT_DIRECTIONAL : mjLIGHT_SPOT;
    has_directional = true;
  }
  if (MapValue(elem, "type", &n, lighttype_map, lighttype_sz)) {
    if (has_directional) {
      throw mjXError(elem, "type and directional cannot both be defined");
    }
    light->type = (mjtLightType)n;
  }
  if (MapValue(elem, "castshadow", &n, bool_map, 2)) {
    light->castshadow = (n == 1);
  }
  if (MapValue(elem, "active", &n, bool_map, 2)) {
    light->active = (n == 1);
  }
  ReadAttr(elem, "pos", 3, light->pos, text);
  ReadAttr(elem, "dir", 3, light->dir, text);
  ReadAttr(elem, "bulbradius", 1, &light->bulbradius, text);
  ReadAttr(elem, "intensity", 1, &light->intensity, text);
  ReadAttr(elem, "range", 1, &light->range, text);
  ReadAttr(elem, "attenuation", 3, light->attenuation, text);
  ReadAttr(elem, "cutoff", 1, &light->cutoff, text);
  ReadAttr(elem, "exponent", 1, &light->exponent, text);
  ReadAttr(elem, "ambient", 3, light->ambient, text);
  ReadAttr(elem, "diffuse", 3, light->diffuse, text);
  ReadAttr(elem, "specular", 3, light->specular, text);

  // write error info
  mjs_setString(light->info, ("line " + std::to_string(elem->GetLineNum())).c_str());
}



// pair element parser
void mjXReader::OnePair(XMLElement* elem, mjsPair* pair) {
  string text, name, geomname1, geomname2;

  // regular only
  if (!readingdefaults) {
    if (ReadAttrTxt(elem, "geom1", geomname1)) {
      mjs_setString(pair->geomname1, geomname1.c_str());
    }
    if (ReadAttrTxt(elem, "geom2", geomname2)) {
      mjs_setString(pair->geomname2, geomname2.c_str());
    }
  }

  // read other parameters
  if (ReadAttrTxt(elem, "name", name)) {
    if (mjs_setName(pair->element, name.c_str())) {
      throw mjXError(elem, "%s", mjs_getError(spec));
    }
  }
  ReadAttrInt(elem, "condim", &pair->condim);
  ReadAttr(elem, "solref", mjNREF, pair->solref, text, false, false);
  ReadAttr(elem, "solreffriction", mjNREF, pair->solreffriction, text, false, false);
  ReadAttr(elem, "solimp", mjNIMP, pair->solimp, text, false, false);
  ReadAttr(elem, "margin", 1, &pair->margin, text);
  ReadAttr(elem, "gap", 1, &pair->gap, text);
  ReadAttr(elem, "friction", 5, pair->friction, text, false, false);

  // write error info
  mjs_setString(pair->info, ("line " + std::to_string(elem->GetLineNum())).c_str());
}



// equality element parser
void mjXReader::OneEquality(XMLElement* elem, mjsEquality* equality) {
  int n;
  string text, name1, name2, name;

  // read type (bad keywords already detected by schema)
  text = elem->Value();
  equality->type = (mjtEq)FindKey(equality_map, equality_sz, text);

  // regular only
  if (!readingdefaults) {
    if (ReadAttrTxt(elem, "name", name)) {
      if (mjs_setName(equality->element, name.c_str())) {
        throw mjXError(elem, "%s", mjs_getError(spec));
      }
    }

    switch (equality->type) {
      case mjEQ_CONNECT: {
        auto maybe_site1 = ReadAttrStr(elem, "site1");
        auto maybe_site2 = ReadAttrStr(elem, "site2");
        auto maybe_body1 = ReadAttrStr(elem, "body1");
        auto maybe_body2 = ReadAttrStr(elem, "body2");
        bool has_anchor = ReadAttr(elem, "anchor", 3, equality->data, text);

        bool maybe_site = maybe_site1.has_value() || maybe_site2.has_value();
        bool maybe_body = maybe_body1.has_value() || maybe_body2.has_value() || has_anchor;

        if (maybe_site && maybe_body) {
          throw mjXError(elem, "body and site semantics cannot be mixed");
        }

        bool site_semantic = maybe_site1.has_value() && maybe_site2.has_value();
        bool body_semantic = maybe_body1.has_value() && has_anchor;
        if (site_semantic == body_semantic) {
          throw mjXError(elem, "either both body1 and anchor must be defined,"
                         " or both site1 and site2 must be defined");
        }

        if (body_semantic) {
          name1 = maybe_body1.value();
          if (maybe_body2.has_value()) {
            name2 = maybe_body2.value();
          }
          equality->objtype = mjOBJ_BODY;
        } else {
          name1 = maybe_site1.value();
          name2 = maybe_site2.value();
          equality->objtype = mjOBJ_SITE;
        }
      }
      break;

      case mjEQ_WELD: {
        auto maybe_site1 = ReadAttrStr(elem, "site1");
        auto maybe_site2 = ReadAttrStr(elem, "site2");
        auto maybe_body1 = ReadAttrStr(elem, "body1");
        auto maybe_body2 = ReadAttrStr(elem, "body2");
        bool has_anchor = ReadAttr(elem, "anchor", 3, equality->data, text);
        bool has_relpose = ReadAttr(elem, "relpose", 7, equality->data+3, text);

        bool maybe_site = maybe_site1.has_value() || maybe_site2.has_value();
        bool maybe_body = maybe_body1.has_value() ||
                          maybe_body2.has_value() ||
                          has_anchor              ||
                          has_relpose;

        if (maybe_site && maybe_body) {
          throw mjXError(elem, "body and site semantics cannot be mixed");
        }

        bool site_semantic = maybe_site1.has_value() && maybe_site2.has_value();
        bool body_semantic = maybe_body1.has_value();

        if (site_semantic == body_semantic) {
          throw mjXError(
                  elem,
                  "either body1 must be defined and optionally {body2, anchor, relpose},"
                  " or site1 and site2 must be defined");
        }

        if (body_semantic) {
          name1 = maybe_body1.value();
          if (maybe_body2.has_value()) {
            name2 = maybe_body2.value();
          }
          equality->objtype = mjOBJ_BODY;
          if (!has_anchor) {
            mjuu_zerovec(equality->data, 3);
          }
        } else {
          name1 = maybe_site1.value();
          name2 = maybe_site2.value();
          equality->objtype = mjOBJ_SITE;
        }

        ReadAttr(elem, "torquescale", 1, equality->data+10, text);
      }
      break;

      case mjEQ_JOINT:
        ReadAttrTxt(elem, "joint1", name1, true);
        ReadAttrTxt(elem, "joint2", name2);
        ReadAttr(elem, "polycoef", 5, equality->data, text, false, false);
        break;

      case mjEQ_TENDON:
        ReadAttrTxt(elem, "tendon1", name1, true);
        ReadAttrTxt(elem, "tendon2", name2);
        ReadAttr(elem, "polycoef", 5, equality->data, text, false, false);
        break;

      case mjEQ_FLEX:
        ReadAttrTxt(elem, "flex", name1, true);
        break;

      case mjEQ_DISTANCE:
        throw mjXError(elem, "support for distance equality constraints was removed in MuJoCo 2.2.2");
        break;

      default:                  // SHOULD NOT OCCUR
        throw mjXError(elem, "unrecognized equality constraint type");
    }

    mjs_setString(equality->name1, name1.c_str());
    if (!name2.empty()) {
      mjs_setString(equality->name2, name2.c_str());
    }
  }

  // read attributes
  if (MapValue(elem, "active", &n, bool_map, 2)) {
    equality->active = (n == 1);
  }
  ReadAttr(elem, "solref", mjNREF, equality->solref, text, false, false);
  ReadAttr(elem, "solimp", mjNIMP, equality->solimp, text, false, false);

  // write error info
  mjs_setString(equality->info, ("line " + std::to_string(elem->GetLineNum())).c_str());
}



// tendon element parser
void mjXReader::OneTendon(XMLElement* elem, mjsTendon* tendon) {
  string text, name, material;
  std::vector<double> userdata;

  // read attributes
  if (ReadAttrTxt(elem, "name", name)) {
    if (mjs_setName(tendon->element, name.c_str())) {
      throw mjXError(elem, "%s", mjs_getError(spec));
    }
  }
  ReadAttrInt(elem, "group", &tendon->group);
  if (ReadAttrTxt(elem, "material", material)) {
    mjs_setString(tendon->material, material.c_str());
  }
  MapValue(elem, "limited", &tendon->limited, TFAuto_map, 3);
  MapValue(elem, "actuatorfrclimited", &tendon->actfrclimited, TFAuto_map, 3);
  ReadAttr(elem, "width", 1, &tendon->width, text);
  ReadAttr(elem, "solreflimit", mjNREF, tendon->solref_limit, text, false, false);
  ReadAttr(elem, "solimplimit", mjNIMP, tendon->solimp_limit, text, false, false);
  ReadAttr(elem, "solreffriction", mjNREF, tendon->solref_friction, text, false, false);
  ReadAttr(elem, "solimpfriction", mjNIMP, tendon->solimp_friction, text, false, false);
  ReadAttr(elem, "range", 2, tendon->range, text);
  ReadAttr(elem, "actuatorfrcrange", 2, tendon->actfrcrange, text);
  ReadAttr(elem, "margin", 1, &tendon->margin, text);
  ReadAttr(elem, "stiffness", 1, &tendon->stiffness, text);
  ReadAttr(elem, "damping", 1, &tendon->damping, text);
  ReadAttr(elem, "armature", 1, &tendon->armature, text);
  ReadAttr(elem, "frictionloss", 1, &tendon->frictionloss, text);
  // read springlength, either one or two values; if one, copy to second value
  if (ReadAttr(elem, "springlength", 2, tendon->springlength, text, false, false) == 1) {
    tendon->springlength[1] = tendon->springlength[0];
  }
  ReadAttr(elem, "rgba", 4, tendon->rgba, text);

  // read userdata
  if (ReadVector(elem, "user", userdata, text)) {
    mjs_setDouble(tendon->userdata, userdata.data(), userdata.size());
  }

  // write error info
  mjs_setString(tendon->info, ("line " + std::to_string(elem->GetLineNum())).c_str());
}



// actuator element parser
void mjXReader::OneActuator(XMLElement* elem, mjsActuator* actuator) {
  string text, type, name, target, slidersite, refsite;

  // common attributes
  if (ReadAttrTxt(elem, "name", name)) {
    if (mjs_setName(actuator->element, name.c_str())) {
      throw mjXError(elem, "%s", mjs_getError(spec));
    }
  }
  ReadAttrInt(elem, "group", &actuator->group);
  MapValue(elem, "ctrllimited", &actuator->ctrllimited, TFAuto_map, 3);
  MapValue(elem, "forcelimited", &actuator->forcelimited, TFAuto_map, 3);
  MapValue(elem, "actlimited", &actuator->actlimited, TFAuto_map, 3);
  ReadAttr(elem, "ctrlrange", 2, actuator->ctrlrange, text);
  ReadAttr(elem, "forcerange", 2, actuator->forcerange, text);
  ReadAttr(elem, "actrange", 2, actuator->actrange, text);
  ReadAttr(elem, "lengthrange", 2, actuator->lengthrange, text);
  ReadAttr(elem, "gear", 6, actuator->gear, text, false, false);

  // transmission target and type
  int cnt = 0;
  if (ReadAttrTxt(elem, "joint", target)) {
    mjs_setString(actuator->target, target.c_str());
    actuator->trntype = mjTRN_JOINT;
    cnt++;
  }
  if (ReadAttrTxt(elem, "jointinparent", target)) {
    mjs_setString(actuator->target, target.c_str());
    actuator->trntype = mjTRN_JOINTINPARENT;
    cnt++;
  }
  if (ReadAttrTxt(elem, "tendon", target)) {
    mjs_setString(actuator->target, target.c_str());
    actuator->trntype = mjTRN_TENDON;
    cnt++;
  }
  if (ReadAttrTxt(elem, "cranksite", target)) {
    mjs_setString(actuator->target, target.c_str());
    actuator->trntype = mjTRN_SLIDERCRANK;
    cnt++;
  }
  if (ReadAttrTxt(elem, "site", target)) {
    mjs_setString(actuator->target, target.c_str());
    actuator->trntype = mjTRN_SITE;
    cnt++;
  }
  if (ReadAttrTxt(elem, "body", target)) {
    mjs_setString(actuator->target, target.c_str());
    actuator->trntype = mjTRN_BODY;
    cnt++;
  }
  // check for repeated transmission
  if (cnt > 1) {
    throw mjXError(elem, "actuator can have at most one of transmission target");
  }

  // slidercrank-specific parameters
  int r1 = ReadAttr(elem, "cranklength", 1, &actuator->cranklength, text);
  int r2 = ReadAttrTxt(elem, "slidersite", slidersite);
  if (r2) {
    mjs_setString(actuator->slidersite, slidersite.c_str());
  }
  if ((r1 || r2) &&
      actuator->trntype != mjTRN_SLIDERCRANK &&
      actuator->trntype != mjTRN_UNDEFINED) {
    throw mjXError(elem, "cranklength and slidersite can only be used in slidercrank transmission");
  }

  // site-specific parameters (refsite)
  int r3 = ReadAttrTxt(elem, "refsite", refsite);
  if (r3) {
    mjs_setString(actuator->refsite, refsite.c_str());
  }
  if (r3 && actuator->trntype != mjTRN_SITE && actuator->trntype != mjTRN_UNDEFINED) {
    throw mjXError(elem, "refsite can only be used with site transmission");
  }

  // get predefined type
  type = elem->Value();

  // explicit attributes
  string err;
  if (type == "general") {
    // explicit attributes
    int n;
    if (MapValue(elem, "dyntype", &n, dyn_map, dyn_sz)) {
      actuator->dyntype = (mjtDyn)n;
    }
    if (MapValue(elem, "gaintype", &n, gain_map, gain_sz)) {
      actuator->gaintype = (mjtGain)n;
    }
    if (MapValue(elem, "biastype", &n, bias_map, bias_sz)) {
      actuator->biastype = (mjtBias)n;
    }
    if (MapValue(elem, "actearly", &n, bool_map, 2)) {
      actuator->actearly = (n == 1);
    }
    ReadAttr(elem, "dynprm", mjNDYN, actuator->dynprm, text, false, false);
    ReadAttr(elem, "gainprm", mjNGAIN, actuator->gainprm, text, false, false);
    ReadAttr(elem, "biasprm", mjNBIAS, actuator->biasprm, text, false, false);
    ReadAttrInt(elem, "actdim", &actuator->actdim);
  }

  // direct drive motor
  else if (type == "motor") {
    err = mjs_setToMotor(actuator);
  }

  // position or integrated velocity servo
  else if (type == "position" || type == "intvelocity") {
    double kp = actuator->gainprm[0];
    ReadAttr(elem, "kp", 1, &kp, text);

    // read kv
    double kv_data;
    double *kv = &kv_data;
    if (!ReadAttr(elem, "kv", 1, kv, text)) {
      kv = nullptr;
    }

    // read dampratio
    double dampratio_data;
    double *dampratio = &dampratio_data;
    if (!ReadAttr(elem, "dampratio", 1, dampratio, text)) {
      dampratio = nullptr;
    }

    // read timeconst, set dyntype
    double timeconst_data;
    double *timeconst = &timeconst_data;
    if (!ReadAttr(elem, "timeconst", 1, timeconst, text)) {
      timeconst = nullptr;
    }

    // handle inheritrange
    double inheritrange = actuator->inheritrange;
    ReadAttr(elem, "inheritrange", 1, &inheritrange, text);

    if (type == "position") {
      err = mjs_setToPosition(actuator, kp, kv, dampratio, timeconst, inheritrange);
    } else {
      err = mjs_setToIntVelocity(actuator, kp, kv, dampratio, timeconst, inheritrange);
    }
  }

  // velocity servo
  else if (type == "velocity") {
    double kv = actuator->gainprm[0];
    ReadAttr(elem, "kv", 1, &kv, text);
    err = mjs_setToVelocity(actuator, kv);
  }

  // damper
  else if (type == "damper") {
    double kv = 0;
    ReadAttr(elem, "kv", 1, &kv, text);
    err = mjs_setToDamper(actuator, kv);
  }

  // cylinder
  else if (type == "cylinder") {
    double timeconst = actuator->dynprm[0];
    double bias = actuator->biasprm[0];
    double area = actuator->gainprm[0];
    double diameter = -1;
    ReadAttr(elem, "timeconst", 1, &timeconst, text);
    ReadAttr(elem, "bias", 3, &bias, text);
    ReadAttr(elem, "area", 1, &area, text);
    ReadAttr(elem, "diameter", 1, &diameter, text);
    err = mjs_setToCylinder(actuator, timeconst, bias, area, diameter);
  }

  // muscle
  else if (type == "muscle") {
    double tausmooth = actuator->dynprm[2];
    double force = -1, scale = -1, lmin = -1, lmax = -1, vmax = -1, fpmax = -1, fvmax = -1;
    double range[2] = {-1, -1}, timeconst[2] = {-1, -1};
    ReadAttr(elem, "timeconst", 2, timeconst, text);
    ReadAttr(elem, "tausmooth", 1, &tausmooth, text);
    ReadAttr(elem, "range", 2, range, text);
    ReadAttr(elem, "force", 1, &force, text);
    ReadAttr(elem, "scale", 1, &scale, text);
    ReadAttr(elem, "lmin", 1, &lmin, text);
    ReadAttr(elem, "lmax", 1, &lmax, text);
    ReadAttr(elem, "vmax", 1, &vmax, text);
    ReadAttr(elem, "fpmax", 1, &fpmax, text);
    ReadAttr(elem, "fvmax", 1, &fvmax, text);
    err = mjs_setToMuscle(actuator, timeconst, tausmooth, range, force, scale,
                          lmin, lmax, vmax, fpmax, fvmax);
  }

  // adhesion
  else if (type == "adhesion") {
    double gain = actuator->gainprm[0];
    ReadAttr(elem, "gain", 1, &gain, text);
    ReadAttr(elem, "ctrlrange", 2, actuator->ctrlrange, text);
    err = mjs_setToAdhesion(actuator, gain);
  }

  else if (type == "plugin") {
    OnePlugin(elem, &actuator->plugin);
    int n;
    if (MapValue(elem, "dyntype", &n, dyn_map, dyn_sz)) {
      actuator->dyntype = (mjtDyn)n;
    }
    if (MapValue(elem, "actearly", &n, bool_map, 2)) {
      actuator->actearly = (n == 1);
    }
    ReadAttr(elem, "dynprm", mjNDYN, actuator->dynprm, text, false, false);
    ReadAttrInt(elem, "actdim", &actuator->actdim);
  }

  else {          // SHOULD NOT OCCUR
    throw mjXError(elem, "unrecognized actuator type: %s", type.c_str());
  }

  // throw error if any of the above failed
  if (!err.empty()) {
    throw mjXError(elem, err.c_str());
  }

  // read userdata
  std::vector<double> userdata;
  if (ReadVector(elem, "user", userdata, text)) {
    mjs_setDouble(actuator->userdata, userdata.data(), userdata.size());
  }

  // write info
  mjs_setString(actuator->info, ("line " + std::to_string(elem->GetLineNum())).c_str());
}



// make composite
void mjXReader::OneComposite(XMLElement* elem, mjsBody* body, mjsFrame* frame, const mjsDefault* def) {
  string text;
  int n;

  // create out-of-DOM element
  mjCComposite comp;

  // common properties
  ReadAttrTxt(elem, "prefix", comp.prefix);
  if (MapValue(elem, "type", &n, comp_map, mjNCOMPTYPES, true)) {
    comp.type = (mjtCompType)n;
  }
  ReadAttr(elem, "count", 3, comp.count, text, false, false);
  ReadAttr(elem, "offset", 3, comp.offset, text);
  ReadAttr(elem, "quat", 4, comp.quat, text);
  comp.frame = frame;

  // plugin
  XMLElement* eplugin = FirstChildElement(elem, "plugin");
  if (eplugin) {
    OnePlugin(eplugin, &comp.plugin);
  }

  // cable
  string curves;
  ReadAttrTxt(elem, "curve", curves);
  ReadAttrTxt(elem, "initial", comp.initial);
  ReadAttr(elem, "size", 3, comp.size, text, false, false);
  auto uservert = ReadAttrVec<float>(elem, "vertex");
  if (uservert.has_value()) {
    comp.uservert = std::move(uservert.value());
  }

  // process curve string
  std::istringstream iss(curves);
  int i = 0;
  while (iss) {
    if (curves.empty()) {
      break;
    }
    iss >> text;
    if (i > 2) {
      throw mjXError(elem, "The curve array must have a maximum of 3 components");
    }
    comp.curve[i++] = (mjtCompShape)FindKey(shape_map, mjNCOMPSHAPES, text);
    if (comp.curve[i-1] == -1) {
      throw mjXError(elem, "The curve array contains an invalid shape");
    }
    if (iss.eof()){
      break;
    }
  };

  // skin
  XMLElement* eskin = FirstChildElement(elem, "skin");
  if (eskin) {
    comp.skin = true;
    if (MapValue(eskin, "texcoord", &n, bool_map, 2)) {
      comp.skintexcoord = (n == 1);
    }
    ReadAttrTxt(eskin, "material", comp.skinmaterial);
    ReadAttr(eskin, "rgba", 4, comp.skinrgba, text);
    ReadAttr(eskin, "inflate", 1, &comp.skininflate, text);
    ReadAttrInt(eskin, "subgrid", &comp.skinsubgrid);
    ReadAttrInt(eskin, "group", &comp.skingroup, 0);
    if (comp.skingroup < 0 || comp.skingroup >= mjNGROUP) {
      throw mjXError(eskin, "skin group must be between 0 and 5");
    }
  }

  // set type-specific defaults
  comp.SetDefault();

  // geom
  XMLElement* egeom = FirstChildElement(elem, "geom");
  if (egeom) {
    string material;
    mjsGeom& dgeom = *comp.def[0].spec.geom;
    if (MapValue(egeom, "type", &n, geom_map, mjNGEOMTYPES)) {
      dgeom.type = (mjtGeom)n;
    }
    ReadAttr(egeom, "size", 3, dgeom.size, text, false, false);
    ReadAttrInt(egeom, "contype", &dgeom.contype);
    ReadAttrInt(egeom, "conaffinity", &dgeom.conaffinity);
    ReadAttrInt(egeom, "condim", &dgeom.condim);
    ReadAttrInt(egeom, "group", &dgeom.group);
    ReadAttrInt(egeom, "priority", &dgeom.priority);
    ReadAttr(egeom, "friction", 3, dgeom.friction, text, false, false);
    ReadAttr(egeom, "solmix", 1, &dgeom.solmix, text);
    ReadAttr(egeom, "solref", mjNREF, dgeom.solref, text, false, false);
    ReadAttr(egeom, "solimp", mjNIMP, dgeom.solimp, text, false, false);
    ReadAttr(egeom, "margin", 1, &dgeom.margin, text);
    ReadAttr(egeom, "gap", 1, &dgeom.gap, text);
    if (ReadAttrTxt(egeom, "material", material)) {
      mjs_setString(dgeom.material, material.c_str());
    }
    ReadAttr(egeom, "rgba", 4, dgeom.rgba, text);
    ReadAttr(egeom, "mass", 1, &dgeom.mass, text);
    ReadAttr(egeom, "density", 1, &dgeom.density, text);
  }

  // site
  XMLElement* esite = FirstChildElement(elem, "site");
  if (esite) {
    string material;
    mjsSite& dsite = *comp.def[0].spec.site;
    ReadAttr(esite, "size", 3, dsite.size, text, false, false);
    ReadAttrInt(esite, "group", &dsite.group);
    ReadAttrTxt(esite, "material", material);
    ReadAttr(esite, "rgba", 4, dsite.rgba, text);
    mjs_setString(dsite.material, material.c_str());
  }

  // joint
  XMLElement* ejnt = FirstChildElement(elem, "joint");
  while (ejnt) {
    // kind
    int kind;
    MapValue(ejnt, "kind", &kind, jkind_map, 1, true);

    // create a new element if this kind already exists
    if (comp.add[kind]) {
      char error[200];
      if (!comp.AddDefaultJoint(error, 200)) {
        throw mjXError(elem, "%s", error);
      }
    }
    comp.add[kind] = true;

    // get element
    mjsDefault* dspec = &comp.defjoint[(mjtCompKind)kind].back().spec;
    mjsJoint& djoint = *dspec->joint;
    mjsEquality& dequality = *dspec->equality;

    // particle joint
    if (MapValue(ejnt, "type", &n, joint_map, joint_sz)) {
      djoint.type = (mjtJoint)n;
    }
    ReadAttr(ejnt, "axis", 3, djoint.axis, text);

    // solreffix, solimpfix
    ReadAttr(ejnt, "solreffix", mjNREF, dequality.solref, text, false, false);
    ReadAttr(ejnt, "solimpfix", mjNIMP, dequality.solimp, text, false, false);

    // joint attributes
    MapValue(elem, "limited", &djoint.limited, TFAuto_map, 3);
    ReadAttrInt(ejnt, "group", &djoint.group);
    ReadAttr(ejnt, "solreflimit", mjNREF, djoint.solref_limit, text, false, false);
    ReadAttr(ejnt, "solimplimit", mjNIMP, djoint.solimp_limit, text, false, false);
    ReadAttr(ejnt,
             "solreffriction", mjNREF, djoint.solref_friction, text, false, false);
    ReadAttr(ejnt,
             "solimpfriction", mjNIMP, djoint.solimp_friction, text, false, false);
    ReadAttr(ejnt, "stiffness", 1, &djoint.stiffness, text);
    ReadAttr(ejnt, "range", 2, djoint.range, text);
    ReadAttr(ejnt, "margin", 1, &djoint.margin, text);
    ReadAttr(ejnt, "armature", 1, &djoint.armature, text);
    ReadAttr(ejnt, "damping", 1, &djoint.damping, text);
    ReadAttr(ejnt, "frictionloss", 1, &djoint.frictionloss, text);

    // advance
    ejnt = NextSiblingElement(ejnt, "joint");
  }


  // make composite
  char error[200];
  bool res = comp.Make(spec, body, error, 200);

  // throw error
  if (!res) {
    throw mjXError(elem, "%s", error);
  }
}



// make flexcomp
void mjXReader::OneFlexcomp(XMLElement* elem, mjsBody* body, const mjVFS* vfs) {
  string text, material;
  int n;

  // create out-of-DOM element
  mjCFlexcomp fcomp;
  mjsFlex& dflex = *fcomp.def.spec.flex;

  // common properties
  ReadAttrTxt(elem, "name", fcomp.name, true);
  if (MapValue(elem, "type", &n, fcomp_map, mjNFCOMPTYPES)) {
    fcomp.type = (mjtFcompType)n;
  }
  ReadAttr(elem, "count", 3, fcomp.count, text);
  ReadAttr(elem, "spacing", 3, fcomp.spacing, text);
  ReadAttr(elem, "scale", 3, fcomp.scale, text);
  ReadAttr(elem, "mass", 1, &fcomp.mass, text);
  ReadAttr(elem, "inertiabox", 1, &fcomp.inertiabox, text);
  auto maybe_file = ReadAttrFile(elem, "file", vfs, modelfiledir_);
  if (maybe_file.has_value()) {
    fcomp.file = std::move(maybe_file.value().Str());
  } else {
    fcomp.file = "";
  }
  if (ReadAttrTxt(elem, "material", material)) {
    mjs_setString(dflex.material, material.c_str());
  }
  ReadAttr(elem, "rgba", 4, dflex.rgba, text);
  if (MapValue(elem, "flatskin", &n, bool_map, 2)) {
    dflex.flatskin = (n == 1);
  }
  ReadAttrInt(elem, "dim", &dflex.dim);
  ReadAttr(elem, "radius", 1, &dflex.radius, text);
  ReadAttrInt(elem, "group", &dflex.group);
  if (!ReadAttr(elem, "origin", 3, fcomp.origin, text) &&
      fcomp.type == mjFCOMPTYPE_MESH && dflex.dim == 3) {
    throw mjXError(elem, "origin must be specified for mesh flexcomps if dim=3");
  }

  // pose
  ReadAttr(elem, "pos", 3, fcomp.pos, text);
  ReadAttr(elem, "quat", 4, fcomp.quat, text);
  ReadAlternative(elem, fcomp.alt);

  // user or internal
  if (MapValue(elem, "rigid", &n, bool_map, 2)) {
    fcomp.rigid = (n == 1);
  }
  auto point = ReadAttrVec<double>(elem, "point");
  if (point.has_value()) {
    fcomp.point = std::move(point.value());
  }
  auto element = ReadAttrVec<int>(elem, "element");
  if (element.has_value()) {
    fcomp.element = std::move(element.value());
  }
  auto texcoord = ReadAttrVec<float>(elem, "texcoord");
  if (texcoord.has_value()) {
    fcomp.texcoord = std::move(texcoord.value());
  }

  // dof type
  if (MapValue(elem, "dof", &n, fdof_map, mjNFCOMPDOFS)) {
    fcomp.doftype = (mjtDof)n;
  }

  // edge
  XMLElement* edge = FirstChildElement(elem, "edge");
  if (edge) {
    if (MapValue(edge, "equality", &n, bool_map, 2)) {
      fcomp.equality = (n == 1);
    }
    ReadAttr(edge, "solref", mjNREF, fcomp.def.spec.equality->solref, text, false, false);
    ReadAttr(edge, "solimp", mjNIMP, fcomp.def.spec.equality->solimp, text, false, false);
    ReadAttr(edge, "stiffness", 1, &dflex.edgestiffness, text);
    ReadAttr(edge, "damping", 1, &dflex.edgedamping, text);
  }

  // elasticity
  XMLElement* elasticity = FirstChildElement(elem, "elasticity");
  if (elasticity) {
    ReadAttr(elasticity, "young", 1, &dflex.young, text);
    ReadAttr(elasticity, "poisson", 1, &dflex.poisson, text);
    ReadAttr(elasticity, "damping", 1, &dflex.damping, text);
    ReadAttr(elasticity, "thickness", 1, &dflex.thickness, text);
    MapValue(elasticity, "elastic2d", &dflex.elastic2d, elastic2d_map, 4);
  }

  // check errors
  if (dflex.elastic2d >= 2 && fcomp.equality) {
    throw mjXError(elem, "elasticity and edge constraints cannot both be present");
  }

  // contact
  XMLElement* cont = FirstChildElement(elem, "contact");
  if (cont) {
    ReadAttrInt(cont, "contype", &dflex.contype);
    ReadAttrInt(cont, "conaffinity", &dflex.conaffinity);
    ReadAttrInt(cont, "condim", &dflex.condim);
    ReadAttrInt(cont, "priority", &dflex.priority);
    ReadAttr(cont, "friction", 3, dflex.friction, text, false, false);
    ReadAttr(cont, "solmix", 1, &dflex.solmix, text);
    ReadAttr(cont, "solref", mjNREF, dflex.solref, text, false, false);
    ReadAttr(cont, "solimp", mjNIMP, dflex.solimp, text, false, false);
    ReadAttr(cont, "margin", 1, &dflex.margin, text);
    ReadAttr(cont, "gap", 1, &dflex.gap, text);
    if (MapValue(cont, "internal", &n, bool_map, 2)) {
      dflex.internal = (n == 1);
    }
    MapValue(cont, "selfcollide", &dflex.selfcollide, flexself_map, 5);
    if (MapValue(cont, "vertcollide", &n, bool_map, 2)) {
      dflex.vertcollide = (n == 1);
    }
    if (MapValue(cont, "passive", &n, bool_map, 2)) {
      dflex.passive = (n == 1);
    }
    ReadAttrInt(cont, "activelayers", &dflex.activelayers);
  }

  // pin
  XMLElement* epin = FirstChildElement(elem, "pin");
  while (epin) {
    auto id = ReadAttrVec<int>(epin, "id");
    if (id.has_value()) {
      fcomp.pinid.insert(fcomp.pinid.end(), id->begin(), id->end());
    }
    auto range = ReadAttrVec<int>(epin, "range");
    if (range.has_value()) {
      fcomp.pinrange.insert(fcomp.pinrange.end(), range->begin(), range->end());
    }
    auto grid = ReadAttrVec<int>(epin, "grid");
    if (grid.has_value()) {
      fcomp.pingrid.insert(fcomp.pingrid.end(), grid->begin(), grid->end());
    }
    auto gridrange = ReadAttrVec<int>(epin, "gridrange");
    if (gridrange.has_value()) {
      fcomp.pingridrange.insert(fcomp.pingridrange.end(),
                                gridrange->begin(), gridrange->end());
    }

    // advance
    epin = NextSiblingElement(epin, "pin");
  }

  // plugin
  XMLElement* eplugin = FirstChildElement(elem, "plugin");
  if (eplugin) {
    OnePlugin(eplugin, &fcomp.plugin);
  }

  // make flexcomp
  char error[200];
  bool res = fcomp.Make(body, error, 200);

  // throw error
  if (!res) {
    throw mjXError(elem, "%s", error);
  }
}



// add plugin
void mjXReader::OnePlugin(XMLElement* elem, mjsPlugin* plugin) {
  plugin->active = true;
  string name = "";
  string instance_name = "";
  ReadAttrTxt(elem, "plugin", name);
  ReadAttrTxt(elem, "instance", instance_name);
  mjs_setString(plugin->plugin_name, name.c_str());
  mjs_setString(plugin->name, instance_name.c_str());
  if (instance_name.empty()) {
    plugin->element = mjs_addPlugin(spec)->element;
    ReadPluginConfigs(elem, plugin);
  } else {
    spec->hasImplicitPluginElem = true;
  }
}



//------------------ MJCF-specific sections --------------------------------------------------------

// default section parser
void mjXReader::Default(XMLElement* section, const mjsDefault* def, const mjVFS* vfs) {
  XMLElement* elem;
  string text, name;

  // create new default, except at top level (already added in mjCModel constructor)
  text.clear();
  ReadAttrTxt(section, "class", text);
  if (text.empty()) {
    if (def) {
      throw mjXError(section, "empty class name");
    }
  }
  if (def) {
    def = mjs_addDefault(spec, text.c_str(), def);
    if (!def) {
      throw mjXError(section, "repeated default class name");
    }
  } else {
    def = mjs_getSpecDefault(spec);
    if (!text.empty() && text != "main") {
      throw mjXError(section, "top-level default class 'main' cannot be renamed");
    }
  }

  // iterate over elements other than nested defaults
  elem = FirstChildElement(section);
  while (elem) {
    // get element name
    name = elem->Value();

    // read mesh
    if (name == "mesh")OneMesh(elem, def->mesh, vfs);

    // read material
    else if (name == "material")OneMaterial(elem, def->material);

    // read joint
    else if (name == "joint")OneJoint(elem, def->joint);

    // read geom
    else if (name == "geom")OneGeom(elem, def->geom);

    // read site
    else if (name == "site")OneSite(elem, def->site);

    // read camera
    else if (name == "camera")OneCamera(elem, def->camera);

    // read light
    else if (name == "light")OneLight(elem, def->light);

    // read pair
    else if (name == "pair")OnePair(elem, def->pair);

    // read equality
    else if (name == "equality")OneEquality(elem, def->equality);

    // read tendon
    else if (name == "tendon")OneTendon(elem, def->tendon);

    // read actuator
    else if (name == "general"     ||
             name == "motor"       ||
             name == "position"    ||
             name == "velocity"    ||
             name == "damper"      ||
             name == "intvelocity" ||
             name == "cylinder"    ||
             name == "muscle"      ||
             name == "adhesion") {
      OneActuator(elem, def->actuator);
    }

    // advance
    elem = NextSiblingElement(elem);
  }

  // iterate over nested defaults
  elem = FirstChildElement(section);
  while (elem) {
    // get element name
    name = elem->Value();

    // read default
    if (name == "default") {
      Default(elem, def, vfs);
    }

    // advance
    elem = NextSiblingElement(elem);
  }
}



// extension section parser
void mjXReader::Extension(XMLElement* section) {
  XMLElement* elem = FirstChildElement(section);

  while (elem) {
    // get sub-element name
    string_view name = elem->Value();

    if (name == "plugin") {
      string plugin_name;
      ReadAttrTxt(elem, "plugin", plugin_name, /* required = */ true);
      if (mjs_activatePlugin(spec, plugin_name.c_str())) {
        throw mjXError(elem, "plugin %s not found", plugin_name.c_str());
      }

      XMLElement* child = FirstChildElement(elem);
      while (child) {
        if (string(child->Value()) == "instance") {
          if (spec->hasImplicitPluginElem) {
            throw mjXError(
                    child, "explicit plugin instance must appear before implicit plugin elements");
          }
          string name;
          mjsPlugin* p = mjs_addPlugin(spec);
          mjs_setString(p->plugin_name, plugin_name.c_str());
          mjs_setString(p->info, ("line " + std::to_string(elem->GetLineNum())).c_str());
          ReadAttrTxt(child, "name", name, /* required = */ true);
          mjs_setString(p->name, name.c_str());
          if (!p->name) {
            throw mjXError(child, "plugin instance must have a name");
          }
          ReadPluginConfigs(child, p);
        }
        child = NextSiblingElement(child);
      }
    }

    // advance to next element
    elem = NextSiblingElement(elem);
  }
}



// custom section parser
void mjXReader::Custom(XMLElement* section) {
  string str, name;
  XMLElement* elem;
  double data[500];

  // iterate over child elements
  elem = FirstChildElement(section);
  while (elem) {
    // get sub-element name
    name = elem->Value();
    string elname;

    // numeric
    if (name == "numeric") {
      // create custom
      mjsNumeric* numeric = mjs_addNumeric(spec);

      // write error info
      mjs_setString(numeric->info, ("line " + std::to_string(elem->GetLineNum())).c_str());

      // read attributes
      ReadAttrTxt(elem, "name", elname, true);
      if (mjs_setName(numeric->element, elname.c_str())) {
        throw mjXError(elem, "%s", mjs_getError(spec));
      }
      if (ReadAttrInt(elem, "size", &numeric->size)) {
        int sz = numeric->size < 500 ? numeric->size : 500;
        for (int i=0; i < sz; i++) {
          data[i] = 0;
        }
      } else {
        numeric->size = 501;
      }
      int len = ReadAttr(elem, "data", numeric->size, data, str, false, false);
      if (numeric->size == 501) {
        numeric->size = len;
      }
      if (numeric->size < 1 || numeric->size > 500) {
        throw mjXError(elem, "custom field size must be between 1 and 500");
      }

      // copy data
      mjs_setDouble(numeric->data, data, numeric->size);
    }

    // text
    else if (name == "text") {
      // create custom
      mjsText* text = mjs_addText(spec);

      // write error info
      mjs_setString(text->info, ("line " + std::to_string(elem->GetLineNum())).c_str());

      // read attributes
      ReadAttrTxt(elem, "name", elname, true);
      if (mjs_setName(text->element, elname.c_str())) {
        throw mjXError(elem, "%s", mjs_getError(spec));
      }
      ReadAttrTxt(elem, "data", str, true);
      if (str.empty()) {
        throw mjXError(elem, "text field cannot be empty");
      }

      // copy data
      mjs_setString(text->data, str.c_str());
    }

    // tuple
    else if (name == "tuple") {
      // create custom
      mjsTuple* tuple = mjs_addTuple(spec);

      // write error info
      mjs_setString(tuple->info, ("line " + std::to_string(elem->GetLineNum())).c_str());

      // read attributes
      ReadAttrTxt(elem, "name", elname, true);
      if (mjs_setName(tuple->element, elname.c_str())) {
        throw mjXError(elem, "%s", mjs_getError(spec));
      }

      // read objects and add
      XMLElement* obj = FirstChildElement(elem);
      std::vector<int> objtype;
      string objname = "";
      std::vector<double> objprm;

      while (obj) {
        // get sub-element name
        name = obj->Value();

        // new object
        if (name == "element") {
          // read type, check and assign
          ReadAttrTxt(obj, "objtype", str, true);
          mjtObj otype = (mjtObj)mju_str2Type(str.c_str());
          if (otype == mjOBJ_UNKNOWN) {
            throw mjXError(obj, "unknown object type");
          }
          objtype.push_back(otype);

          // read name and assign
          ReadAttrTxt(obj, "objname", str, true);
          objname += " " + str;

          // read parameter and assign
          double oprm = 0;
          ReadAttr(obj, "prm", 1, &oprm, str);
          objprm.push_back(oprm);
        }

        // advance to next object
        obj = NextSiblingElement(obj);
      }

      mjs_setInt(tuple->objtype, objtype.data(), objtype.size());
      mjs_setStringVec(tuple->objname, objname.c_str());
      mjs_setDouble(tuple->objprm, objprm.data(), objprm.size());
    }

    // advance to next element
    elem = NextSiblingElement(elem);
  }
}



// visual section parser
void mjXReader::Visual(XMLElement* section) {
  string text, name;
  XMLElement* elem;
  mjVisual* vis = &spec->visual;
  int n;

  // iterate over child elements
  elem = FirstChildElement(section);
  while (elem) {
    // get sub-element name
    name = elem->Value();

    // global sub-element
    if (name == "global") {
      ReadAttrInt(elem, "cameraid",     &vis->global.cameraid);
      if (MapValue(elem, "orthographic", &n, bool_map, 2)) {
        vis->global.orthographic = (n == 1);
      }
      ReadAttr(elem,    "fovy",      1, &vis->global.fovy,      text);
      ReadAttr(elem,    "ipd",       1, &vis->global.ipd,       text);
      ReadAttr(elem,    "azimuth",   1, &vis->global.azimuth,   text);
      ReadAttr(elem,    "elevation", 1, &vis->global.elevation, text);
      ReadAttr(elem,    "linewidth", 1, &vis->global.linewidth, text);
      ReadAttr(elem,    "glow",      1, &vis->global.glow,      text);
      ReadAttrInt(elem, "offwidth",     &vis->global.offwidth);
      ReadAttrInt(elem, "offheight",    &vis->global.offheight);
      if (ReadAttr(elem, "realtime", 1, &vis->global.realtime, text)) {
        if (vis->global.realtime <= 0) {
          throw mjXError(elem, "realtime must be greater than 0");
        }
      }
      if (MapValue(elem, "ellipsoidinertia", &n, bool_map, 2)) {
        vis->global.ellipsoidinertia = (n == 1);
      }
      if (MapValue(elem, "bvactive", &n, bool_map, 2)) {
        vis->global.bvactive = (n == 1);
      }
    }

    // quality sub-element
    else if (name == "quality") {
      ReadAttrInt(elem, "shadowsize", &vis->quality.shadowsize);
      ReadAttrInt(elem, "offsamples", &vis->quality.offsamples);
      ReadAttrInt(elem, "numslices",  &vis->quality.numslices);
      ReadAttrInt(elem, "numstacks",  &vis->quality.numstacks);
      ReadAttrInt(elem, "numquads",   &vis->quality.numquads);
    }

    // headlight sub-element
    else if (name == "headlight") {
      ReadAttr(elem, "ambient",  3, vis->headlight.ambient,  text);
      ReadAttr(elem, "diffuse",  3, vis->headlight.diffuse,  text);
      ReadAttr(elem, "specular", 3, vis->headlight.specular, text);
      ReadAttrInt(elem, "active",  &vis->headlight.active);
    }

    // map sub-element
    else if (name == "map") {
      ReadAttr(elem, "stiffness",      1, &vis->map.stiffness, text);
      ReadAttr(elem, "stiffnessrot",   1, &vis->map.stiffnessrot, text);
      ReadAttr(elem, "force",          1, &vis->map.force,     text);
      ReadAttr(elem, "torque",         1, &vis->map.torque,    text);
      ReadAttr(elem, "alpha",          1, &vis->map.alpha,     text);
      ReadAttr(elem, "fogstart",       1, &vis->map.fogstart,  text);
      ReadAttr(elem, "fogend",         1, &vis->map.fogend,    text);
      ReadAttr(elem, "znear",          1, &vis->map.znear,     text);
      if (vis->map.znear <= 0) {
        throw mjXError(elem, "znear must be strictly positive");
      }
      ReadAttr(elem, "zfar",           1, &vis->map.zfar,      text);
      ReadAttr(elem, "haze",           1, &vis->map.haze,      text);
      ReadAttr(elem, "shadowclip",     1, &vis->map.shadowclip, text);
      ReadAttr(elem, "shadowscale",    1, &vis->map.shadowscale, text);
      ReadAttr(elem, "actuatortendon", 1, &vis->map.actuatortendon, text);
    }

    // scale sub-element
    else if (name == "scale") {
      ReadAttr(elem, "forcewidth",     1, &vis->scale.forcewidth,     text);
      ReadAttr(elem, "contactwidth",   1, &vis->scale.contactwidth,   text);
      ReadAttr(elem, "contactheight",  1, &vis->scale.contactheight,  text);
      ReadAttr(elem, "connect",        1, &vis->scale.connect,        text);
      ReadAttr(elem, "com",            1, &vis->scale.com,            text);
      ReadAttr(elem, "camera",         1, &vis->scale.camera,         text);
      ReadAttr(elem, "light",          1, &vis->scale.light,          text);
      ReadAttr(elem, "selectpoint",    1, &vis->scale.selectpoint,    text);
      ReadAttr(elem, "jointlength",    1, &vis->scale.jointlength,    text);
      ReadAttr(elem, "jointwidth",     1, &vis->scale.jointwidth,     text);
      ReadAttr(elem, "actuatorlength", 1, &vis->scale.actuatorlength, text);
      ReadAttr(elem, "actuatorwidth",  1, &vis->scale.actuatorwidth,  text);
      ReadAttr(elem, "framelength",    1, &vis->scale.framelength,    text);
      ReadAttr(elem, "framewidth",     1, &vis->scale.framewidth,     text);
      ReadAttr(elem, "constraint",     1, &vis->scale.constraint,     text);
      ReadAttr(elem, "slidercrank",    1, &vis->scale.slidercrank,    text);
      ReadAttr(elem, "frustum",        1, &vis->scale.frustum,        text);
    }

    // rgba sub-element
    else if (name == "rgba") {
      ReadAttr(elem, "fog",              4, vis->rgba.fog,             text);
      ReadAttr(elem, "haze",             4, vis->rgba.haze,            text);
      ReadAttr(elem, "force",            4, vis->rgba.force,           text);
      ReadAttr(elem, "inertia",          4, vis->rgba.inertia,         text);
      ReadAttr(elem, "joint",            4, vis->rgba.joint,           text);
      ReadAttr(elem, "actuator",         4, vis->rgba.actuator,        text);
      ReadAttr(elem, "actuatornegative", 4, vis->rgba.actuatornegative, text);
      ReadAttr(elem, "actuatorpositive", 4, vis->rgba.actuatorpositive, text);
      ReadAttr(elem, "com",              4, vis->rgba.com,             text);
      ReadAttr(elem, "camera",           4, vis->rgba.camera,          text);
      ReadAttr(elem, "light",            4, vis->rgba.light,           text);
      ReadAttr(elem, "selectpoint",      4, vis->rgba.selectpoint,     text);
      ReadAttr(elem, "connect",          4, vis->rgba.connect,         text);
      ReadAttr(elem, "contactpoint",     4, vis->rgba.contactpoint,    text);
      ReadAttr(elem, "contactforce",     4, vis->rgba.contactforce,    text);
      ReadAttr(elem, "contactfriction",  4, vis->rgba.contactfriction, text);
      ReadAttr(elem, "contacttorque",    4, vis->rgba.contacttorque,   text);
      ReadAttr(elem, "contactgap",       4, vis->rgba.contactgap,      text);
      ReadAttr(elem, "rangefinder",      4, vis->rgba.rangefinder,     text);
      ReadAttr(elem, "constraint",       4, vis->rgba.constraint,      text);
      ReadAttr(elem, "slidercrank",      4, vis->rgba.slidercrank,     text);
      ReadAttr(elem, "crankbroken",      4, vis->rgba.crankbroken,     text);
      ReadAttr(elem, "frustum",          4, vis->rgba.frustum,         text);
      ReadAttr(elem, "bv",               4, vis->rgba.bv,              text);
      ReadAttr(elem, "bvactive",         4, vis->rgba.bvactive,        text);
    }

    // advance to next element
    elem = NextSiblingElement(elem);
  }
}



// asset section parser
void mjXReader::Asset(XMLElement* section, const mjVFS* vfs) {
  int n;
  string text, name, texname, content_type;
  XMLElement* elem;

  // iterate over child elements
  elem = FirstChildElement(section);
  while (elem) {
    // get sub-element name
    name = elem->Value();

    // get class if specified, otherwise use default0
    const mjsDefault* def = GetClass(elem);
    if (!def) {
      def = mjs_getSpecDefault(spec);
    }

    // texture sub-element
    if (name == "texture") {
      // create texture
      mjsTexture* texture = mjs_addTexture(spec);

      // write error info
      mjs_setString(texture->info, ("line " + std::to_string(elem->GetLineNum())).c_str());

      // read attributes
      if (MapValue(elem, "type", &n, texture_map, texture_sz)) {
        texture->type = (mjtTexture)n;
      }
      if (MapValue(elem, "colorspace", &n, colorspace_map, colorspace_sz)) {
        texture->colorspace = (mjtColorSpace)n;
      }
      if (ReadAttrTxt(elem, "name", texname)) {
        if (mjs_setName(texture->element, texname.c_str())) {
          throw mjXError(elem, "%s", mjs_getError(spec));
        }
      }
      if (ReadAttrTxt(elem, "content_type", content_type)) {
        mjs_setString(texture->content_type, content_type.c_str());
      }
      auto file = ReadAttrFile(elem, "file", vfs, TextureDir());
      if (file.has_value()) {
        mjs_setString(texture->file, file->c_str());
      }
      ReadAttrInt(elem, "width", &texture->width);
      ReadAttrInt(elem, "height", &texture->height);
      ReadAttrInt(elem, "nchannel", &texture->nchannel);
      ReadAttr(elem, "rgb1", 3, texture->rgb1, text);
      ReadAttr(elem, "rgb2", 3, texture->rgb2, text);
      ReadAttr(elem, "markrgb", 3, texture->markrgb, text);
      ReadAttr(elem, "random", 1, &texture->random, text);
      if (MapValue(elem, "builtin", &n, builtin_map, builtin_sz)) {
        texture->builtin = (mjtBuiltin)n;
      }
      if (MapValue(elem, "mark", &n, mark_map, mark_sz)) {
        texture->mark = (mjtMark)n;
      }
      if (MapValue(elem, "hflip", &n, bool_map, 2)) {
        texture->hflip = (n != 0);
      }
      if (MapValue(elem, "vflip", &n, bool_map, 2)) {
        texture->vflip = (n != 0);
      }

      // grid
      ReadAttr(elem, "gridsize", 2, texture->gridsize, text);
      if (ReadAttrTxt(elem, "gridlayout", text)) {
        // check length
        if (text.length() > 12) {
          throw mjXError(elem, "gridlayout length cannot exceed 12 characters");
        }
        if (text.length() != texture->gridsize[0]*texture->gridsize[1]) {
          throw mjXError(elem, "gridlayout length must match gridsize");
        }

        memcpy(texture->gridlayout, text.data(), text.length());
      }

      // separate files
      std::vector<string> cubefiles(6);
      std::vector<string> cubefile_names = {"fileright", "fileleft",
                                            "fileup", "filedown",
                                            "filefront", "fileback"};
      for (int i = 0; i < cubefiles.size(); i++) {
        auto maybe_file = ReadAttrFile(elem, cubefile_names[i].c_str(), vfs,
                                       TextureDir());
        if (maybe_file.has_value()) {
          cubefiles[i] = maybe_file.value().Str();
        } else {
          cubefiles[i] = "";
        }
        mjs_setInStringVec(texture->cubefiles, i, cubefiles[i].c_str());
      }
    }

    // material sub-element
    else if (name == "material") {
      // create material and parse
      mjsMaterial* material = mjs_addMaterial(spec, def);
      OneMaterial(elem, material);
    }

    // mesh sub-element
    else if (name == "mesh") {
      // create mesh and parse
      mjsMesh* mesh = mjs_addMesh(spec, def);
      OneMesh(elem, mesh, vfs);
    }

    // skin sub-element... deprecate ???
    else if (name == "skin") {
      // create skin and parse
      mjsSkin* skin = mjs_addSkin(spec);
      OneSkin(elem, skin, vfs);
    }

    // hfield sub-element
    else if (name == "hfield") {
      // create hfield
      mjsHField* hfield = mjs_addHField(spec);

      // write error info
      mjs_setString(hfield->info, ("line " + std::to_string(elem->GetLineNum())).c_str());

      // read attributes
      string name, content_type;
      if (ReadAttrTxt(elem, "name", name)) {
        if (mjs_setName(hfield->element, name.c_str())) {
          throw mjXError(elem, "%s", mjs_getError(spec));
        }
      }
      if (ReadAttrTxt(elem, "content_type", content_type)) {
        mjs_setString(hfield->content_type, content_type.c_str());
      }
      auto file = ReadAttrFile(elem, "file", vfs, AssetDir());
      if (file.has_value()) {
        mjs_setString(hfield->file, file->c_str());
      }
      ReadAttrInt(elem, "nrow", &hfield->nrow);
      ReadAttrInt(elem, "ncol", &hfield->ncol);
      ReadAttr(elem, "size", 4, hfield->size, text, true);

      // allocate buffer for dynamic hfield, copy user data if given
      if (!file.has_value() && hfield->nrow > 0 && hfield->ncol > 0) {
        int nrow = hfield->nrow;
        int ncol = hfield->ncol;

        // read user data
        auto userdata = ReadAttrVec<float>(elem, "elevation");

        // user data given, copy into data
        if (userdata.has_value()) {
          if (userdata->size() != nrow*ncol) {
            throw mjXError(elem, "elevation data length must match nrow*ncol");
          }

          // copy in reverse row order, so XML string is top-to-bottom
          std::vector<float> flipped(nrow*ncol);
          for (int i = 0; i < nrow; i++) {
            int flip = nrow-1-i;
            for (int j = 0; j < ncol; j++) {
              flipped[flip*ncol + j] = userdata->data()[i*ncol + j];
            }
          }

          mjs_setFloat(hfield->userdata, flipped.data(), flipped.size());
        }

        // user data not given, set to 0
        else {
          std::vector<float> zero(nrow*ncol);
          mjs_setFloat(hfield->userdata, zero.data(), zero.size());
        }
      }
    }

    // model sub-element
    else if (name == "model") {
      std::string content_type;
      ReadAttrTxt(elem, "content_type", content_type);

      // parse the child
      mjSpec* child = nullptr;
      std::array<char, 1024> error;
      auto filename = modelfiledir_ + ReadAttrFile(elem, "file", vfs).value();

#ifdef mjUSEUSD
      if (content_type == "text/usd") {
        child = mj_parseUSD(filename.c_str(), vfs, error.data(), error.size());
      } else {
#endif  // mjUSEUSD
        child = mj_parse(filename.c_str(), content_type.c_str(), vfs,
                               error.data(), error.size());
#ifdef mjUSEUSD
      }
#endif  // mjUSEUSD

      if (!child) {
        throw mjXError(elem, "could not parse model file with error: %s", error.data());
      }

      // overwrite model name if given
      string modelname = "";
      if (ReadAttrTxt(elem, "name", modelname)) {
        mjs_setString(child->modelname, modelname.c_str());
      }

      // store child spec in model
      mjs_addSpec(spec, child);
    }

    // advance to next element
    elem = NextSiblingElement(elem);
  }
}



// body/world section parser; recursive
void mjXReader::Body(XMLElement* section, mjsBody* body, mjsFrame* frame,
                     const mjVFS* vfs) {
  string text, name;
  XMLElement* elem;
  int n;

  // sanity check
  if (!body) {
    throw mjXError(section, "null body pointer");
  }

  // no attributes allowed in world body
  if (mjs_getId(body->element) == 0 && section->FirstAttribute() && !frame) {
    throw mjXError(section, "World body cannot have attributes");
  }

  // iterate over sub-elements; attributes set while parsing parent body
  elem = FirstChildElement(section);
  while (elem) {
    // get sub-element name
    name = elem->Value();

    // get class if specified, otherwise use body
    const mjsDefault* def = GetClass(elem);
    if (!def) {
      def = mjs_getDefault(frame ? frame->element : body->element);
    }

    // inertial sub-element
    if (name == "inertial") {
      // no inertia allowed in world body
      if (mjs_getId(body->element) == 0) {
        throw mjXError(elem, "World body cannot have inertia");
      }
      body->explicitinertial = true;
      ReadAttr(elem, "pos", 3, body->ipos, text, true);
      ReadQuat(elem, "quat", body->iquat, text);
      ReadAttr(elem, "mass", 1, &body->mass, text, true);
      ReadAttr(elem, "diaginertia", 3, body->inertia, text);
      bool alt = ReadAlternative(elem, body->ialt);
      bool full = ReadAttr(elem, "fullinertia", 6, body->fullinertia, text);
      if (alt && full) {
        throw mjXError(elem, "fullinertia and inertial orientation cannot both be specified");
      }
    }

    // joint sub-element
    else if (name == "joint") {
      // no joints allowed in world body
      if (mjs_getId(body->element) == 0) {
        throw mjXError(elem, "World body cannot have joints");
      }

      // create joint and parse
      mjsJoint* joint = mjs_addJoint(body, def);
      OneJoint(elem, joint);
      mjs_setFrame(joint->element, frame);
    }

    // freejoint sub-element
    else if (name == "freejoint") {
      // no joints allowed in world body
      if (mjs_getId(body->element) == 0) {
        throw mjXError(elem, "World body cannot have joints");
      }

      // create free joint without defaults
      mjsJoint* joint = mjs_addFreeJoint(body);
      mjs_setFrame(joint->element, frame);

      // save defaults after creation, to make sure writing is ok
      mjs_setDefault(joint->element, def);

      // read attributes
      string name;
      if (ReadAttrTxt(elem, "name", name)) {
        if (mjs_setName(joint->element, name.c_str())) {
          throw mjXError(elem, "%s", mjs_getError(spec));
        }
      }
      ReadAttrInt(elem, "group", &joint->group);
      MapValue(elem, "align", &joint->align, TFAuto_map, 3);
    }

    // geom sub-element
    else if (name == "geom") {
      // create geom and parse
      mjsGeom* geom = mjs_addGeom(body, def);
      OneGeom(elem, geom);
      mjs_setFrame(geom->element, frame);
    }

    // site sub-element
    else if (name == "site") {
      // create site and parse
      mjsSite* site = mjs_addSite(body,  def);
      OneSite(elem, site);
      mjs_setFrame(site->element, frame);
    }

    // camera sub-element
    else if (name == "camera") {
      // create camera and parse
      mjsCamera* camera = mjs_addCamera(body, def);
      OneCamera(elem, camera);
      mjs_setFrame(camera->element, frame);
    }

    // light sub-element
    else if (name == "light") {
      // create light and parse
      mjsLight* light = mjs_addLight(body, def);
      OneLight(elem, light);
      mjs_setFrame(light->element, frame);
    }

    // plugin sub-element
    else if (name == "plugin") {
      OnePlugin(elem, &(body->plugin));
    }

    // composite sub-element
    else if (name == "composite") {
      // parse composite
      OneComposite(elem, body, frame, def);
    }

    // flexcomp sub-element
    else if (name == "flexcomp") {
      // parse flexcomp
      OneFlexcomp(elem, body, vfs);
    }

    // frame sub-element
    else if (name == "frame") {
      // read childdef
      bool has_childclass = ReadAttrTxt(elem, "childclass", text);
      const mjsDefault* childdef = has_childclass ? mjs_findDefault(spec, text.c_str()) : nullptr;
      if (has_childclass && !childdef) {
        throw mjXError(elem, "unknown default childclass");
      }

      // create frame
      mjsFrame* pframe = mjs_addFrame(body, frame);
      mjs_setString(pframe->info, ("line " + std::to_string(elem->GetLineNum())).c_str());
      mjs_setDefault(pframe->element, childdef ? childdef : def);

      // read attributes
      string name, childclass;
      if (ReadAttrTxt(elem, "name", name)) {
        if (mjs_setName(pframe->element, name.c_str())) {
          throw mjXError(elem, "%s", mjs_getError(spec));
        }
      }
      if (ReadAttrTxt(elem, "childclass", childclass)) {
        mjs_setString(pframe->childclass, childclass.c_str());
      }
      ReadAttr(elem, "pos", 3, pframe->pos, text);
      ReadQuat(elem, "quat", pframe->quat, text);
      ReadAlternative(elem, pframe->alt);

      Body(elem, body, pframe, vfs);
    }

    // replicate sub-element
    else if (name == "replicate") {
      int count;
      double offset[3] = {0, 0, 0};
      double euler[3] = {0, 0, 0};
      string separator = "";
      ReadAttr(elem, "count", 1, &count, text, true);
      ReadAttr(elem, "offset", 3, offset, text);
      ReadAttr(elem, "euler", 3, euler, text);
      ReadAttrTxt(elem, "sep", separator);

      // store rotation difference
      mjsOrientation alt;
      mjs_defaultOrientation(&alt);
      alt.type = mjORIENTATION_EULER;
      mjuu_copyvec(alt.euler, euler, 3);
      double rotation[4] = {1, 0, 0, 0};
      mjs_resolveOrientation(rotation, spec->compiler.degree, spec->compiler.eulerseq, &alt);

      // read childdef
      bool has_childclass = ReadAttrTxt(elem, "childclass", text);
      const mjsDefault* childdef = has_childclass ? mjs_findDefault(spec, text.c_str()) : nullptr;
      if (has_childclass && !childdef) {
        throw mjXError(elem, "unknown default childclass");
      }

      // create subtree
      mjsBody* subtree = mjs_addBody(body, childdef);
      double pos[3] = {0, 0, 0};
      double quat[4] = {1, 0, 0, 0};

      // parent frame that will be used to attach the subtree
      mjsFrame* pframe = mjs_addFrame(subtree, frame);
      mjs_setDefault(pframe->element, childdef ? childdef : def);
      mjs_setString(pframe->info, ("line = " + std::to_string(elem->GetLineNum())).c_str());

      // parse subtree
      Body(elem, subtree, pframe, vfs);

      // update pframe and attach
      for (int i = 0; i < count; i++) {
        // overwrite orientation to increase precision
        alt.euler[0] = i*euler[0];
        alt.euler[1] = i*euler[1];
        alt.euler[2] = i*euler[2];
        mjs_resolveOrientation(quat, spec->compiler.degree, spec->compiler.eulerseq, &alt);

        // set position and orientation
        mjuu_setvec(pframe->pos, pos[0], pos[1], pos[2]);
        mjuu_setvec(pframe->quat, quat[0], quat[1], quat[2], quat[3]);

        // accumulate rotation
        mjuu_frameaccum(pos, quat, offset, rotation);

        // process suffix
        string suffix = separator;
        UpdateString(suffix, count, i);

        // attach to parent
        if (!mjs_attach(body->element, pframe->element, /*prefix=*/"", suffix.c_str())) {
          throw mjXError(elem, "%s", mjs_getError(spec));
        }
      }

      // delete subtree
      if (mjs_delete(spec, subtree->element)) {
        throw mjXError(elem, "%s", mjs_getError(spec));
      }
    }

    // body sub-element
    else if (name == "body") {
      // read childdef
      bool has_childclass = ReadAttrTxt(elem, "childclass", text);
      const mjsDefault* childdef = has_childclass ? mjs_findDefault(spec, text.c_str()) : nullptr;
      if (has_childclass && !childdef) {
        throw mjXError(elem, "unknown default childclass");
      }

      // create child body
      mjsBody* child = mjs_addBody(body, childdef);
      mjs_setString(child->info, string("line " + std::to_string(elem->GetLineNum())).c_str());

      // set default from class or childclass
      mjs_setDefault(child->element, childdef ? childdef : def);

      // read attributes
      string name, childclass;
      if (ReadAttrTxt(elem, "name", name)) {
        if (mjs_setName(child->element, name.c_str())) {
          throw mjXError(elem, "%s", mjs_getError(spec));
        }
      }
      if (ReadAttrTxt(elem, "childclass", childclass)) {
        mjs_setString(child->childclass, childclass.c_str());
      }
      ReadAttr(elem, "pos", 3, child->pos, text);
      ReadQuat(elem, "quat", child->quat, text);
      if (MapValue(elem, "mocap", &n, bool_map, 2)) {
        child->mocap = (n == 1);
      }
      ReadAlternative(elem, child->alt);

      // gravcomp, sleep policy
      ReadAttr(elem, "gravcomp", 1, &child->gravcomp, text);
      if (MapValue(elem, "sleep", &n, bodysleep_map, bodysleep_sz)) {
        child->sleep = (mjtSleepPolicy) n;
      }

      // read userdata
      std::vector<double> userdata;
      ReadVector(elem, "user", userdata, text);
      mjs_setDouble(child->userdata, userdata.data(), userdata.size());

      // add frame
      mjs_setFrame(child->element, frame);

      // make recursive call
      Body(elem, child, nullptr, vfs);
    }

    // attachment
    else if (name == "attach") {
      string model_name, body_name, prefix;
      ReadAttrTxt(elem, "model", model_name, /*required=*/true);
      ReadAttrTxt(elem, "body", body_name, /*required=*/false);
      ReadAttrTxt(elem, "prefix", prefix, /*required=*/true);

      mjsBody* child_body = mjs_findBody(spec, (prefix+body_name).c_str());
      mjsFrame* pframe = frame ? frame : mjs_addFrame(body, nullptr);

      if (!child_body) {
        mjSpec* asset = mjs_findSpec(spec, model_name.c_str());
        if (!asset) {
          throw mjXError(elem, "could not find model '%s'", model_name.c_str());
        }
        mjsElement* child;
        if (body_name.empty()) {
          child = asset->element;
        } else {
          child_body = mjs_findBody(asset, body_name.c_str());
          if (!child_body) {
            throw mjXError(elem, "could not find body '%s''%s'", body_name.c_str());
          }
          child = child_body->element;
        }
        if (!mjs_attach(pframe->element, child, prefix.c_str(), "")) {
          throw mjXError(elem, "%s", mjs_getError(spec));
        }
      } else {
        // only set frame to existing body
        if (mjs_setFrame(child_body->element, pframe)) {
          throw mjXError(elem, "%s", mjs_getError(spec));
        }
      }
    }

    // no match
    else {
      throw mjXError(elem, "unrecognized model element '%s'", name.c_str());
    }

    // advance to next element
    elem = NextSiblingElement(elem);
  }
}



// contact section parser
void mjXReader::Contact(XMLElement* section) {
  string text, name;
  XMLElement* elem;

  // iterate over child elements
  elem = FirstChildElement(section);
  while (elem) {
    // get sub-element name
    name = elem->Value();

    // get class if specified, otherwise use default0
    const mjsDefault* def = GetClass(elem);
    if (!def) {
      def = mjs_getSpecDefault(spec);
    }

    // geom pair to include
    if (name == "pair") {
      // create pair and parse
      mjsPair* pair = mjs_addPair(spec, def);
      OnePair(elem, pair);
    }

    // body pair to exclude
    else if (name == "exclude") {
      mjsExclude* exclude = mjs_addExclude(spec);
      string exname, exbody1, exbody2;

      // write error info
      mjs_setString(exclude->info, ("line " + std::to_string(elem->GetLineNum())).c_str());

      // read name and body names
      if (ReadAttrTxt(elem, "name", exname)) {
        if (mjs_setName(exclude->element, exname.c_str())) {
          throw mjXError(elem, "%s", mjs_getError(spec));
        }
      }
      ReadAttrTxt(elem, "body1", exbody1, true);
      mjs_setString(exclude->bodyname1, exbody1.c_str());
      ReadAttrTxt(elem, "body2", exbody2, true);
      mjs_setString(exclude->bodyname2, exbody2.c_str());
    }

    // advance to next element
    elem = NextSiblingElement(elem);
  }
}



// constraint section parser
void mjXReader::Equality(XMLElement* section) {
  XMLElement* elem;

  // iterate over child elements
  elem = FirstChildElement(section);
  while (elem) {
    // get class if specified, otherwise use default0
    const mjsDefault* def = GetClass(elem);
    if (!def) {
      def = mjs_getSpecDefault(spec);
    }

    // create equality constraint and parse
    mjsEquality* equality = mjs_addEquality(spec, def);
    OneEquality(elem, equality);

    // advance to next element
    elem = NextSiblingElement(elem);
  }
}



// deformable section parser
void mjXReader::Deformable(XMLElement* section, const mjVFS* vfs) {
  string name;
  XMLElement* elem;

  // iterate over child elements
  elem = FirstChildElement(section);
  while (elem) {
    // get sub-element name
    name = elem->Value();

    // get class if specified, otherwise use default0
    const mjsDefault* def = GetClass(elem);
    if (!def) {
      def = mjs_getSpecDefault(spec);
    }

    // flex sub-element
    if (name == "flex") {
      // create flex and parse
      mjsFlex* flex = mjs_addFlex(spec);
      OneFlex(elem, flex);
    }

    // skin sub-element
    else if (name == "skin") {
      // create skin and parse
      mjsSkin* skin = mjs_addSkin(spec);
      OneSkin(elem, skin, vfs);
    }

    // advance to next element
    elem = NextSiblingElement(elem);
  }
}



// tendon section parser
void mjXReader::Tendon(XMLElement* section) {
  string text, text1;
  XMLElement* elem;
  double data;

  // iterate over child elements
  elem = FirstChildElement(section);
  while (elem) {
    // get class if specified, otherwise use default0
    const mjsDefault* def = GetClass(elem);
    if (!def) {
      def = mjs_getSpecDefault(spec);
    }

    // create tendon and parse
    mjsTendon* tendon = mjs_addTendon(spec, def);
    OneTendon(elem, tendon);

    // process wrap sub-elements
    XMLElement* sub = FirstChildElement(elem);
    while (sub) {
      // get wrap type
      string type = sub->Value();
      mjsWrap* wrap;;

      // read attributes depending on type
      if (type == "site") {
        ReadAttrTxt(sub, "site", text, true);
        wrap = mjs_wrapSite(tendon, text.c_str());
      }

      else if (type == "geom") {
        ReadAttrTxt(sub, "geom", text, true);
        if (!ReadAttrTxt(sub, "sidesite", text1)) {
          text1.clear();
        }
        wrap = mjs_wrapGeom(tendon, text.c_str(), text1.c_str());
      }

      else if (type == "pulley") {
        ReadAttr(sub, "divisor", 1, &data, text, true);
        wrap = mjs_wrapPulley(tendon, data);
      }

      else if (type == "joint") {
        ReadAttrTxt(sub, "joint", text, true);
        ReadAttr(sub, "coef", 1, &data, text1, true);
        wrap = mjs_wrapJoint(tendon, text.c_str(), data);
      }

      else {
        throw mjXError(sub, "unknown wrap type");  // SHOULD NOT OCCUR
      }

      mjs_setString(wrap->info, ("line " + std::to_string(sub->GetLineNum())).c_str());

      // advance to next sub-element
      sub = NextSiblingElement(sub);
    }

    // advance to next element
    elem = NextSiblingElement(elem);
  }
}



// actuator section parser
void mjXReader::Actuator(XMLElement* section) {
  XMLElement* elem;

  // iterate over child elements
  elem = FirstChildElement(section);
  while (elem) {
    // get class if specified, otherwise use default0
    const mjsDefault* def = GetClass(elem);
    if (!def) {
      def = mjs_getSpecDefault(spec);
    }

    // create actuator and parse
    mjsActuator* actuator = mjs_addActuator(spec, def);
    OneActuator(elem, actuator);

    // advance to next element
    elem = NextSiblingElement(elem);
  }
}



// sensor section parser
void mjXReader::Sensor(XMLElement* section) {
  int n;
  XMLElement* elem = FirstChildElement(section);
  while (elem) {
    // create sensor, get string type
    mjsSensor* sensor = mjs_addSensor(spec);
    string type = elem->Value();
    string text, name, objname, refname;
    std::vector<double> userdata;

    // read name, noise, userdata
    if (ReadAttrTxt(elem, "name", name)) {
      if (mjs_setName(sensor->element, name.c_str())) {
        throw mjXError(elem, "%s", mjs_getError(spec));
      }
    }
    ReadAttr(elem, "cutoff", 1, &sensor->cutoff, text);
    ReadAttr(elem, "noise", 1, &sensor->noise, text);
    if (ReadVector(elem, "user", userdata, text)) {
      mjs_setDouble(sensor->userdata, userdata.data(), userdata.size());
    }

    // common robotic sensors, attached to a site
    if (type == "touch") {
      sensor->type = mjSENS_TOUCH;
      sensor->objtype = mjOBJ_SITE;
      ReadAttrTxt(elem, "site", objname, true);
    } else if (type == "accelerometer") {
      sensor->type = mjSENS_ACCELEROMETER;
      sensor->objtype = mjOBJ_SITE;
      ReadAttrTxt(elem, "site", objname, true);
    } else if (type == "velocimeter") {
      sensor->type = mjSENS_VELOCIMETER;
      sensor->objtype = mjOBJ_SITE;
      ReadAttrTxt(elem, "site", objname, true);
    } else if (type == "gyro") {
      sensor->type = mjSENS_GYRO;
      sensor->objtype = mjOBJ_SITE;
      ReadAttrTxt(elem, "site", objname, true);
    } else if (type == "force") {
      sensor->type = mjSENS_FORCE;
      sensor->objtype = mjOBJ_SITE;
      ReadAttrTxt(elem, "site", objname, true);
    } else if (type == "torque") {
      sensor->type = mjSENS_TORQUE;
      sensor->objtype = mjOBJ_SITE;
      ReadAttrTxt(elem, "site", objname, true);
    } else if (type == "magnetometer") {
      sensor->type = mjSENS_MAGNETOMETER;
      sensor->objtype = mjOBJ_SITE;
      ReadAttrTxt(elem, "site", objname, true);
    } else if (type == "camprojection") {
      sensor->type = mjSENS_CAMPROJECTION;
      sensor->objtype = mjOBJ_SITE;
      ReadAttrTxt(elem, "site", objname, true);
      ReadAttrTxt(elem, "camera", refname, true);
      sensor->reftype = mjOBJ_CAMERA;
    } else if (type == "rangefinder") {
      sensor->type = mjSENS_RANGEFINDER;
      sensor->objtype = mjOBJ_SITE;
      ReadAttrTxt(elem, "site", objname, true);
    }

    // sensors related to scalar joints, tendons, actuators
    else if (type == "jointpos") {
      sensor->type = mjSENS_JOINTPOS;
      sensor->objtype = mjOBJ_JOINT;
      ReadAttrTxt(elem, "joint", objname, true);
    } else if (type == "jointvel") {
      sensor->type = mjSENS_JOINTVEL;
      sensor->objtype = mjOBJ_JOINT;
      ReadAttrTxt(elem, "joint", objname, true);
    } else if (type == "tendonpos") {
      sensor->type = mjSENS_TENDONPOS;
      sensor->objtype = mjOBJ_TENDON;
      ReadAttrTxt(elem, "tendon", objname, true);
    } else if (type == "tendonvel") {
      sensor->type = mjSENS_TENDONVEL;
      sensor->objtype = mjOBJ_TENDON;
      ReadAttrTxt(elem, "tendon", objname, true);
    } else if (type == "actuatorpos") {
      sensor->type = mjSENS_ACTUATORPOS;
      sensor->objtype = mjOBJ_ACTUATOR;
      ReadAttrTxt(elem, "actuator", objname, true);
    } else if (type == "actuatorvel") {
      sensor->type = mjSENS_ACTUATORVEL;
      sensor->objtype = mjOBJ_ACTUATOR;
      ReadAttrTxt(elem, "actuator", objname, true);
    } else if (type == "actuatorfrc") {
      sensor->type = mjSENS_ACTUATORFRC;
      sensor->objtype = mjOBJ_ACTUATOR;
      ReadAttrTxt(elem, "actuator", objname, true);
    } else if (type == "jointactuatorfrc") {
      sensor->type = mjSENS_JOINTACTFRC;
      sensor->objtype = mjOBJ_JOINT;
      ReadAttrTxt(elem, "joint", objname, true);
    } else if (type=="tendonactuatorfrc") {
      sensor->type = mjSENS_TENDONACTFRC;
      sensor->objtype = mjOBJ_TENDON;
      ReadAttrTxt(elem, "tendon", objname, true);
    }

    // sensors related to ball joints
    else if (type == "ballquat") {
      sensor->type = mjSENS_BALLQUAT;
      sensor->objtype = mjOBJ_JOINT;
      ReadAttrTxt(elem, "joint", objname, true);
    } else if (type == "ballangvel") {
      sensor->type = mjSENS_BALLANGVEL;
      sensor->objtype = mjOBJ_JOINT;
      ReadAttrTxt(elem, "joint", objname, true);
    }

    // joint and tendon limit sensors
    else if (type == "jointlimitpos") {
      sensor->type = mjSENS_JOINTLIMITPOS;
      sensor->objtype = mjOBJ_JOINT;
      ReadAttrTxt(elem, "joint", objname, true);
    } else if (type == "jointlimitvel") {
      sensor->type = mjSENS_JOINTLIMITVEL;
      sensor->objtype = mjOBJ_JOINT;
      ReadAttrTxt(elem, "joint", objname, true);
    } else if (type == "jointlimitfrc") {
      sensor->type = mjSENS_JOINTLIMITFRC;
      sensor->objtype = mjOBJ_JOINT;
      ReadAttrTxt(elem, "joint", objname, true);
    } else if (type == "tendonlimitpos") {
      sensor->type = mjSENS_TENDONLIMITPOS;
      sensor->objtype = mjOBJ_TENDON;
      ReadAttrTxt(elem, "tendon", objname, true);
    } else if (type == "tendonlimitvel") {
      sensor->type = mjSENS_TENDONLIMITVEL;
      sensor->objtype = mjOBJ_TENDON;
      ReadAttrTxt(elem, "tendon", objname, true);
    } else if (type == "tendonlimitfrc") {
      sensor->type = mjSENS_TENDONLIMITFRC;
      sensor->objtype = mjOBJ_TENDON;
      ReadAttrTxt(elem, "tendon", objname, true);
    }

    // sensors attached to an object with spatial frame: (x)body, geom, site, camera
    else if (type == "framepos") {
      sensor->type = mjSENS_FRAMEPOS;
      ReadAttrTxt(elem, "objtype", text, true);
      sensor->objtype = (mjtObj)mju_str2Type(text.c_str());
      ReadAttrTxt(elem, "objname", objname, true);
      if (ReadAttrTxt(elem, "reftype", text)) {
        sensor->reftype = (mjtObj)mju_str2Type(text.c_str());
        ReadAttrTxt(elem, "refname", refname, true);
      } else if (ReadAttrTxt(elem, "refname", text)) {
        throw mjXError(elem, "refname '%s' given but reftype is missing", text.c_str());
      }
    } else if (type == "framequat") {
      sensor->type = mjSENS_FRAMEQUAT;
      ReadAttrTxt(elem, "objtype", text, true);
      sensor->objtype = (mjtObj)mju_str2Type(text.c_str());
      ReadAttrTxt(elem, "objname", objname, true);
      if (ReadAttrTxt(elem, "reftype", text)) {
        sensor->reftype = (mjtObj)mju_str2Type(text.c_str());
        ReadAttrTxt(elem, "refname", refname, true);
      } else if (ReadAttrTxt(elem, "refname", text)) {
        throw mjXError(elem, "refname '%s' given but reftype is missing", text.c_str());
      }
    } else if (type == "framexaxis") {
      sensor->type = mjSENS_FRAMEXAXIS;
      ReadAttrTxt(elem, "objtype", text, true);
      sensor->objtype = (mjtObj)mju_str2Type(text.c_str());
      ReadAttrTxt(elem, "objname", objname, true);
      if (ReadAttrTxt(elem, "reftype", text)) {
        sensor->reftype = (mjtObj)mju_str2Type(text.c_str());
        ReadAttrTxt(elem, "refname", refname, true);
      } else if (ReadAttrTxt(elem, "refname", text)) {
        throw mjXError(elem, "refname '%s' given but reftype is missing", text.c_str());
      }
    } else if (type == "frameyaxis") {
      sensor->type = mjSENS_FRAMEYAXIS;
      ReadAttrTxt(elem, "objtype", text, true);
      sensor->objtype = (mjtObj)mju_str2Type(text.c_str());
      ReadAttrTxt(elem, "objname", objname, true);
      if (ReadAttrTxt(elem, "reftype", text)) {
        sensor->reftype = (mjtObj)mju_str2Type(text.c_str());
        ReadAttrTxt(elem, "refname", refname, true);
      } else if (ReadAttrTxt(elem, "refname", text)) {
        throw mjXError(elem, "refname '%s' given but reftype is missing", text.c_str());
      }
    } else if (type == "framezaxis") {
      sensor->type = mjSENS_FRAMEZAXIS;
      ReadAttrTxt(elem, "objtype", text, true);
      sensor->objtype = (mjtObj)mju_str2Type(text.c_str());
      ReadAttrTxt(elem, "objname", objname, true);
      if (ReadAttrTxt(elem, "reftype", text)) {
        sensor->reftype = (mjtObj)mju_str2Type(text.c_str());
        ReadAttrTxt(elem, "refname", refname, true);
      } else if (ReadAttrTxt(elem, "refname", text)) {
        throw mjXError(elem, "refname '%s' given but reftype is missing", text.c_str());
      }
    } else if (type == "framelinvel") {
      sensor->type = mjSENS_FRAMELINVEL;
      ReadAttrTxt(elem, "objtype", text, true);
      sensor->objtype = (mjtObj)mju_str2Type(text.c_str());
      ReadAttrTxt(elem, "objname", objname, true);
      if (ReadAttrTxt(elem, "reftype", text)) {
        sensor->reftype = (mjtObj)mju_str2Type(text.c_str());
        ReadAttrTxt(elem, "refname", refname, true);
      } else if (ReadAttrTxt(elem, "refname", text)) {
        throw mjXError(elem, "refname '%s' given but reftype is missing", text.c_str());
      }
    } else if (type == "frameangvel") {
      sensor->type = mjSENS_FRAMEANGVEL;
      ReadAttrTxt(elem, "objtype", text, true);
      sensor->objtype = (mjtObj)mju_str2Type(text.c_str());
      ReadAttrTxt(elem, "objname", objname, true);
      if (ReadAttrTxt(elem, "reftype", text)) {
        sensor->reftype = (mjtObj)mju_str2Type(text.c_str());
        ReadAttrTxt(elem, "refname", refname, true);
      } else if (ReadAttrTxt(elem, "refname", text)) {
        throw mjXError(elem, "refname '%s' given but reftype is missing", text.c_str());
      }
    } else if (type == "framelinacc") {
      sensor->type = mjSENS_FRAMELINACC;
      ReadAttrTxt(elem, "objtype", text, true);
      sensor->objtype = (mjtObj)mju_str2Type(text.c_str());
      ReadAttrTxt(elem, "objname", objname, true);
    } else if (type == "frameangacc") {
      sensor->type = mjSENS_FRAMEANGACC;
      ReadAttrTxt(elem, "objtype", text, true);
      sensor->objtype = (mjtObj)mju_str2Type(text.c_str());
      ReadAttrTxt(elem, "objname", objname, true);
    } else if (type == "insidesite") {
      sensor->type = mjSENS_INSIDESITE;
      sensor->reftype = mjOBJ_SITE;
      ReadAttrTxt(elem, "site", refname, true);
      ReadAttrTxt(elem, "objtype", text, true);
      sensor->objtype = (mjtObj)mju_str2Type(text.c_str());
      ReadAttrTxt(elem, "objname", objname, true);
    }

    // sensors related to kinematic subtrees; attached to a body (which is the subtree root)
    else if (type == "subtreecom") {
      sensor->type = mjSENS_SUBTREECOM;
      sensor->objtype = mjOBJ_BODY;
      ReadAttrTxt(elem, "body", objname, true);
    } else if (type == "subtreelinvel") {
      sensor->type = mjSENS_SUBTREELINVEL;
      sensor->objtype = mjOBJ_BODY;
      ReadAttrTxt(elem, "body", objname, true);
    } else if (type == "subtreeangmom") {
      sensor->type = mjSENS_SUBTREEANGMOM;
      sensor->objtype = mjOBJ_BODY;
      ReadAttrTxt(elem, "body", objname, true);
    }

    // sensors for geometric distance; attached to geoms or bodies
    else if (type == "distance" || type == "normal" || type == "fromto") {
      bool has_body1 = ReadAttrTxt(elem, "body1", objname);
      bool has_geom1 = ReadAttrTxt(elem, "geom1", objname);
      if (has_body1 == has_geom1) {
        throw mjXError(elem, "exactly one of (geom1, body1) must be specified");
      }
      sensor->objtype = has_body1 ? mjOBJ_BODY : mjOBJ_GEOM;
      bool has_body2 = ReadAttrTxt(elem, "body2", refname);
      bool has_geom2 = ReadAttrTxt(elem, "geom2", refname);
      if (has_body2 == has_geom2) {
        throw mjXError(elem, "exactly one of (geom2, body2) must be specified");
      }
      sensor->reftype = has_body2 ? mjOBJ_BODY : mjOBJ_GEOM;
      if (type == "distance") {
        sensor->type = mjSENS_GEOMDIST;
      } else if (type == "normal") {
        sensor->type = mjSENS_GEOMNORMAL;
      } else {
        sensor->type = mjSENS_GEOMFROMTO;
      }
    }

    // sensor for contacts; attached to geoms or bodies or a site
    else if (type == "contact") {
      // first matching criterion
      bool has_site = ReadAttrTxt(elem, "site", objname);
      bool has_body1 = ReadAttrTxt(elem, "body1", objname);
      bool has_subtree1 = ReadAttrTxt(elem, "subtree1", objname);
      bool has_geom1 = ReadAttrTxt(elem, "geom1", objname);
      if (has_site + has_body1 + has_subtree1 + has_geom1 > 1) {
        throw mjXError(elem, "at most one of (geom1, body1, subtree1, site) can be specified");
      }
      if (has_site)          { sensor->objtype = mjOBJ_SITE; }
      else if (has_body1)    { sensor->objtype = mjOBJ_BODY; }
      else if (has_subtree1) { sensor->objtype = mjOBJ_XBODY; }
      else if (has_geom1)    { sensor->objtype = mjOBJ_GEOM; }
      else                   { sensor->objtype = mjOBJ_UNKNOWN; }

      // second matching criterion
      bool has_body2 = ReadAttrTxt(elem, "body2", refname);
      bool has_subtree2 = ReadAttrTxt(elem, "subtree2", refname);
      bool has_geom2 = ReadAttrTxt(elem, "geom2", refname);
      if (has_body2 + has_subtree2 + has_geom2 > 1) {
        throw mjXError(elem, "at most one of (geom2, body2, subtree2) can be specified");
      }
      if (has_body2)         { sensor->reftype = mjOBJ_BODY; }
      else if (has_subtree2) { sensor->reftype = mjOBJ_XBODY; }
      else if (has_geom2)    { sensor->reftype = mjOBJ_GEOM; }
      else                   { sensor->reftype = mjOBJ_UNKNOWN; }

      // process data specification (intprm[0])
      int dataspec = 1 << mjCONDATA_FOUND;
      std::vector<int> condata(mjNCONDATA);
      int nkeys = MapValues(elem, "data", condata.data(), condata_map, mjNCONDATA);
      if (nkeys) {
        dataspec = 1 << condata[0];

        // check ordering while adding bits to dataspec
        for (int i = 1; i < nkeys; ++i) {
          if (condata[i] <= condata[i-1]) {
            std::string correct_order;
            for (int j = 0; j < mjNCONDATA; ++j) {
              correct_order += condata_map[j].key;
              if (j < mjNCONDATA - 1) correct_order += ", ";
            }
            throw mjXError(elem, "data attributes must be in order: %s", correct_order.c_str());
          }
          dataspec |= 1 << condata[i];
        }
      }
      sensor->intprm[0] = dataspec;

      // reduction type (intprm[1])
      sensor->intprm[1] = 0;
      if (MapValue(elem, "reduce", &n, reduce_map, reduce_sz)) {
        sensor->intprm[1] = n;
      }

      // number of contacts (intprm[2])
      sensor->intprm[2] = 1;
      ReadAttrInt(elem, "num", &sensor->intprm[2]);
      if (sensor->intprm[2] <= 0) {
        throw mjXError(elem, "'num' must be positive in sensor");
      }

      // sensor type
      sensor->type = mjSENS_CONTACT;
    }

    // global sensors
    else if (type == "e_potential") {
      sensor->type = mjSENS_E_POTENTIAL;
      sensor->objtype = mjOBJ_UNKNOWN;
    } else if (type == "e_kinetic") {
      sensor->type = mjSENS_E_KINETIC;
      sensor->objtype = mjOBJ_UNKNOWN;
    } else if (type == "clock") {
      sensor->type = mjSENS_CLOCK;
      sensor->objtype = mjOBJ_UNKNOWN;
    }

    // user-defined sensor
    else if (type == "user") {
      sensor->type = mjSENS_USER;
      bool objname_given = ReadAttrTxt(elem, "objname", objname);
      if (ReadAttrTxt(elem, "objtype", text)) {
        if (!objname_given) {
          throw mjXError(elem, "objtype '%s' given but objname is missing", text.c_str());
        }
        sensor->objtype = (mjtObj)mju_str2Type(text.c_str());
      } else if (objname_given) {
        throw mjXError(elem, "objname '%s' given but objtype is missing", objname.c_str());
      }
      ReadAttrInt(elem, "dim", &sensor->dim, true);

      // keywords
      if (MapValue(elem, "needstage", &n, stage_map, stage_sz)) {
        sensor->needstage = (mjtStage)n;
      }
      if (MapValue(elem, "datatype", &n, datatype_map, datatype_sz)) {
        sensor->datatype = (mjtDataType)n;
      }
    }

    // tactile sensor
    if (type == "tactile") {
      sensor->type = mjSENS_TACTILE;
      sensor->reftype = mjOBJ_GEOM;
      ReadAttrTxt(elem, "geom", refname, /*required=*/true);

      // associate the sensor with a mesh
      sensor->objtype = mjOBJ_MESH;
      ReadAttrTxt(elem, "mesh", objname, /*required=*/true);
      mjs_setString(sensor->objname, objname.c_str());
    }

    else if (type == "plugin") {
      sensor->type = mjSENS_PLUGIN;
      OnePlugin(elem, &sensor->plugin);
      ReadAttrTxt(elem, "objtype", text);
      sensor->objtype = (mjtObj)mju_str2Type(text.c_str());
      ReadAttrTxt(elem, "objname", objname);
      if (sensor->objtype != mjOBJ_UNKNOWN && objname.empty()) {
        throw mjXError(elem, "objtype is specified but objname is not");
      }
      if (sensor->objtype == mjOBJ_UNKNOWN && !objname.empty()) {
        throw mjXError(elem, "objname is specified but objtype is not");
      }
      if (ReadAttrTxt(elem, "reftype", text)) {
        sensor->reftype = (mjtObj)mju_str2Type(text.c_str());
      }
      ReadAttrTxt(elem, "refname", refname);
      if (sensor->reftype != mjOBJ_UNKNOWN && refname.empty()) {
        throw mjXError(elem, "reftype is specified but refname is not");
      }
      if (sensor->reftype == mjOBJ_UNKNOWN && !refname.empty()) {
        throw mjXError(elem, "refname is specified but reftype is not");
      }
    }

    if (!objname.empty()) {
      mjs_setString(sensor->objname, objname.c_str());
    }

    if (!refname.empty()) {
      mjs_setString(sensor->refname, refname.c_str());
    }

    // write info
    mjs_setString(sensor->info, ("line " + std::to_string(elem->GetLineNum())).c_str());

    // advance to next element
    elem = NextSiblingElement(elem);
  }
}



// keyframe section parser
void mjXReader::Keyframe(XMLElement* section) {
  XMLElement* elem;

  // iterate over child elements
  elem = FirstChildElement(section);
  while (elem) {
    string text, name = "";

    // add keyframe
    mjsKey* key = mjs_addKey(spec);

    // read name, time
    ReadAttrTxt(elem, "name", name);
    if (mjs_setName(key->element, name.c_str())) {
      throw mjXError(elem, "%s", mjs_getError(spec));
    }
    ReadAttr(elem, "time", 1, &key->time, text);

    // read qpos
    auto maybe_data = ReadAttrVec<double>(elem, "qpos", false);
    if (maybe_data.has_value()) {
      mjs_setDouble(key->qpos, maybe_data->data(), maybe_data->size());
    }

    // read qvel
    maybe_data = ReadAttrVec<double>(elem, "qvel", false);
    if (maybe_data.has_value()) {
      mjs_setDouble(key->qvel, maybe_data->data(), maybe_data->size());
    }

    // read act
    maybe_data = ReadAttrVec<double>(elem, "act", false);
    if (maybe_data.has_value()) {
      mjs_setDouble(key->act, maybe_data->data(), maybe_data->size());
    }

    // read mpos
    maybe_data = ReadAttrVec<double>(elem, "mpos", false);
    if (maybe_data.has_value()) {
      mjs_setDouble(key->mpos, maybe_data->data(), maybe_data->size());
    }

    // read mquat
    maybe_data = ReadAttrVec<double>(elem, "mquat", false);
    if (maybe_data.has_value()) {
      mjs_setDouble(key->mquat, maybe_data->data(), maybe_data->size());
    }

    // read ctrl
    maybe_data = ReadAttrVec<double>(elem, "ctrl", false);
    if (maybe_data.has_value()) {
      mjs_setDouble(key->ctrl, maybe_data->data(), maybe_data->size());
    }

    // advance to next element
    elem = NextSiblingElement(elem);
  }
}



// get defaults class
const mjsDefault* mjXReader::GetClass(XMLElement* section) {
  string text;

  if (!ReadAttrTxt(section, "class", text)) {
    return nullptr;
  }

  const mjsDefault* def = mjs_findDefault(spec, text.c_str());
  if (!def) {
    throw mjXError(
            section,
            string("unknown default class name '" + text + "'").c_str());
  }
  return def;
}

void mjXReader::SetModelFileDir(const string& modelfiledir) {
  modelfiledir_ = FilePath(modelfiledir);
}

void mjXReader::SetAssetDir(const string& assetdir) {
  assetdir_ = FilePath(assetdir);
}

void mjXReader::SetMeshDir(const string& meshdir) {
  meshdir_ = FilePath(meshdir);
}

void mjXReader::SetTextureDir(const string& texturedir) {
  texturedir_ = FilePath(texturedir);
}

FilePath mjXReader::AssetDir() const {
  return modelfiledir_ + assetdir_;
}

FilePath mjXReader::MeshDir() const {
  if (meshdir_.empty()) {
    return AssetDir();
  }
  return modelfiledir_ + meshdir_;
}
FilePath mjXReader::TextureDir() const {
  if (texturedir_.empty()) {
    return AssetDir();
  }
  return modelfiledir_ + texturedir_;
}
