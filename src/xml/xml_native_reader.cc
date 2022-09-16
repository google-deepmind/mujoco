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

#include <cfloat>
#include <cstdio>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>

#include <mujoco/mjmodel.h>
#include <mujoco/mjvisualize.h>
#include "engine/engine_macro.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "user/user_composite.h"
#include "user/user_model.h"
#include "user/user_util.h"
#include "tinyxml2.h"

namespace {

using std::string;
using std::vector;
using tinyxml2::XMLElement;

}  // namespace


//---------------------------------- MJCF schema ---------------------------------------------------

static const int nMJCF = 165;
static const char* MJCF[nMJCF][mjXATTRNUM] = {
{"mujoco", "!", "1", "model"},
{"<"},
    {"compiler", "*", "19", "autolimits", "boundmass", "boundinertia", "settotalmass",
        "balanceinertia", "strippath", "coordinate", "angle", "fitaabb", "eulerseq",
        "meshdir", "texturedir", "discardvisual", "convexhull", "usethread",
        "fusestatic", "inertiafromgeom", "inertiagrouprange", "exactmeshinertia"},
    {"<"},
        {"lengthrange", "?", "10", "mode", "useexisting", "uselimit",
            "accel", "maxforce", "timeconst", "timestep",
            "inttotal", "inteval", "tolrange"},
    {">"},

    {"option", "*", "22",
        "timestep", "apirate", "impratio", "tolerance", "noslip_tolerance", "mpr_tolerance",
        "gravity", "wind", "magnetic", "density", "viscosity",
        "o_margin", "o_solref", "o_solimp",
        "integrator", "collision", "cone", "jacobian",
        "solver", "iterations", "noslip_iterations", "mpr_iterations"},
    {"<"},
        {"flag", "?", "18", "constraint", "equality", "frictionloss", "limit", "contact",
            "passive", "gravity", "clampctrl", "warmstart",
            "filterparent", "actuation", "refsafe", "sensor",
            "override", "energy", "fwdinv", "sensornoise", "multiccd"},
    {">"},

    {"size", "*", "13", "njmax", "nconmax", "nstack", "nuserdata", "nkey",
        "nuser_body", "nuser_jnt", "nuser_geom", "nuser_site", "nuser_cam",
        "nuser_tendon", "nuser_actuator", "nuser_sensor"},

    {"visual", "*", "0"},
    {"<"},
        {"global", "?", "8", "fovy", "ipd", "azimuth", "elevation", "linewidth", "glow", "offwidth",
            "offheight"},
        {"quality", "?", "5", "shadowsize", "offsamples", "numslices", "numstacks",
            "numquads"},
        {"headlight", "?", "4", "ambient", "diffuse", "specular", "active"},
        {"map", "?", "13", "stiffness", "stiffnessrot", "force", "torque", "alpha",
            "fogstart", "fogend", "znear", "zfar", "haze", "shadowclip", "shadowscale",
            "actuatortendon"},
        {"scale", "?", "16", "forcewidth", "contactwidth", "contactheight", "connect", "com",
            "camera", "light", "selectpoint", "jointlength", "jointwidth", "actuatorlength",
            "actuatorwidth", "framelength", "framewidth", "constraint", "slidercrank"},
        {"rgba", "?", "22", "fog", "haze", "force", "inertia", "joint",
            "actuator", "actuatornegative", "actuatorpositive", "com",
            "camera", "light", "selectpoint", "connect", "contactpoint", "contactforce",
            "contactfriction", "contacttorque", "contactgap", "rangefinder",
            "constraint", "slidercrank", "crankbroken"},
    {">"},

    {"statistic", "*", "5", "meaninertia", "meanmass", "meansize", "extent", "center"},

    {"default", "R", "1", "class"},
    {"<"},
        {"mesh", "?", "1", "scale"},
        {"material", "?", "8", "texture", "emission", "specular", "shininess",
            "reflectance", "rgba", "texrepeat", "texuniform"},
        {"joint", "?", "19", "type", "group", "pos", "axis", "springdamper",
            "limited", "solreflimit", "solimplimit",
            "solreffriction", "solimpfriction", "stiffness", "range", "margin",
            "ref", "springref", "armature", "damping", "frictionloss", "user"},
        {"geom", "?", "31", "type", "pos", "quat", "contype", "conaffinity", "condim",
            "group", "priority", "size", "material", "friction", "mass", "density",
            "shellinertia", "solmix", "solref", "solimp",
            "margin", "gap", "fromto", "axisangle", "xyaxes", "zaxis", "euler",
            "hfield", "mesh", "fitscale", "rgba", "fluidshape", "fluidcoef", "user"},
        {"site", "?", "13", "type", "group", "pos", "quat", "material",
            "size", "fromto", "axisangle", "xyaxes", "zaxis", "euler", "rgba", "user"},
        {"camera", "?", "10", "fovy", "ipd", "pos", "quat",
            "axisangle", "xyaxes", "zaxis", "euler", "mode", "user"},
        {"light", "?", "12", "pos", "dir", "directional", "castshadow", "active",
            "attenuation", "cutoff", "exponent", "ambient", "diffuse", "specular", "mode"},
        {"pair", "?", "6", "condim", "friction", "solref", "solimp", "gap", "margin"},
        {"equality", "?", "3", "active", "solref", "solimp"},
        {"tendon", "?", "16", "group", "limited", "range",
            "solreflimit", "solimplimit", "solreffriction", "solimpfriction",
            "frictionloss", "springlength", "width", "material",
            "margin", "stiffness", "damping", "rgba", "user"},
        {"general", "?", "16", "ctrllimited", "forcelimited", "actlimited", "ctrlrange",
            "forcerange", "actrange", "gear", "cranklength", "user", "group",
            "dyntype", "gaintype", "biastype", "dynprm", "gainprm", "biasprm"},
        {"motor", "?", "8", "ctrllimited", "forcelimited", "ctrlrange", "forcerange",
            "gear", "cranklength", "user", "group"},
        {"position", "?", "9", "ctrllimited", "forcelimited", "ctrlrange", "forcerange",
            "gear", "cranklength", "user", "group",
            "kp"},
        {"velocity", "?", "9", "ctrllimited", "forcelimited", "ctrlrange", "forcerange",
            "gear", "cranklength", "user", "group",
            "kv"},
        {"intvelocity", "?", "10", "ctrllimited", "forcelimited",
            "ctrlrange", "forcerange", "actrange",
            "gear", "cranklength", "user", "group",
            "kp"},
        {"damper", "?", "8", "forcelimited", "ctrlrange", "forcerange",
            "gear", "cranklength", "user", "group",
            "kv"},
        {"cylinder", "?", "12", "ctrllimited", "forcelimited", "ctrlrange", "forcerange",
            "gear", "cranklength", "user", "group",
            "timeconst", "area", "diameter", "bias"},
        {"muscle", "?", "17", "ctrllimited", "forcelimited", "ctrlrange", "forcerange",
            "gear", "cranklength", "user", "group",
            "timeconst", "range", "force", "scale",
            "lmin", "lmax", "vmax", "fpmax", "fvmax"},
        {"adhesion", "?", "6", "forcelimited", "ctrlrange", "forcerange",
            "gain", "user", "group"},
    {">"},

    {"custom", "*", "0"},
    {"<"},
        {"numeric", "*", "3", "name", "size", "data"},
        {"text", "*", "2", "name", "data"},
        {"tuple", "*", "1", "name"},
        {"<"},
            {"element", "*", "3", "objtype", "objname", "prm"},
        {">"},
    {">"},

    {"asset", "*", "0"},
    {"<"},
        {"texture", "*", "21", "name", "type", "file", "gridsize", "gridlayout",
            "fileright", "fileleft", "fileup", "filedown", "filefront", "fileback",
            "builtin", "rgb1", "rgb2", "mark", "markrgb", "random", "width", "height",
            "hflip", "vflip"},
        {"hfield", "*", "5", "name", "file", "nrow", "ncol", "size"},
        {"mesh", "*", "11", "name", "class", "file", "vertex", "normal",
            "texcoord", "face", "refpos", "refquat", "scale", "smoothnormal"},
        {"skin", "*", "9", "name", "file", "material", "rgba", "inflate",
            "vertex", "texcoord", "face", "group"},
        {"<"},
            {"bone", "*", "5", "body", "bindpos", "bindquat", "vertid", "vertweight"},
        {">"},
        {"material", "*", "10", "name", "class", "texture",  "texrepeat", "texuniform",
            "emission", "specular", "shininess", "reflectance", "rgba"},
    {">"},

    {"body", "R", "10", "name", "childclass", "pos", "quat", "mocap",
        "axisangle", "xyaxes", "zaxis", "euler", "user"},
    {"<"},
        {"inertial", "?", "9", "pos", "quat", "mass", "diaginertia",
            "axisangle", "xyaxes", "zaxis", "euler", "fullinertia"},
        {"joint", "*", "21", "name", "class", "type", "group", "pos", "axis",
            "springdamper", "limited",
            "solreflimit", "solimplimit", "solreffriction", "solimpfriction",
            "stiffness", "range", "margin", "ref", "springref", "armature", "damping",
            "frictionloss", "user"},
        {"freejoint", "*", "2", "name", "group"},
        {"geom", "*", "33", "name", "class", "type", "contype", "conaffinity", "condim",
            "group", "priority", "size", "material", "friction", "mass", "density",
            "shellinertia", "solmix", "solref", "solimp",
            "margin", "gap", "fromto", "pos", "quat", "axisangle", "xyaxes", "zaxis", "euler",
            "hfield", "mesh", "fitscale", "rgba", "fluidshape", "fluidcoef", "user"},
        {"site", "*", "15", "name", "class", "type", "group", "pos", "quat",
            "material", "size", "fromto", "axisangle", "xyaxes", "zaxis", "euler", "rgba", "user"},
        {"camera", "*", "13", "name", "class", "fovy", "ipd",
            "pos", "quat", "axisangle", "xyaxes", "zaxis", "euler",
            "mode", "target", "user"},
        {"light", "*", "15", "name", "class", "directional", "castshadow", "active",
            "pos", "dir", "attenuation", "cutoff", "exponent", "ambient", "diffuse", "specular",
            "mode", "target"},
        {"composite", "*", "8", "prefix", "type", "count", "spacing", "offset",
            "flatinertia", "solrefsmooth", "solimpsmooth"},
        {"<"},
            {"joint", "*", "15", "kind", "group", "stiffness", "damping", "armature",
                "solreffix", "solimpfix",
                "limited", "range", "margin", "solreflimit", "solimplimit",
                "frictionloss", "solreffriction", "solimpfriction"},
            {"tendon", "*", "17", "kind", "group", "stiffness", "damping",
                "solreffix", "solimpfix",
                "limited", "range", "margin", "solreflimit", "solimplimit",
                "frictionloss", "solreffriction", "solimpfriction",
                "material", "rgba", "width"},
            {"skin", "?", "6", "texcoord", "material", "group", "rgba", "inflate", "subgrid"},
            {"geom", "?", "17", "type", "contype", "conaffinity", "condim",
                "group", "priority", "size", "material", "rgba", "friction", "mass",
                "density", "solmix", "solref", "solimp", "margin", "gap"},
            {"site", "?", "4", "group", "size", "material", "rgba"},
            {"pin", "*", "1", "coord"},
        {">"},
    {">"},

    {"contact", "*", "0"},
    {"<"},
        {"pair", "*", "10", "name", "class", "geom1", "geom2", "condim", "friction",
            "solref", "solimp", "gap", "margin"},
        {"exclude", "*", "3", "name", "body1", "body2"},
    {">"},

    {"equality", "*", "0"},
    {"<"},
        {"connect", "*", "8", "name", "class", "body1", "body2", "anchor",
            "active", "solref", "solimp"},
        {"weld", "*", "10", "name", "class", "body1", "body2", "relpose", "anchor",
            "active", "solref", "solimp", "torquescale"},
        {"joint", "*", "8", "name", "class", "joint1", "joint2", "polycoef",
            "active", "solref", "solimp"},
        {"tendon", "*", "8", "name", "class", "tendon1", "tendon2", "polycoef",
            "active", "solref", "solimp"},
        {"distance", "*", "8", "name", "class", "geom1", "geom2", "distance",
            "active", "solref", "solimp"},
    {">"},

    {"tendon", "*", "0"},
    {"<"},
        {"spatial", "*", "18", "name", "class", "group", "limited", "range",
            "solreflimit", "solimplimit", "solreffriction", "solimpfriction",
            "frictionloss", "springlength", "width", "material",
            "margin", "stiffness", "damping", "rgba", "user"},
        {"<"},
            {"site", "*", "1", "site"},
            {"geom", "*", "2", "geom", "sidesite"},
            {"pulley", "*", "1", "divisor"},
        {">"},
        {"fixed", "*", "15", "name", "class", "group", "limited", "range",
            "solreflimit", "solimplimit", "solreffriction", "solimpfriction",
            "frictionloss", "springlength", "margin", "stiffness", "damping", "user"},
        {"<"},
            {"joint", "*", "2", "joint", "coef"},
        {">"},
    {">"},

    {"actuator", "*", "0"},
    {"<"},
        {"general", "*", "27", "name", "class", "group",
            "ctrllimited", "forcelimited", "actlimited", "ctrlrange", "forcerange", "actrange",
            "lengthrange", "gear", "cranklength", "user",
            "joint", "jointinparent", "tendon", "slidersite", "cranksite", "site", "refsite",
            "body", "dyntype", "gaintype", "biastype", "dynprm", "gainprm", "biasprm"},
        {"motor", "*", "18", "name", "class", "group",
            "ctrllimited", "forcelimited", "ctrlrange", "forcerange",
            "lengthrange", "gear", "cranklength", "user",
            "joint", "jointinparent", "tendon", "slidersite", "cranksite", "site", "refsite"},
        {"position", "*", "19", "name", "class", "group",
            "ctrllimited", "forcelimited", "ctrlrange", "forcerange",
            "lengthrange", "gear", "cranklength", "user",
            "joint", "jointinparent", "tendon", "slidersite", "cranksite", "site", "refsite",
            "kp"},
        {"velocity", "*", "19", "name", "class", "group",
            "ctrllimited", "forcelimited", "ctrlrange", "forcerange",
            "lengthrange", "gear", "cranklength", "user",
            "joint", "jointinparent", "tendon", "slidersite", "cranksite", "site", "refsite",
            "kv"},
        {"intvelocity", "*", "20", "name", "class", "group",
            "ctrllimited", "forcelimited",
            "ctrlrange", "forcerange", "actrange", "lengthrange",
            "gear", "cranklength", "user",
            "joint", "jointinparent", "tendon", "slidersite", "cranksite", "site", "refsite",
            "kp"},
        {"damper", "*", "18", "name", "class", "group",
            "forcelimited", "ctrlrange", "forcerange",
            "lengthrange", "gear", "cranklength", "user",
            "joint", "jointinparent", "tendon", "slidersite", "cranksite", "site", "refsite",
            "kv"},
        {"cylinder", "*", "22", "name", "class", "group",
            "ctrllimited", "forcelimited", "ctrlrange", "forcerange",
            "lengthrange", "gear", "cranklength", "user",
            "joint", "jointinparent", "tendon", "slidersite", "cranksite", "site", "refsite",
            "timeconst", "area", "diameter", "bias"},
        {"muscle", "*", "25",  "name", "class", "group",
            "ctrllimited", "forcelimited", "ctrlrange", "forcerange",
            "lengthrange", "gear", "cranklength", "user",
            "joint", "jointinparent", "tendon", "slidersite", "cranksite",
            "timeconst", "range", "force", "scale",
            "lmin", "lmax", "vmax", "fpmax", "fvmax"},
        {"adhesion", "*", "9", "name", "class", "group",
            "forcelimited", "ctrlrange", "forcerange", "user", "body", "gain"},
    {">"},

    {"sensor", "*", "0"},
    {"<"},
        {"touch", "*", "5", "name", "site", "cutoff", "noise", "user"},
        {"accelerometer", "*", "5", "name", "site", "cutoff", "noise", "user"},
        {"velocimeter", "*", "5", "name", "site", "cutoff", "noise", "user"},
        {"gyro", "*", "5", "name", "site", "cutoff", "noise", "user"},
        {"force", "*", "5", "name", "site", "cutoff", "noise", "user"},
        {"torque", "*", "5", "name", "site", "cutoff", "noise", "user"},
        {"magnetometer", "*", "5", "name", "site", "cutoff", "noise", "user"},
        {"rangefinder", "*", "5", "name", "site", "cutoff", "noise", "user"},
        {"jointpos", "*", "5", "name", "joint", "cutoff", "noise", "user"},
        {"jointvel", "*", "5", "name", "joint", "cutoff", "noise", "user"},
        {"tendonpos", "*", "5", "name", "tendon", "cutoff", "noise", "user"},
        {"tendonvel", "*", "5", "name", "tendon", "cutoff", "noise", "user"},
        {"actuatorpos", "*", "5", "name", "actuator", "cutoff", "noise", "user"},
        {"actuatorvel", "*", "5", "name", "actuator", "cutoff", "noise", "user"},
        {"actuatorfrc", "*", "5", "name", "actuator", "cutoff", "noise", "user"},
        {"ballquat", "*", "5", "name", "joint", "cutoff", "noise", "user"},
        {"ballangvel", "*", "5", "name", "joint", "cutoff", "noise", "user"},
        {"jointlimitpos", "*", "5", "name", "joint", "cutoff", "noise", "user"},
        {"jointlimitvel", "*", "5", "name", "joint", "cutoff", "noise", "user"},
        {"jointlimitfrc", "*", "5", "name", "joint", "cutoff", "noise", "user"},
        {"tendonlimitpos", "*", "5", "name", "tendon", "cutoff", "noise", "user"},
        {"tendonlimitvel", "*", "5", "name", "tendon", "cutoff", "noise", "user"},
        {"tendonlimitfrc", "*", "5", "name", "tendon", "cutoff", "noise", "user"},
        {"framepos", "*", "8", "name", "objtype", "objname", "reftype", "refname", "cutoff", "noise", "user"},
        {"framequat", "*", "8", "name", "objtype", "objname", "reftype", "refname", "cutoff", "noise", "user"},
        {"framexaxis", "*", "8", "name", "objtype", "objname", "reftype", "refname", "cutoff", "noise", "user"},
        {"frameyaxis", "*", "8", "name", "objtype", "objname", "reftype", "refname", "cutoff", "noise", "user"},
        {"framezaxis", "*", "8", "name", "objtype", "objname", "reftype", "refname", "cutoff", "noise", "user"},
        {"framelinvel", "*", "8", "name", "objtype", "objname", "reftype", "refname", "cutoff", "noise", "user"},
        {"frameangvel", "*", "8", "name", "objtype", "objname", "reftype", "refname", "cutoff", "noise", "user"},
        {"framelinacc", "*", "6", "name", "objtype", "objname", "cutoff", "noise", "user"},
        {"frameangacc", "*", "6", "name", "objtype", "objname", "cutoff", "noise", "user"},
        {"subtreecom", "*", "5", "name", "body", "cutoff", "noise", "user"},
        {"subtreelinvel", "*", "5", "name", "body", "cutoff", "noise", "user"},
        {"subtreeangmom", "*", "5", "name", "body", "cutoff", "noise", "user"},
        {"clock", "*", "4", "name", "cutoff", "noise", "user"},
        {"user", "*", "9", "name", "objtype", "objname", "datatype", "needstage",
            "dim", "cutoff", "noise", "user"},
    {">"},

    {"keyframe", "*", "0"},
    {"<"},
        {"key", "*", "8", "name", "time", "qpos", "qvel", "act", "mpos", "mquat", "ctrl"},
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


// joint type
const int joint_sz = 4;
const mjMap joint_map[joint_sz] = {
  {"free",        mjJNT_FREE},
  {"ball",        mjJNT_BALL},
  {"slide",       mjJNT_SLIDE},
  {"hinge",       mjJNT_HINGE}
};


// geom type
const mjMap geom_map[mjNGEOMTYPES] = {
  {"plane",       mjGEOM_PLANE},
  {"hfield",      mjGEOM_HFIELD},
  {"sphere",      mjGEOM_SPHERE},
  {"capsule",     mjGEOM_CAPSULE},
  {"ellipsoid",   mjGEOM_ELLIPSOID},
  {"cylinder",    mjGEOM_CYLINDER},
  {"box",         mjGEOM_BOX},
  {"mesh",        mjGEOM_MESH}
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


// integrator type
const int integrator_sz = 3;
const mjMap integrator_map[integrator_sz] = {
  {"Euler",       mjINT_EULER},
  {"RK4",         mjINT_RK4},
  {"implicit",    mjINT_IMPLICIT}
};


// collision type
const int collision_sz = 3;
const mjMap collision_map[collision_sz] = {
  {"all",         mjCOL_ALL},
  {"predefined",  mjCOL_PAIR},
  {"dynamic",     mjCOL_DYNAMIC}
};


// cone type
const int cone_sz = 2;
const mjMap cone_map[cone_sz] = {
  {"pyramidal",   mjCONE_PYRAMIDAL},
  {"elliptic",    mjCONE_ELLIPTIC}
};


// Jacobian type
const int jac_sz = 3;
const mjMap jac_map[jac_sz] = {
  {"dense",       mjJAC_DENSE},
  {"sparse",      mjJAC_SPARSE},
  {"auto",        mjJAC_AUTO}
};


// solver type
const int solver_sz = 3;
const mjMap solver_map[solver_sz] = {
  {"PGS",         mjSOL_PGS},
  {"CG",          mjSOL_CG},
  {"Newton",      mjSOL_NEWTON}
};


// constraint type
const int equality_sz = 5;
const mjMap equality_map[equality_sz] = {
  {"connect",     mjEQ_CONNECT},
  {"weld",        mjEQ_WELD},
  {"joint",       mjEQ_JOINT},
  {"tendon",      mjEQ_TENDON},
  {"distance",    mjEQ_DISTANCE}
};


// type for texture
const int texture_sz = 3;
const mjMap texture_map[texture_sz] = {
  {"2d",          mjTEXTURE_2D},
  {"cube",        mjTEXTURE_CUBE},
  {"skybox",      mjTEXTURE_SKYBOX}
};


// builtin type for texture
const int builtin_sz = 4;
const mjMap builtin_map[builtin_sz] = {
  {"none",        mjBUILTIN_NONE},
  {"gradient",    mjBUILTIN_GRADIENT},
  {"checker",     mjBUILTIN_CHECKER},
  {"flat",        mjBUILTIN_FLAT}
};


// mark type for texture
const int mark_sz = 4;
const mjMap mark_map[mark_sz] = {
  {"none",        mjMARK_NONE},
  {"edge",        mjMARK_EDGE},
  {"cross",       mjMARK_CROSS},
  {"random",      mjMARK_RANDOM}
};


// dyn type
const int dyn_sz = 5;
const mjMap dyn_map[dyn_sz] = {
  {"none",        mjDYN_NONE},
  {"integrator",  mjDYN_INTEGRATOR},
  {"filter",      mjDYN_FILTER},
  {"muscle",      mjDYN_MUSCLE},
  {"user",        mjDYN_USER}
};


// gain type
const int gain_sz = 4;
const mjMap gain_map[gain_sz] = {
  {"fixed",       mjGAIN_FIXED},
  {"affine",      mjGAIN_AFFINE},
  {"muscle",      mjGAIN_MUSCLE},
  {"user",        mjGAIN_USER}
};


// bias type
const int bias_sz = 4;
const mjMap bias_map[bias_sz] = {
  {"none",        mjBIAS_NONE},
  {"affine",      mjBIAS_AFFINE},
  {"muscle",      mjBIAS_MUSCLE},
  {"user",        mjBIAS_USER}
};


// stage type
const int stage_sz = 4;
const mjMap stage_map[stage_sz] = {
  {"none",        mjSTAGE_NONE},
  {"pos",         mjSTAGE_POS},
  {"vel",         mjSTAGE_VEL},
  {"acc",         mjSTAGE_ACC}
};


// data type
const int datatype_sz = 4;
const mjMap datatype_map[datatype_sz] = {
  {"real",        mjDATATYPE_REAL},
  {"positive",    mjDATATYPE_POSITIVE},
  {"axis",        mjDATATYPE_AXIS},
  {"quaternion",  mjDATATYPE_QUATERNION}
};


// LR mode
const int lrmode_sz = 4;
const mjMap lrmode_map[datatype_sz] = {
  {"none",        mjLRMODE_NONE},
  {"muscle",      mjLRMODE_MUSCLE},
  {"muscleuser",  mjLRMODE_MUSCLEUSER},
  {"all",         mjLRMODE_ALL}
};


// composite type
const mjMap comp_map[mjNCOMPTYPES] = {
  {"particle",    mjCOMPTYPE_PARTICLE},
  {"grid",        mjCOMPTYPE_GRID},
  {"rope",        mjCOMPTYPE_ROPE},
  {"loop",        mjCOMPTYPE_LOOP},
  {"cloth",       mjCOMPTYPE_CLOTH},
  {"box",         mjCOMPTYPE_BOX},
  {"cylinder",    mjCOMPTYPE_CYLINDER},
  {"ellipsoid",   mjCOMPTYPE_ELLIPSOID}
};


// composite joint kind
const mjMap jkind_map[3] = {
  {"main",        mjCOMPKIND_JOINT},
  {"twist",       mjCOMPKIND_TWIST},
  {"stretch",     mjCOMPKIND_STRETCH}
};


// composite tendon kind
const mjMap tkind_map[2] = {
  {"main",        mjCOMPKIND_TENDON},
  {"shear",       mjCOMPKIND_SHEAR}
};


// mesh type
const mjMap  meshtype_map[2] = {
    {"false", mjVOLUME_MESH},
    {"true",  mjSHELL_MESH},
  };



//---------------------------------- class mjXReader implementation --------------------------------

// constructor
mjXReader::mjXReader() : schema(MJCF, nMJCF) {
  // check for schema construction error
  if (!schema.GetError().empty()) {
    throw mjXError(0, "Schema construction error: %s",
                   schema.GetError().c_str());
  }

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
void mjXReader::Parse(XMLElement* root) {
  XMLElement *section;

  // check schema
  if (!schema.GetError().empty()) {
    throw mjXError(0, "XML Schema Construction Error: %s\n",
                   schema.GetError().c_str());
  }

  // validate
  XMLElement* bad = 0;
  if ((bad = schema.Check(root, 0))) {
    throw mjXError(bad, "Schema violation: %s\n",
                   schema.GetError().c_str());
  }

  // get model name
  ReadAttrTxt(root, "model", model->modelname);

  // get comment
  if (root->FirstChild() && root->FirstChild()->ToComment()) {
    model->comment = root->FirstChild()->Value();
  } else {
    model->comment.clear();
  }

  //------------------- parse MuJoCo sections embedded in all XML formats

  for (section = root->FirstChildElement("compiler"); section;
       section = section->NextSiblingElement("compiler")) {
    Compiler(section, model);
  }

  for (section = root->FirstChildElement("option"); section;
       section = section->NextSiblingElement("option")) {
    Option(section, &model->option);
  }

  for (section = root->FirstChildElement("size"); section;
       section = section->NextSiblingElement("size")) {
    Size(section, model);
  }

  //------------------ parse MJCF-specific sections

  for (section = root->FirstChildElement("visual"); section;
       section = section->NextSiblingElement("visual")) {
    Visual(section);
  }

  for (section = root->FirstChildElement("statistic"); section;
       section = section->NextSiblingElement("statistic")) {
    Statistic(section);
  }

  readingdefaults = true;
  for (section = root->FirstChildElement("default"); section;
       section = section->NextSiblingElement("default")) {
    Default(section, -1);
  }
  readingdefaults = false;

  for (section = root->FirstChildElement("custom"); section;
       section = section->NextSiblingElement("custom")) {
    Custom(section);
  }

  for (section = root->FirstChildElement("asset"); section;
       section = section->NextSiblingElement("asset")) {
    Asset(section);
  }

  for (section = root->FirstChildElement("worldbody"); section;
       section = section->NextSiblingElement("worldbody")) {
    Body(section, model->GetWorld());
  }

  for (section = root->FirstChildElement("contact"); section;
       section = section->NextSiblingElement("contact")) {
    Contact(section);
  }

  for (section = root->FirstChildElement("equality"); section;
       section = section->NextSiblingElement("equality")) {
    Equality(section);
  }

  for (section = root->FirstChildElement("tendon"); section;
       section = section->NextSiblingElement("tendon")) {
    Tendon(section);
  }

  for (section = root->FirstChildElement("actuator"); section;
       section = section->NextSiblingElement("actuator")) {
    Actuator(section);
  }

  for (section = root->FirstChildElement("sensor"); section;
       section = section->NextSiblingElement("sensor")) {
    Sensor(section);
  }

  for (section = root->FirstChildElement("keyframe"); section;
       section = section->NextSiblingElement("keyframe")) {
    Keyframe(section);
  }
}



// compiler section parser
void mjXReader::Compiler(XMLElement* section, mjCModel* mod) {
  string text;
  int n;

  // top-level attributes
  if (MapValue(section, "autolimits", &n, bool_map, 2)) {
    mod->autolimits = (n==1);
  }
  ReadAttr(section, "boundmass", 1, &mod->boundmass, text);
  ReadAttr(section, "boundinertia", 1, &mod->boundinertia, text);
  ReadAttr(section, "settotalmass", 1, &mod->settotalmass, text);
  if (MapValue(section, "balanceinertia", &n, bool_map, 2)) {
    mod->balanceinertia = (n==1);
  }
  if (MapValue(section, "strippath", &n, bool_map, 2)) {
    mod->strippath = (n==1);
  }
  if (MapValue(section, "fitaabb", &n, bool_map, 2)) {
    mod->fitaabb = (n==1);
  }
  if (MapValue(section, "coordinate", &n, coordinate_map, 2)) {
    mod->global = (n==1);
  }
  if (MapValue(section, "angle", &n, angle_map, 2)) {
    mod->degree = (n==1);
  }
  if (ReadAttrTxt(section, "eulerseq", text)) {
    if (text.size()!=3) {
      throw mjXError(section, "euler format must have length 3");
    }
    memcpy(mod->euler, text.c_str(), 3);
  }
  ReadAttrTxt(section, "meshdir", mod->meshdir);
  ReadAttrTxt(section, "texturedir", mod->texturedir);
  if (MapValue(section, "discardvisual", &n, bool_map, 2)) {
    mod->discardvisual = (n==1);
  }
  if (MapValue(section, "convexhull", &n, bool_map, 2)) {
    mod->convexhull = (n==1);
  }
  if (MapValue(section, "usethread", &n, bool_map, 2)) {
    mod->usethread = (n==1);
  }
  if (MapValue(section, "fusestatic", &n, bool_map, 2)) {
    mod->fusestatic = (n==1);
  }
  MapValue(section, "inertiafromgeom", &mod->inertiafromgeom, TFAuto_map, 3);
  ReadAttr(section, "inertiagrouprange", 2, mod->inertiagrouprange, text);
  if (MapValue(section, "exactmeshinertia", &n, bool_map, 2)){
    mod->exactmeshinertia = (n==1);
  }

  // lengthrange subelement
  XMLElement* elem = FindSubElem(section, "lengthrange");
  if (elem) {
    mjLROpt* opt = &(mod->LRopt);

    // flags
    MapValue(elem, "mode", &opt->mode, lrmode_map, lrmode_sz);
    if (MapValue(elem, "useexisting", &n, bool_map, 2)) {
      opt->useexisting = (n==1);
    }
    if (MapValue(elem, "uselimit", &n, bool_map, 2)) {
      opt->uselimit = (n==1);
    }

    // algorithm parameters
    ReadAttr(elem, "accel", 1, &opt->accel, text);
    ReadAttr(elem, "maxforce", 1, &opt->maxforce, text);
    ReadAttr(elem, "timeconst", 1, &opt->timeconst, text);
    ReadAttr(elem, "timestep", 1, &opt->timestep, text);
    ReadAttr(elem, "inttotal", 1, &opt->inttotal, text);
    ReadAttr(elem, "inteval", 1, &opt->inteval, text);
    ReadAttr(elem, "tolrange", 1, &opt->tolrange, text);
  }
}



// option section parser
void mjXReader::Option(XMLElement* section, mjOption* opt) {
  string text;
  int n;

  // read options
  ReadAttr(section, "timestep", 1, &opt->timestep, text);
  ReadAttr(section, "apirate", 1, &opt->apirate, text);
  ReadAttr(section, "impratio", 1, &opt->impratio, text);
  ReadAttr(section, "tolerance", 1, &opt->tolerance, text);
  ReadAttr(section, "noslip_tolerance", 1, &opt->noslip_tolerance, text);
  ReadAttr(section, "mpr_tolerance", 1, &opt->mpr_tolerance, text);
  ReadAttr(section, "gravity", 3, opt->gravity, text);
  ReadAttr(section, "wind", 3, opt->wind, text);
  ReadAttr(section, "magnetic", 3, opt->magnetic, text);
  ReadAttr(section, "density", 1, &opt->density, text);
  ReadAttr(section, "viscosity", 1, &opt->viscosity, text);

  ReadAttr(section, "o_margin", 1, &opt->o_margin, text);
  ReadAttr(section, "o_solref", mjNREF, opt->o_solref, text, false, false);
  ReadAttr(section, "o_solimp", mjNIMP, opt->o_solimp, text, false, false);

  MapValue(section, "integrator", &opt->integrator, integrator_map, integrator_sz);
  MapValue(section, "collision", &opt->collision, collision_map, collision_sz);
  MapValue(section, "cone", &opt->cone, cone_map, cone_sz);
  MapValue(section, "jacobian", &opt->jacobian, jac_map, jac_sz);
  MapValue(section, "solver", &opt->solver, solver_map, solver_sz);
  ReadAttrInt(section, "iterations", &opt->iterations);
  ReadAttrInt(section, "noslip_iterations", &opt->noslip_iterations);
  ReadAttrInt(section, "mpr_iterations", &opt->mpr_iterations);

  // read disable sub-element
  XMLElement* elem = FindSubElem(section, "flag");
  if (elem) {
#define READDSBL(NAME, MASK) \
        if( MapValue(elem, NAME, &n, enable_map, 2) ) { \
            opt->disableflags ^= (opt->disableflags & MASK); \
            opt->disableflags |= (n ? 0 : MASK); }

    READDSBL("constraint",   mjDSBL_CONSTRAINT)
    READDSBL("equality",     mjDSBL_EQUALITY)
    READDSBL("frictionloss", mjDSBL_FRICTIONLOSS)
    READDSBL("limit",        mjDSBL_LIMIT)
    READDSBL("contact",      mjDSBL_CONTACT)
    READDSBL("passive",      mjDSBL_PASSIVE)
    READDSBL("gravity",      mjDSBL_GRAVITY)
    READDSBL("clampctrl",    mjDSBL_CLAMPCTRL)
    READDSBL("warmstart",    mjDSBL_WARMSTART)
    READDSBL("filterparent", mjDSBL_FILTERPARENT)
    READDSBL("actuation",    mjDSBL_ACTUATION)
    READDSBL("refsafe",      mjDSBL_REFSAFE)
    READDSBL("sensor",       mjDSBL_SENSOR)
#undef READDSBL

#define READENBL(NAME, MASK) \
        if( MapValue(elem, NAME, &n, enable_map, 2) ) { \
            opt->enableflags ^= (opt->enableflags & MASK); \
            opt->enableflags |= (n ? MASK : 0); }

    READENBL("override",    mjENBL_OVERRIDE)
    READENBL("energy",      mjENBL_ENERGY)
    READENBL("fwdinv",      mjENBL_FWDINV)
    READENBL("sensornoise", mjENBL_SENSORNOISE)
    READENBL("multiccd",    mjENBL_MULTICCD)
#undef READENBL
  }
}



// size section parser
void mjXReader::Size(XMLElement* section, mjCModel* mod) {
  // read sizes
  ReadAttrInt(section, "njmax", &mod->njmax);
  ReadAttrInt(section, "nconmax", &mod->nconmax);
  ReadAttrInt(section, "nstack", &mod->nstack);
  ReadAttrInt(section, "nuserdata", &mod->nuserdata);
  ReadAttrInt(section, "nkey", &mod->nkey);

  ReadAttrInt(section, "nuser_body", &mod->nuser_body);
  if (mod->nuser_body < -1) throw mjXError(section, "nuser_body must be >= -1");

  ReadAttrInt(section, "nuser_jnt", &mod->nuser_jnt);
  if (mod->nuser_jnt < -1) throw mjXError(section, "nuser_jnt must be >= -1");

  ReadAttrInt(section, "nuser_geom", &mod->nuser_geom);
  if (mod->nuser_geom < -1) throw mjXError(section, "nuser_geom must be >= -1");

  ReadAttrInt(section, "nuser_site", &mod->nuser_site);
  if (mod->nuser_site < -1) throw mjXError(section, "nuser_site must be >= -1");

  ReadAttrInt(section, "nuser_cam", &mod->nuser_cam);
  if (mod->nuser_cam < -1) throw mjXError(section, "nuser_cam must be >= -1");

  ReadAttrInt(section, "nuser_tendon", &mod->nuser_tendon);
  if (mod->nuser_tendon < -1) throw mjXError(section, "nuser_tendon must be >= -1");

  ReadAttrInt(section, "nuser_actuator", &mod->nuser_actuator);
  if (mod->nuser_actuator < -1) throw mjXError(section, "nuser_actuator must be >= -1");

  ReadAttrInt(section, "nuser_sensor", &mod->nuser_sensor);
  if (mod->nuser_sensor < -1) throw mjXError(section, "nuser_sensor must be >= -1");

}



// statistic section parser
void mjXReader::Statistic(XMLElement* section) {
  string text;

  // read statistics
  ReadAttr(section, "meaninertia", 1, &model->meaninertia, text);
  ReadAttr(section, "meanmass", 1, &model->meanmass, text);
  ReadAttr(section, "meansize", 1, &model->meansize, text);
  ReadAttr(section, "extent", 1, &model->extent, text);
  ReadAttr(section, "center", 3, model->center, text);
}



//---------------------------------- one-element parsers -------------------------------------------

// mesh element parser
void mjXReader::OneMesh(XMLElement* elem, mjCMesh* pmesh) {
  int n;
  string text;

  // read attributes
  ReadAttrTxt(elem, "name", pmesh->name);
  ReadAttrTxt(elem, "class", pmesh->classname);
  ReadAttrTxt(elem, "file", pmesh->file);
  ReadAttr(elem, "refpos", 3, pmesh->refpos, text);
  ReadAttr(elem, "refquat", 4, pmesh->refquat, text);
  ReadAttr(elem, "scale", 3, pmesh->scale, text);
  if (MapValue(elem, "smoothnormal", &n, bool_map, 2)) {
    pmesh->smoothnormal = (n==1);
  }

  // read user vertex data
  if (ReadAttrTxt(elem, "vertex", text)) String2Vector(text, pmesh->uservert);

  // read user normal data
  if (ReadAttrTxt(elem, "normal", text)) String2Vector(text, pmesh->usernormal);

  // read user texcoord data
  if (ReadAttrTxt(elem, "texcoord", text)) String2Vector(text, pmesh->usertexcoord);

  // read user face data
  if (ReadAttrTxt(elem, "face", text)) String2Vector(text, pmesh->userface);

  GetXMLPos(elem, pmesh);
}



// skin element parser
void mjXReader::OneSkin(XMLElement* elem, mjCSkin* pskin) {
  string text;
  float data[4];

  // read attributes
  ReadAttrTxt(elem, "name", pskin->name);
  ReadAttrTxt(elem, "file", pskin->file);
  ReadAttrTxt(elem, "material", pskin->material);
  ReadAttrInt(elem, "group", &pskin->group);
  if (pskin->group<0 || pskin->group>=mjNGROUP) {
    throw mjXError(elem, "skin group must be between 0 and 5");
  }
  ReadAttr(elem, "rgba", 4, pskin->rgba, text);
  ReadAttr(elem, "inflate", 1, &pskin->inflate, text);

  // read vertex data
  if (ReadAttrTxt(elem, "vertex", text)) String2Vector(text, pskin->vert);

  // read texcoord data
  if (ReadAttrTxt(elem, "texcoord", text)) String2Vector(text, pskin->texcoord);

  // read user face data
  if (ReadAttrTxt(elem, "face", text)) String2Vector(text, pskin->face);

  // read bones
  XMLElement* bone = elem->FirstChildElement("bone");
  while (bone) {
    // read body
    ReadAttrTxt(bone, "body", text, true);
    pskin->bodyname.push_back(text);

    // read bindpos
    ReadAttr(bone, "bindpos", 3, data, text, true);
    pskin->bindpos.push_back(data[0]);
    pskin->bindpos.push_back(data[1]);
    pskin->bindpos.push_back(data[2]);

    // read bindquat
    ReadAttr(bone, "bindquat", 4, data, text, true);
    pskin->bindquat.push_back(data[0]);
    pskin->bindquat.push_back(data[1]);
    pskin->bindquat.push_back(data[2]);
    pskin->bindquat.push_back(data[3]);

    // read vertid
    vector<int> tempid;
    ReadAttrTxt(bone, "vertid", text, true);
    String2Vector(text, tempid);
    pskin->vertid.push_back(tempid);

    // read vertweight
    vector<float> tempweight;
    ReadAttrTxt(bone, "vertweight", text, true);
    String2Vector(text, tempweight);
    pskin->vertweight.push_back(tempweight);

    // advance to next bone
    bone = bone->NextSiblingElement("bone");
  }

  GetXMLPos(elem, pskin);
}



// material element parser
void mjXReader::OneMaterial(XMLElement* elem, mjCMaterial* pmat) {
  string text;
  int n;

  // read attributes
  ReadAttrTxt(elem, "name", pmat->name);
  ReadAttrTxt(elem, "class", pmat->classname);
  ReadAttrTxt(elem, "texture", pmat->texture);
  if (MapValue(elem, "texuniform", &n, bool_map, 2)) {
    pmat->texuniform = (n==1);
  }
  ReadAttr(elem, "texrepeat", 2, pmat->texrepeat, text);
  ReadAttr(elem, "emission", 1, &pmat->emission, text);
  ReadAttr(elem, "specular", 1, &pmat->specular, text);
  ReadAttr(elem, "shininess", 1, &pmat->shininess, text);
  ReadAttr(elem, "reflectance", 1, &pmat->reflectance, text);
  ReadAttr(elem, "rgba", 4, pmat->rgba, text);

  GetXMLPos(elem, pmat);
}



// joint element parser
void mjXReader::OneJoint(XMLElement* elem, mjCJoint* pjoint) {
  string text;
  int n;

  // read attributes
  ReadAttrTxt(elem, "name", pjoint->name);
  ReadAttrTxt(elem, "class", pjoint->classname);
  if (MapValue(elem, "type", &n, joint_map, joint_sz)) {
    pjoint->type = (mjtJoint)n;
  }
  MapValue(elem, "limited", &pjoint->limited, TFAuto_map, 3);
  ReadAttrInt(elem, "group", &pjoint->group);
  ReadAttr(elem, "solreflimit", mjNREF, pjoint->solref_limit, text, false, false);
  ReadAttr(elem, "solimplimit", mjNIMP, pjoint->solimp_limit, text, false, false);
  ReadAttr(elem, "solreffriction", mjNREF, pjoint->solref_friction, text, false, false);
  ReadAttr(elem, "solimpfriction", mjNIMP, pjoint->solimp_friction, text, false, false);
  ReadAttr(elem, "pos", 3, pjoint->pos, text);
  ReadAttr(elem, "axis", 3, pjoint->axis, text);
  ReadAttr(elem, "springdamper", 2, pjoint->springdamper, text);
  ReadAttr(elem, "stiffness", 1, &pjoint->stiffness, text);
  ReadAttr(elem, "range", 2, pjoint->range, text);
  ReadAttr(elem, "margin", 1, &pjoint->margin, text);
  ReadAttr(elem, "ref", 1, &pjoint->ref, text);
  ReadAttr(elem, "springref", 1, &pjoint->springref, text);
  ReadAttr(elem, "armature", 1, &pjoint->armature, text);
  ReadAttr(elem, "damping", 1, &pjoint->damping, text);
  ReadAttr(elem, "frictionloss", 1, &pjoint->frictionloss, text);

  // read userdata
  ReadVector(elem, "user", pjoint->userdata, text);

  GetXMLPos(elem, pjoint);
}



// geom element parser
void mjXReader::OneGeom(XMLElement* elem, mjCGeom* pgeom) {
  string text;
  int n;

  // read attributes
  ReadAttrTxt(elem, "name", pgeom->name);
  ReadAttrTxt(elem, "class", pgeom->classname);
  if (MapValue(elem, "type", &n, geom_map, mjNGEOMTYPES)) {
    pgeom->type = (mjtGeom)n;
  }
  ReadAttr(elem, "size", 3, pgeom->size, text, false, false);
  ReadAttrInt(elem, "contype", &pgeom->contype);
  ReadAttrInt(elem, "conaffinity", &pgeom->conaffinity);
  ReadAttrInt(elem, "condim", &pgeom->condim);
  ReadAttrInt(elem, "group", &pgeom->group);
  ReadAttrInt(elem, "priority", &pgeom->priority);
  ReadAttr(elem, "friction", 3, pgeom->friction, text, false, false);
  ReadAttr(elem, "solmix", 1, &pgeom->solmix, text);
  ReadAttr(elem, "solref", mjNREF, pgeom->solref, text, false, false);
  ReadAttr(elem, "solimp", mjNIMP, pgeom->solimp, text, false, false);
  ReadAttr(elem, "margin", 1, &pgeom->margin, text);
  ReadAttr(elem, "gap", 1, &pgeom->gap, text);
  ReadAttrTxt(elem, "hfield", pgeom->hfield);
  ReadAttrTxt(elem, "mesh", pgeom->mesh);
  ReadAttr(elem, "fitscale", 1, &pgeom->fitscale, text);
  ReadAttrTxt(elem, "material", pgeom->material);
  ReadAttr(elem, "rgba", 4, pgeom->rgba, text);
  if (MapValue(elem, "fluidshape", &n, fluid_map, 2)) {
    pgeom->fluid_switch = (n == 1);
  }
  ReadAttr(elem, "fluidcoef", 5, pgeom->fluid_coefs, text, false, false);

  // read userdata
  ReadVector(elem, "user", pgeom->userdata, text);

  // remaining attributes
  ReadAttr(elem, "mass", 1, &pgeom->_mass, text);
  ReadAttr(elem, "density", 1, &pgeom->density, text);
  ReadAttr(elem, "fromto", 6, pgeom->fromto, text);
  ReadAttr(elem, "pos", 3, pgeom->pos, text);
  ReadAttr(elem, "quat", 4, pgeom->quat, text);
  ReadAlternative(elem, pgeom->alt);

  // compute inertia using either solid or shell geometry
  if (MapValue(elem, "shellinertia", &n, meshtype_map, 2)) {
    pgeom->typeinertia = (mjtMeshType)n;
  }

  GetXMLPos(elem, pgeom);
}



// site element parser
void mjXReader::OneSite(XMLElement* elem, mjCSite* psite) {
  int n;
  string text;

  // read attributes
  ReadAttrTxt(elem, "name", psite->name);
  ReadAttrTxt(elem, "class", psite->classname);
  if (MapValue(elem, "type", &n, geom_map, mjNGEOMTYPES)) {
    psite->type = (mjtGeom)n;
  }
  ReadAttr(elem, "size", 3, psite->size, text, false, false);
  ReadAttrInt(elem, "group", &psite->group);
  ReadAttr(elem, "pos", 3, psite->pos, text);
  ReadAttr(elem, "quat", 4, psite->quat, text);
  ReadAttrTxt(elem, "material", psite->material);
  ReadAttr(elem, "rgba", 4, psite->rgba, text);
  ReadAttr(elem, "fromto", 6, psite->fromto, text);
  ReadAlternative(elem, psite->alt);

  // read userdata
  ReadVector(elem, "user", psite->userdata, text);

  GetXMLPos(elem, psite);
}



// camera element parser
void mjXReader::OneCamera(XMLElement* elem, mjCCamera* pcam) {
  int n;
  string text;

  // read attributes
  ReadAttrTxt(elem, "name", pcam->name);
  ReadAttrTxt(elem, "class", pcam->classname);
  ReadAttrTxt(elem, "target", pcam->targetbody);
  if (MapValue(elem, "mode", &n, camlight_map, camlight_sz)) {
    pcam->mode = (mjtCamLight)n;
  }
  ReadAttr(elem, "pos", 3, pcam->pos, text);
  ReadAttr(elem, "quat", 4, pcam->quat, text);
  ReadAlternative(elem, pcam->alt);
  ReadAttr(elem, "fovy", 1, &pcam->fovy, text);
  ReadAttr(elem, "ipd", 1, &pcam->ipd, text);

  // read userdata
  ReadVector(elem, "user", pcam->userdata, text);

  GetXMLPos(elem, pcam);
}



// light element parser
void mjXReader::OneLight(XMLElement* elem, mjCLight* plight) {
  int n;
  string text;

  // read attributes
  ReadAttrTxt(elem, "name", plight->name);
  ReadAttrTxt(elem, "class", plight->classname);
  ReadAttrTxt(elem, "target", plight->targetbody);
  if (MapValue(elem, "mode", &n, camlight_map, camlight_sz)) {
    plight->mode = (mjtCamLight)n;
  }
  if (MapValue(elem, "directional", &n, bool_map, 2)) {
    plight->directional = (n==1);
  }
  if (MapValue(elem, "castshadow", &n, bool_map, 2)) {
    plight->castshadow = (n==1);
  }
  if (MapValue(elem, "active", &n, bool_map, 2)) {
    plight->active = (n==1);
  }
  ReadAttr(elem, "pos", 3, plight->pos, text);
  ReadAttr(elem, "dir", 3, plight->dir, text);
  ReadAttr(elem, "attenuation", 3, plight->attenuation, text);
  ReadAttr(elem, "cutoff", 1, &plight->cutoff, text);
  ReadAttr(elem, "exponent", 1, &plight->exponent, text);
  ReadAttr(elem, "ambient", 3, plight->ambient, text);
  ReadAttr(elem, "diffuse", 3, plight->diffuse, text);
  ReadAttr(elem, "specular", 3, plight->specular, text);

  GetXMLPos(elem, plight);
}



// pair element parser
void mjXReader::OnePair(XMLElement* elem, mjCPair* ppair) {
  string text;

  // regular only
  if (!readingdefaults) {
    ReadAttrTxt(elem, "class", ppair->classname);
    ReadAttrTxt(elem, "geom1", ppair->geomname1, true);
    ReadAttrTxt(elem, "geom2", ppair->geomname2, true);
  }

  // read other parameters
  ReadAttrTxt(elem, "name", ppair->name);
  ReadAttrInt(elem, "condim", &ppair->condim);
  ReadAttr(elem, "solref", mjNREF, ppair->solref, text, false, false);
  ReadAttr(elem, "solimp", mjNIMP, ppair->solimp, text, false, false);
  ReadAttr(elem, "margin", 1, &ppair->margin, text);
  ReadAttr(elem, "gap", 1, &ppair->gap, text);
  ReadAttr(elem, "friction", 5, ppair->friction, text, false, false);

  GetXMLPos(elem, ppair);
}



// equality element parser
void mjXReader::OneEquality(XMLElement* elem, mjCEquality* pequality) {
  int n;
  string text;

  // read type (bad keywords already detected by schema)
  text = elem->Value();
  pequality->type = (mjtEq)FindKey(equality_map, equality_sz, text);

  // regular only
  if (!readingdefaults) {
    ReadAttrTxt(elem, "name", pequality->name);
    ReadAttrTxt(elem, "class", pequality->classname);

    switch (pequality->type) {
    case mjEQ_CONNECT:
      ReadAttrTxt(elem, "body1", pequality->name1, true);
      ReadAttrTxt(elem, "body2", pequality->name2);
      ReadAttr(elem, "anchor", 3, pequality->data, text, true);
      break;

    case mjEQ_WELD:
      ReadAttrTxt(elem, "body1", pequality->name1, true);
      ReadAttrTxt(elem, "body2", pequality->name2);
      ReadAttr(elem, "relpose", 7, pequality->data+3, text);
      ReadAttr(elem, "torquescale", 1, pequality->data+10, text);
      if (!ReadAttr(elem, "anchor", 3, pequality->data, text)) {
        mjuu_zerovec(pequality->data, 3);
      }
      break;

    case mjEQ_JOINT:
      ReadAttrTxt(elem, "joint1", pequality->name1, true);
      ReadAttrTxt(elem, "joint2", pequality->name2);
      ReadAttr(elem, "polycoef", 5, pequality->data, text);
      break;

    case mjEQ_TENDON:
      ReadAttrTxt(elem, "tendon1", pequality->name1, true);
      ReadAttrTxt(elem, "tendon2", pequality->name2);
      ReadAttr(elem, "polycoef", 5, pequality->data, text);
      break;

    case mjEQ_DISTANCE:
      throw mjXError(elem, "support for distance equality contraints was removed in MuJoCo 2.2.2");
      break;

    default:                    // SHOULD NOT OCCUR
      throw mjXError(elem, "unrecognized equality constraint type");
    }
  }

  // read attributes
  if (MapValue(elem, "active", &n, bool_map, 2)) {
    pequality->active = (n==1);
  }
  ReadAttr(elem, "solref", mjNREF, pequality->solref, text, false, false);
  ReadAttr(elem, "solimp", mjNIMP, pequality->solimp, text, false, false);

  GetXMLPos(elem, pequality);
}



// tendon element parser
void mjXReader::OneTendon(XMLElement* elem, mjCTendon* pten) {
  string text;

  // read attributes
  ReadAttrTxt(elem, "name", pten->name);
  ReadAttrTxt(elem, "class", pten->classname);
  ReadAttrInt(elem, "group", &pten->group);
  ReadAttrTxt(elem, "material", pten->material);
  MapValue(elem, "limited", &pten->limited, TFAuto_map, 3);
  ReadAttr(elem, "width", 1, &pten->width, text);
  ReadAttr(elem, "solreflimit", mjNREF, pten->solref_limit, text, false, false);
  ReadAttr(elem, "solimplimit", mjNIMP, pten->solimp_limit, text, false, false);
  ReadAttr(elem, "solreffriction", mjNREF, pten->solref_friction, text, false, false);
  ReadAttr(elem, "solimpfriction", mjNIMP, pten->solimp_friction, text, false, false);
  ReadAttr(elem, "range", 2, pten->range, text);
  ReadAttr(elem, "margin", 1, &pten->margin, text);
  ReadAttr(elem, "stiffness", 1, &pten->stiffness, text);
  ReadAttr(elem, "damping", 1, &pten->damping, text);
  ReadAttr(elem, "frictionloss", 1, &pten->frictionloss, text);
  ReadAttr(elem, "springlength", 1, &pten->springlength, text);
  ReadAttr(elem, "rgba", 4, pten->rgba, text);

  // read userdata
  ReadVector(elem, "user", pten->userdata, text);

  GetXMLPos(elem, pten);
}



// actuator element parser
void mjXReader::OneActuator(XMLElement* elem, mjCActuator* pact) {
  int n;
  string text, type;
  double diameter;

  // common attributes
  ReadAttrTxt(elem, "name", pact->name);
  ReadAttrTxt(elem, "class", pact->classname);
  ReadAttrInt(elem, "group", &pact->group);
  MapValue(elem, "ctrllimited", &pact->ctrllimited, TFAuto_map, 3);
  MapValue(elem, "forcelimited", &pact->forcelimited, TFAuto_map, 3);
  MapValue(elem, "actlimited", &pact->actlimited, TFAuto_map, 3);
  ReadAttr(elem, "ctrlrange", 2, pact->ctrlrange, text);
  ReadAttr(elem, "forcerange", 2, pact->forcerange, text);
  ReadAttr(elem, "actrange", 2, pact->actrange, text);
  ReadAttr(elem, "lengthrange", 2, pact->lengthrange, text);
  ReadAttr(elem, "gear", 6, pact->gear, text, false, false);

  // transmission target and type
  int cnt = 0;
  if (ReadAttrTxt(elem, "joint", pact->target)) {
    pact->trntype = mjTRN_JOINT;
    cnt++;
  }
  if (ReadAttrTxt(elem, "jointinparent", pact->target)) {
    pact->trntype = mjTRN_JOINTINPARENT;
    cnt++;
  }
  if (ReadAttrTxt(elem, "tendon", pact->target)) {
    pact->trntype = mjTRN_TENDON;
    cnt++;
  }
  if (ReadAttrTxt(elem, "cranksite", pact->target)) {
    pact->trntype = mjTRN_SLIDERCRANK;
    cnt++;
  }
  if (ReadAttrTxt(elem, "site", pact->target)) {
    pact->trntype = mjTRN_SITE;
    cnt++;
  }
  if (ReadAttrTxt(elem, "body", pact->target)) {
    pact->trntype = mjTRN_BODY;
    cnt++;
  }
  // check for repeated transmission
  if (cnt>1) {
    throw mjXError(elem, "actuator can have at most one of transmission target");
  }

  // slidercrank-specific parameters
  int r1 = ReadAttr(elem, "cranklength", 1, &pact->cranklength, text);
  int r2 = ReadAttrTxt(elem, "slidersite", pact->slidersite);
  if ((r1 || r2) && pact->trntype!=mjTRN_SLIDERCRANK && pact->trntype!=mjTRN_UNDEFINED) {
    throw mjXError(elem, "cranklength and slidersite can only be used in slidercrank transmission");
  }

  // site-specific parameters (refsite)
  int r3 = ReadAttrTxt(elem, "refsite", pact->refsite);
  if (r3 && pact->trntype!=mjTRN_SITE && pact->trntype!=mjTRN_UNDEFINED) {
    throw mjXError(elem, "refsite can only be used with site transmission");
  }

  // get predefined type
  type = elem->Value();

  // explicit attributes
  if (type=="general") {
    // explicit attributes
    if (MapValue(elem, "dyntype", &n, dyn_map, dyn_sz)) {
      pact->dyntype = (mjtDyn)n;
    }
    if (MapValue(elem, "gaintype", &n, gain_map, gain_sz)) {
      pact->gaintype = (mjtGain)n;
    }
    if (MapValue(elem, "biastype", &n, bias_map, bias_sz)) {
      pact->biastype = (mjtBias)n;
    }
    ReadAttr(elem, "dynprm", mjNDYN, pact->dynprm, text, false, false);
    ReadAttr(elem, "gainprm", mjNGAIN, pact->gainprm, text, false, false);
    ReadAttr(elem, "biasprm", mjNBIAS, pact->biasprm, text, false, false);
  }

  // direct drive motor
  else if (type=="motor") {
    // unit gain
    pact->gainprm[0] = 1;

    // implied parameters
    pact->dyntype = mjDYN_NONE;
    pact->gaintype = mjGAIN_FIXED;
    pact->biastype = mjBIAS_NONE;
  }

  // position servo
  else if (type=="position") {
    // clear bias
    mjuu_zerovec(pact->biasprm, mjNBIAS);

    // explicit attributes
    ReadAttr(elem, "kp", 1, pact->gainprm, text);
    pact->biasprm[1] = -pact->gainprm[0];

    // implied parameters
    pact->dyntype = mjDYN_NONE;
    pact->gaintype = mjGAIN_FIXED;
    pact->biastype = mjBIAS_AFFINE;
  }

  // velocity servo
  else if (type=="velocity") {
    // clear bias
    mjuu_zerovec(pact->biasprm, mjNBIAS);

    // explicit attributes
    ReadAttr(elem, "kv", 1, pact->gainprm, text);
    pact->biasprm[2] = -pact->gainprm[0];

    // implied parameters
    pact->dyntype = mjDYN_NONE;
    pact->gaintype = mjGAIN_FIXED;
    pact->biastype = mjBIAS_AFFINE;
  }

  // integrated velocity
  else if (type=="intvelocity") {
    // clear bias
    mjuu_zerovec(pact->biasprm, mjNBIAS);

    // explicit attributes
    ReadAttr(elem, "kp", 1, pact->gainprm, text);

    // implied parameters
    pact->dyntype = mjDYN_INTEGRATOR;
    pact->gaintype = mjGAIN_FIXED;
    pact->biastype = mjBIAS_AFFINE;
    pact->actlimited = 1;
    pact->biasprm[1] = -pact->gainprm[0];
  }

  // damper
  else if (type=="damper") {
    // clear gain
    mjuu_zerovec(pact->gainprm, mjNGAIN);

    // explicit attributes
    ReadAttr(elem, "kv", 1, pact->gainprm+2, text);
    if (pact->gainprm[2]<0)
      throw mjXError(elem, "damping coefficient cannot be negative");
    pact->gainprm[2] = -pact->gainprm[2];

    // require nonnegative range
    ReadAttr(elem, "ctrlrange", 2, pact->ctrlrange, text, true);
    if (pact->ctrlrange[0]<0 || pact->ctrlrange[1]<0) {
      throw mjXError(elem, "damper control range cannot be negative");
    }

    // implied parameters
    pact->ctrllimited = 1;
    pact->dyntype = mjDYN_NONE;
    pact->gaintype = mjGAIN_AFFINE;
    pact->biastype = mjBIAS_NONE;
  }

  // cylinder
  else if (type=="cylinder") {
    // explicit attributes
    ReadAttr(elem, "timeconst", 1, pact->dynprm, text);
    ReadAttr(elem, "bias", 3, pact->biasprm, text);
    ReadAttr(elem, "area", 1, pact->gainprm, text);
    if (ReadAttr(elem, "diameter", 1, &diameter, text)) {
      pact->gainprm[0] = mjPI / 4 * diameter*diameter;
    }

    // implied parameters
    pact->dyntype = mjDYN_FILTER;
    pact->gaintype = mjGAIN_FIXED;
    pact->biastype = mjBIAS_AFFINE;
  }

  // muscle
  else if (type=="muscle") {
    // set muscle defaults if same as global defaults
    if (pact->dynprm[0]==1) pact->dynprm[0] = 0.01;    // tau act
    if (pact->dynprm[1]==0) pact->dynprm[1] = 0.04;    // tau deact
    if (pact->gainprm[0]==1) pact->gainprm[0] = 0.75;  // range[0]
    if (pact->gainprm[1]==0) pact->gainprm[1] = 1.05;  // range[1]
    if (pact->gainprm[2]==0) pact->gainprm[2] = -1;    // force
    if (pact->gainprm[3]==0) pact->gainprm[3] = 200;   // scale
    if (pact->gainprm[4]==0) pact->gainprm[4] = 0.5;   // lmin
    if (pact->gainprm[5]==0) pact->gainprm[5] = 1.6;   // lmax
    if (pact->gainprm[6]==0) pact->gainprm[6] = 1.5;   // vmax
    if (pact->gainprm[7]==0) pact->gainprm[7] = 1.3;   // fpmax
    if (pact->gainprm[8]==0) pact->gainprm[8] = 1.2;   // fvmax

    // explicit attributes
    ReadAttr(elem, "timeconst", 2, pact->dynprm, text);
    ReadAttr(elem, "range", 2, pact->gainprm, text);
    ReadAttr(elem, "force", 1, pact->gainprm+2, text);
    ReadAttr(elem, "scale", 1, pact->gainprm+3, text);
    ReadAttr(elem, "lmin", 1, pact->gainprm+4, text);
    ReadAttr(elem, "lmax", 1, pact->gainprm+5, text);
    ReadAttr(elem, "vmax", 1, pact->gainprm+6, text);
    ReadAttr(elem, "fpmax", 1, pact->gainprm+7, text);
    ReadAttr(elem, "fvmax", 1, pact->gainprm+8, text);

    // biasprm = gainprm
    for (n=0; n<9; n++) {
      pact->biasprm[n] = pact->gainprm[n];
    }

    // implied parameters
    pact->dyntype = mjDYN_MUSCLE;
    pact->gaintype = mjGAIN_MUSCLE;
    pact->biastype = mjBIAS_MUSCLE;
  }

  // adhesion
  else if (type=="adhesion") {
    // clear bias, set default gain
    mjuu_zerovec(pact->biasprm, mjNBIAS);
    mjuu_zerovec(pact->gainprm, mjNGAIN);
    pact->gainprm[0] = 1;

    // explicit attributes
    ReadAttr(elem, "gain", 1, pact->gainprm, text);
    if (pact->gainprm[0]<0)
      throw mjXError(elem, "adhesion gain cannot be negative");

    // require nonnegative range
    ReadAttr(elem, "ctrlrange", 2, pact->ctrlrange, text, true);
    if (pact->ctrlrange[0]<0 || pact->ctrlrange[1]<0) {
      throw mjXError(elem, "adhesion control range cannot be negative");
    }

    // implied parameters
    pact->ctrllimited = 1;
    pact->dyntype = mjDYN_NONE;
    pact->gaintype = mjGAIN_FIXED;
    pact->biastype = mjBIAS_NONE;
  }

  else {          // SHOULD NOT OCCUR
    throw mjXError(elem, "unrecognized actuator type: %s", type.c_str());
  }

  // read userdata
  ReadVector(elem, "user", pact->userdata, text);

  GetXMLPos(elem, pact);
}



// make composite
void mjXReader::OneComposite(XMLElement* elem, mjCBody* pbody, mjCDef* def) {
  string text;
  int n;

  // create out-of-DOM element
  mjCComposite comp;

  // common properties
  ReadAttrTxt(elem, "prefix", comp.prefix);
  if (MapValue(elem, "type", &n, comp_map, mjNCOMPTYPES, true)) {
    comp.type = (mjtCompType)n;
  }
  ReadAttr(elem, "count", 3, comp.count, text, true, false);
  ReadAttr(elem, "spacing", 1, &comp.spacing, text, true);
  ReadAttr(elem, "offset", 3, comp.offset, text);
  ReadAttr(elem, "flatinertia", 1, &comp.flatinertia, text);

  // skin
  XMLElement* eskin = elem->FirstChildElement("skin");
  if (eskin) {
    comp.skin = true;
    if (MapValue(eskin, "texcoord", &n, bool_map, 2)) {
      comp.skintexcoord = (n==1);
    }
    ReadAttrTxt(eskin, "material", comp.skinmaterial);
    ReadAttr(eskin, "rgba", 4, comp.skinrgba, text);
    ReadAttr(eskin, "inflate", 1, &comp.skininflate, text);
    ReadAttrInt(eskin, "subgrid", &comp.skinsubgrid);
    ReadAttrInt(eskin, "group", &comp.skingroup, 0);
    if (comp.skingroup<0 || comp.skingroup>=mjNGROUP) {
      throw mjXError(eskin, "skin group must be between 0 and 5");
    }
  }

  // set type-specific defaults
  comp.SetDefault();

  // parse smooth solver parameters after type-specific defaults are set
  ReadAttr(elem, "solrefsmooth", mjNREF, comp.solrefsmooth, text, false, false);
  ReadAttr(elem, "solimpsmooth", mjNIMP, comp.solimpsmooth, text, false, false);

  // geom
  XMLElement* egeom = elem->FirstChildElement("geom");
  if (egeom) {
    if (MapValue(egeom, "type", &n, geom_map, mjNGEOMTYPES)) {
      comp.def[0].geom.type = (mjtGeom)n;
    }
    ReadAttr(egeom, "size", 3, comp.def[0].geom.size, text, false, false);
    ReadAttrInt(egeom, "contype", &comp.def[0].geom.contype);
    ReadAttrInt(egeom, "conaffinity", &comp.def[0].geom.conaffinity);
    ReadAttrInt(egeom, "condim", &comp.def[0].geom.condim);
    ReadAttrInt(egeom, "group", &comp.def[0].geom.group);
    ReadAttrInt(egeom, "priority", &comp.def[0].geom.priority);
    ReadAttr(egeom, "friction", 3, comp.def[0].geom.friction, text, false, false);
    ReadAttr(egeom, "solmix", 1, &comp.def[0].geom.solmix, text);
    ReadAttr(egeom, "solref", mjNREF, comp.def[0].geom.solref, text, false, false);
    ReadAttr(egeom, "solimp", mjNIMP, comp.def[0].geom.solimp, text, false, false);
    ReadAttr(egeom, "margin", 1, &comp.def[0].geom.margin, text);
    ReadAttr(egeom, "gap", 1, &comp.def[0].geom.gap, text);
    ReadAttrTxt(egeom, "material", comp.def[0].geom.material);
    ReadAttr(egeom, "rgba", 4, comp.def[0].geom.rgba, text);
    ReadAttr(egeom, "mass", 1, &comp.def[0].geom._mass, text);
    ReadAttr(egeom, "density", 1, &comp.def[0].geom.density, text);
  }

  // site
  XMLElement* esite = elem->FirstChildElement("site");
  if (esite) {
    ReadAttr(esite, "size", 3, comp.def[0].site.size, text, false, false);
    ReadAttrInt(esite, "group", &comp.def[0].site.group);
    ReadAttrTxt(esite, "material", comp.def[0].site.material);
    ReadAttr(esite, "rgba", 4, comp.def[0].site.rgba, text);
  }

  // joint
  XMLElement* ejnt = elem->FirstChildElement("joint");
  while (ejnt) {
    // kind
    int kind;
    MapValue(ejnt, "kind", &kind, jkind_map, 3, true);
    comp.add[kind] = true;

    // solreffix, solimpfix
    ReadAttr(ejnt, "solreffix", mjNREF, comp.def[kind].equality.solref, text, false, false);
    ReadAttr(ejnt, "solimpfix", mjNIMP, comp.def[kind].equality.solimp, text, false, false);

    // joint attributes
    MapValue(elem, "limited", &comp.def[kind].joint.limited, TFAuto_map, 3);
    ReadAttrInt(ejnt, "group", &comp.def[kind].joint.group);
    ReadAttr(ejnt, "solreflimit", mjNREF, comp.def[kind].joint.solref_limit, text, false, false);
    ReadAttr(ejnt, "solimplimit", mjNIMP, comp.def[kind].joint.solimp_limit, text, false, false);
    ReadAttr(ejnt,
             "solreffriction", mjNREF, comp.def[kind].joint.solref_friction, text, false, false);
    ReadAttr(ejnt,
             "solimpfriction", mjNIMP, comp.def[kind].joint.solimp_friction, text, false, false);
    ReadAttr(ejnt, "stiffness", 1, &comp.def[kind].joint.stiffness, text);
    ReadAttr(ejnt, "range", 2, comp.def[kind].joint.range, text);
    ReadAttr(ejnt, "margin", 1, &comp.def[kind].joint.margin, text);
    ReadAttr(ejnt, "armature", 1, &comp.def[kind].joint.armature, text);
    ReadAttr(ejnt, "damping", 1, &comp.def[kind].joint.damping, text);
    ReadAttr(ejnt, "frictionloss", 1, &comp.def[kind].joint.frictionloss, text);

    // advance
    ejnt = ejnt->NextSiblingElement("joint");
  }

  // tendon
  XMLElement* eten = elem->FirstChildElement("tendon");
  while (eten) {
    // kind
    int kind;
    MapValue(eten, "kind", &kind, tkind_map, 2, true);
    comp.add[kind] = true;

    // solreffix, solimpfix
    ReadAttr(eten, "solreffix", mjNREF, comp.def[kind].equality.solref, text, false, false);
    ReadAttr(eten, "solimpfix", mjNIMP, comp.def[kind].equality.solimp, text, false, false);

    // tendon attributes
    MapValue(elem, "limited", &comp.def[kind].tendon.limited, TFAuto_map, 3);
    ReadAttrInt(eten, "group", &comp.def[kind].tendon.group);
    ReadAttr(eten, "solreflimit", mjNREF, comp.def[kind].tendon.solref_limit, text, false, false);
    ReadAttr(eten, "solimplimit", mjNIMP, comp.def[kind].tendon.solimp_limit, text, false, false);
    ReadAttr(eten,
             "solreffriction", mjNREF, comp.def[kind].tendon.solref_friction, text, false, false);
    ReadAttr(eten,
             "solimpfriction", mjNIMP, comp.def[kind].tendon.solimp_friction, text, false, false);
    ReadAttr(eten, "range", 2, comp.def[kind].tendon.range, text);
    ReadAttr(eten, "margin", 1, &comp.def[kind].tendon.margin, text);
    ReadAttr(eten, "stiffness", 1, &comp.def[kind].tendon.stiffness, text);
    ReadAttr(eten, "damping", 1, &comp.def[kind].tendon.damping, text);
    ReadAttr(eten, "frictionloss", 1, &comp.def[kind].tendon.frictionloss, text);
    ReadAttrTxt(eten, "material", comp.def[kind].tendon.material);
    ReadAttr(eten, "rgba", 4, comp.def[kind].tendon.rgba, text);
    ReadAttr(eten, "width", 1, &comp.def[kind].tendon.width, text);

    // advance
    eten = eten->NextSiblingElement("tendon");
  }

  // pin
  XMLElement* epin = elem->FirstChildElement("pin");
  while (epin) {
    // read
    int coord[2] = {0, 0};
    ReadAttr(epin, "coord", 2, coord, text, true, false);

    // insert 2 coordinates (2nd may be unused)
    comp.pin.push_back(coord[0]);
    comp.pin.push_back(coord[1]);

    // advance
    epin = epin->NextSiblingElement("pin");
  }

  // make composite
  char error[200];
  bool res = comp.Make(pbody->model, pbody, error, 200);

  // throw error
  if (!res) {
    throw mjXError(elem, error);
  }
}



//------------------ MJCF-specific sections --------------------------------------------------------

// default section parser
void mjXReader::Default(XMLElement* section, int parentid) {
  XMLElement* elem;
  string text, name;
  mjCDef* def;
  int thisid;

  // create new default, except at top level (already added in mjCModel ctor)
  text.clear();
  ReadAttrTxt(section, "class", text);
  if (text.empty()) {
    if (parentid>=0) {
      throw mjXError(section, "empty class name");
    } else {
      text = "main";
    }
  }
  if (parentid>=0) {
    thisid = (int)model->defaults.size();
    def = model->AddDef(text, parentid);
    if (!def) {
      throw mjXError(section, "repeated default class name");
    }
  } else {
    thisid = 0;
    def = model->defaults[0];
    def->name = text;
  }

  // iterate over elements other than nested defaults
  elem = section->FirstChildElement();
  while (elem) {
    // get element name
    name = elem->Value();

    // read mesh
    if (name=="mesh") OneMesh(elem, &def->mesh);

    // read material
    else if (name=="material") OneMaterial(elem, &def->material);

    // read joint
    else if (name=="joint") OneJoint(elem, &def->joint);

    // read geom
    else if (name=="geom") OneGeom(elem, &def->geom);

    // read site
    else if (name=="site") OneSite(elem, &def->site);

    // read camera
    else if (name=="camera") OneCamera(elem, &def->camera);

    // read light
    else if (name=="light") OneLight(elem, &def->light);

    // read pair
    else if (name=="pair") OnePair(elem, &def->pair);

    // read equality
    else if (name=="equality") OneEquality(elem, &def->equality);

    // read tendon
    else if (name=="tendon") OneTendon(elem, &def->tendon);

    // read actuator: general, motor, position, velocity, cylinder
    else if (name=="general"     ||
             name=="motor"       ||
             name=="position"    ||
             name=="velocity"    ||
             name=="damper"      ||
             name=="intvelocity" ||
             name=="cylinder"    ||
             name=="muscle") {
      OneActuator(elem, &def->actuator);
    }

    // advance
    elem = elem->NextSiblingElement();
  }

  // iterate over nested defaults
  elem = section->FirstChildElement();
  while (elem) {
    // get element name
    name = elem->Value();

    // read default
    if (name=="default") {
      Default(elem, thisid);
    }

    // advance
    elem = elem->NextSiblingElement();
  }
}



// custom section parser
void mjXReader::Custom(XMLElement* section) {
  string text, name;
  XMLElement* elem;
  double data[500];

  // iterate over child elements
  elem = section->FirstChildElement();
  while (elem) {
    // get sub-element name
    name = elem->Value();

    // numeric
    if (name=="numeric") {
      // create custom
      mjCNumeric* pnum = model->AddNumeric();
      GetXMLPos(elem, pnum);

      // read attributes
      ReadAttrTxt(elem, "name", pnum->name, true);
      if (ReadAttrInt(elem, "size", &pnum->size))
        for (int i=0; i<mjMIN(pnum->size, 500); i++) {
          data[i] = 0;
        } else {
        pnum->size = 501;
      }
      int len = ReadAttr(elem, "data", pnum->size, data, text, false, false);
      if (pnum->size==501) {
        pnum->size = len;
      }
      if (pnum->size<1 || pnum->size>500) {
        throw mjXError(elem, "custom field size must be between 1 and 500");
      }

      // copy data
      for (int i=0; i<pnum->size; i++) {
        pnum->data.push_back(data[i]);
      }
    }

    // text
    else if (name=="text") {
      // create custom
      mjCText* pte = model->AddText();
      GetXMLPos(elem, pte);

      // read attributes
      ReadAttrTxt(elem, "name", pte->name, true);
      ReadAttrTxt(elem, "data", text, true);
      if (text.empty()) {
        throw mjXError(elem, "text field cannot be empty");
      }

      // copy data
      pte->data = text;
    }

    // tuple
    else if (name=="tuple") {
      // create custom
      mjCTuple* ptu = model->AddTuple();
      GetXMLPos(elem, ptu);

      // read attributes
      ReadAttrTxt(elem, "name", ptu->name, true);

      // read objects and add
      XMLElement* obj = elem->FirstChildElement();
      while (obj) {
        // get sub-element name
        name = obj->Value();

        // new object
        if (name=="element") {
          // read type, check and assign
          ReadAttrTxt(obj, "objtype", text, true);
          mjtObj otype = (mjtObj)mju_str2Type(text.c_str());
          if (otype==mjOBJ_UNKNOWN) {
            throw mjXError(obj, "unknown object type");
          }
          ptu->objtype.push_back(otype);

          // read name and assign
          ReadAttrTxt(obj, "objname", text, true);
          ptu->objname.push_back(text);

          // read parameter and assign
          double oprm = 0;
          ReadAttr(obj, "prm", 1, &oprm, text);
          ptu->objprm.push_back(oprm);
        }

        // advance to next object
        obj = obj->NextSiblingElement();
      }
    }

    // advance to next element
    elem = elem->NextSiblingElement();
  }
}



// visual section parser
void mjXReader::Visual(XMLElement* section) {
  string text, name;
  XMLElement* elem;
  mjVisual* vis = &model->visual;

  // iterate over child elements
  elem = section->FirstChildElement();
  while (elem) {
    // get sub-element name
    name = elem->Value();

    // global sub-element
    if (name=="global") {
      ReadAttr(elem,    "fovy",      1, &vis->global.fovy,      text);
      ReadAttr(elem,    "ipd",       1, &vis->global.ipd,       text);
      ReadAttr(elem,    "azimuth",   1, &vis->global.azimuth,   text);
      ReadAttr(elem,    "elevation", 1, &vis->global.elevation, text);
      ReadAttr(elem,    "linewidth", 1, &vis->global.linewidth, text);
      ReadAttr(elem,    "glow",      1, &vis->global.glow,      text);
      ReadAttrInt(elem, "offwidth",     &vis->global.offwidth);
      ReadAttrInt(elem, "offheight",    &vis->global.offheight);
    }

    // quality sub-element
    else if (name=="quality") {
      ReadAttrInt(elem, "shadowsize", &vis->quality.shadowsize);
      ReadAttrInt(elem, "offsamples", &vis->quality.offsamples);
      ReadAttrInt(elem, "numslices",  &vis->quality.numslices);
      ReadAttrInt(elem, "numstacks",  &vis->quality.numstacks);
      ReadAttrInt(elem, "numquads",   &vis->quality.numquads);
    }

    // headlight sub-element
    else if (name=="headlight") {
      ReadAttr(elem, "ambient",  3, vis->headlight.ambient,  text);
      ReadAttr(elem, "diffuse",  3, vis->headlight.diffuse,  text);
      ReadAttr(elem, "specular", 3, vis->headlight.specular, text);
      ReadAttrInt(elem, "active",  &vis->headlight.active);
    }

    // map sub-element
    else if (name=="map") {
      ReadAttr(elem, "stiffness",      1, &vis->map.stiffness, text);
      ReadAttr(elem, "stiffnessrot",   1, &vis->map.stiffnessrot, text);
      ReadAttr(elem, "force",          1, &vis->map.force,     text);
      ReadAttr(elem, "torque",         1, &vis->map.torque,    text);
      ReadAttr(elem, "alpha",          1, &vis->map.alpha,     text);
      ReadAttr(elem, "fogstart",       1, &vis->map.fogstart,  text);
      ReadAttr(elem, "fogend",         1, &vis->map.fogend,    text);
      ReadAttr(elem, "znear",          1, &vis->map.znear,     text);
      ReadAttr(elem, "zfar",           1, &vis->map.zfar,      text);
      ReadAttr(elem, "haze",           1, &vis->map.haze,      text);
      ReadAttr(elem, "shadowclip",     1, &vis->map.shadowclip, text);
      ReadAttr(elem, "shadowscale",    1, &vis->map.shadowscale, text);
      ReadAttr(elem, "actuatortendon", 1, &vis->map.actuatortendon, text);
    }

    // scale sub-element
    else if (name=="scale") {
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
    }

    // rgba sub-element
    else if (name=="rgba") {
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
    }

    // advance to next element
    elem = elem->NextSiblingElement();
  }
}



// asset section parser
void mjXReader::Asset(XMLElement* section) {
  int n;
  string text, name;
  XMLElement* elem;

  // iterate over child elements
  elem = section->FirstChildElement();
  while (elem) {
    // get sub-element name
    name = elem->Value();

    // get class if specified, otherwise use default0
    mjCDef* def = GetClass(elem);
    if (!def) {
      def = model->defaults[0];
    }

    // texture sub-element
    if (name=="texture") {
      // create texture
      mjCTexture* ptex = model->AddTexture();
      GetXMLPos(elem, ptex);

      // read attributes
      if (MapValue(elem, "type", &n, texture_map, texture_sz)) {
        ptex->type = (mjtTexture)n;
      }
      ReadAttrTxt(elem, "name", ptex->name);
      ReadAttrTxt(elem, "file", ptex->file);
      ReadAttrInt(elem, "width", &ptex->width);
      ReadAttrInt(elem, "height", &ptex->height);
      ReadAttr(elem, "rgb1", 3, ptex->rgb1, text);
      ReadAttr(elem, "rgb2", 3, ptex->rgb2, text);
      ReadAttr(elem, "markrgb", 3, ptex->markrgb, text);
      ReadAttr(elem, "random", 1, &ptex->random, text);
      if (MapValue(elem, "builtin", &n, builtin_map, builtin_sz)) {
        ptex->builtin = (mjtBuiltin)n;
      }
      if (MapValue(elem, "mark", &n, mark_map, mark_sz)) {
        ptex->mark = (mjtMark)n;
      }
      if (MapValue(elem, "hflip", &n, bool_map, 2)) {
        ptex->hflip = (n!=0);
      }
      if (MapValue(elem, "vflip", &n, bool_map, 2)) {
        ptex->vflip = (n!=0);
      }

      // grid
      ReadAttr(elem, "gridsize", 2, ptex->gridsize, text);
      if (ReadAttrTxt(elem, "gridlayout", text)) {
        // check length
        if (text.length()>12) {
          throw mjXError(elem, "gridlayout length cannot exceed 12 characters");
        }
        if (text.length()!=ptex->gridsize[0]*ptex->gridsize[1]) {
          throw mjXError(elem, "gridlayout length must match gridsize");
        }

        memcpy(ptex->gridlayout, text.data(), text.length());
      }

      // separate files
      ReadAttrTxt(elem, "fileright", ptex->cubefiles[0]);
      ReadAttrTxt(elem, "fileleft",  ptex->cubefiles[1]);
      ReadAttrTxt(elem, "fileup",    ptex->cubefiles[2]);
      ReadAttrTxt(elem, "filedown",  ptex->cubefiles[3]);
      ReadAttrTxt(elem, "filefront", ptex->cubefiles[4]);
      ReadAttrTxt(elem, "fileback",  ptex->cubefiles[5]);
    }

    // material sub-element
    else if (name=="material") {
      // create material and parse
      mjCMaterial* pmat = model->AddMaterial(def);
      OneMaterial(elem, pmat);
    }

    // mesh sub-element
    else if (name=="mesh") {
      // create mesh and parse
      mjCMesh* pmesh = model->AddMesh(def);
      OneMesh(elem, pmesh);
    }

    // skin sub-element
    else if (name=="skin") {
      // create mesh and parse
      mjCSkin* pskin = model->AddSkin();
      OneSkin(elem, pskin);
    }

    // hfield sub-element
    else if (name=="hfield") {
      // create hfield
      mjCHField* phf = model->AddHField();
      GetXMLPos(elem, phf);

      // read attributes
      ReadAttrTxt(elem, "name", phf->name);
      ReadAttrTxt(elem, "file", phf->file);
      ReadAttrInt(elem, "nrow", &phf->nrow);
      ReadAttrInt(elem, "ncol", &phf->ncol);
      ReadAttr(elem, "size", 4, phf->size, text, true);

      // allocate buffer for dynamic hfield
      if (phf->file.empty() && phf->nrow>0 && phf->ncol>0) {
        phf->data = (float*) mju_malloc(phf->nrow*phf->ncol*sizeof(float));
        memset(phf->data, 0, phf->nrow*phf->ncol*sizeof(float));
      }
    }

    // advance to next element
    elem = elem->NextSiblingElement();
  }
}



// body/world section parser; recursive
void mjXReader::Body(XMLElement* section, mjCBody* pbody) {
  string text, name;
  XMLElement* elem;
  int n;

  // sanity check
  if (!pbody) {
    throw mjXError(section, "null body pointer");
  }

  // no attributes allowed in world body
  if (pbody->id==0 && section->FirstAttribute()) {
    throw mjXError(section, "World body cannot have attributes");
  }

  // iterate over sub-elements; attributes set while parsing parent body
  elem = section->FirstChildElement();
  while (elem) {
    // get sub-element name
    name = elem->Value();

    // get class if specified, otherwise use body
    mjCDef* def = GetClass(elem);
    if (!def) {
      def = pbody->def;
    }

    // inertial sub-element
    if (name=="inertial") {
      // no inertia allowed in world body
      if (pbody->id==0) {
        throw mjXError(elem, "World body cannot have inertia");
      }
      pbody->explicitinertial = true;
      ReadAttr(elem, "pos", 3, pbody->ipos, text, true);
      ReadAttr(elem, "quat", 4, pbody->iquat, text);
      ReadAttr(elem, "mass", 1, &pbody->mass, text, true);
      ReadAttr(elem, "diaginertia", 3, pbody->inertia, text);
      ReadAlternative(elem, pbody->ialt);
    }

    // joint sub-element
    else if (name=="joint") {
      // no joints allowed in world body
      if (pbody->id==0) {
        throw mjXError(elem, "World body cannot have joints");
      }

      // create joint and parse
      mjCJoint* pjoint = pbody->AddJoint(def);
      OneJoint(elem, pjoint);
    }

    // freejoint sub-element
    else if (name=="freejoint") {
      // no joints allowed in world body
      if (pbody->id==0) {
        throw mjXError(elem, "World body cannot have joints");
      }

      // create free joint without defaults
      mjCJoint* pjoint = pbody->AddJoint(NULL, true);

      // save defaults after creation, to make sure writing is ok
      pjoint->def = def;

      // read attributes
      ReadAttrTxt(elem, "name", pjoint->name);
      ReadAttrInt(elem, "group", &pjoint->group);
    }

    // geom sub-element
    else if (name=="geom") {
      // create geom and parse
      mjCGeom* pgeom = pbody->AddGeom(def);
      OneGeom(elem, pgeom);

      // discard visual
      if (!pgeom->contype && !pgeom->conaffinity && model->discardvisual) {
        delete pbody->geoms.back();
        pbody->geoms.pop_back();
      }
    }

    // site sub-element
    else if (name=="site") {
      // create site and parse
      mjCSite* psite = pbody->AddSite(def);
      OneSite(elem, psite);
    }

    // camera sub-element
    else if (name=="camera") {
      // create camera and parse
      mjCCamera* pcam = pbody->AddCamera(def);
      OneCamera(elem, pcam);
    }

    // light sub-element
    else if (name=="light") {
      // create light and parse
      mjCLight* plight = pbody->AddLight(def);
      OneLight(elem, plight);
    }

    // composite sub-element
    else if (name=="composite") {
      // create composite and parse
      OneComposite(elem, pbody, def);
    }

    // body sub-element
    else if (name=="body") {
      // read childdef
      mjCDef* childdef = 0;
      if (ReadAttrTxt(elem, "childclass", text)) {
        childdef = model->FindDef(text);
        if (!childdef) {
          throw mjXError(elem, "unknown default childclass");
        }
      }

      // create child body
      mjCBody* pchild = pbody->AddBody(childdef);
      GetXMLPos(elem, pchild);

      // read attributes
      ReadAttrTxt(elem, "name", pchild->name);
      ReadAttrTxt(elem, "childclass", pchild->classname);
      ReadAttr(elem, "pos", 3, pchild->pos, text);
      ReadAttr(elem, "quat", 4, pchild->quat, text);
      if (MapValue(elem, "mocap", &n, bool_map, 2)) {
        pchild->mocap = (n==1);
      }
      ReadAlternative(elem, pchild->alt);

      // read userdata
      ReadVector(elem, "user", pchild->userdata, text);

      // make recursive call
      Body(elem, pchild);
    }

    // no match
    else {
      throw mjXError(elem, "unrecognized model element '%s'", name.c_str());
    }

    // advance to next element
    elem = elem->NextSiblingElement();
  }
}



// contact section parser
void mjXReader::Contact(XMLElement* section) {
  string text, name;
  XMLElement* elem;

  // iterate over child elements
  elem = section->FirstChildElement();
  while (elem) {
    // get sub-element name
    name = elem->Value();

    // get class if specified, otherwise use default0
    mjCDef* def = GetClass(elem);
    if (!def) {
      def = model->defaults[0];
    }

    // geom pair to include
    if (name=="pair") {
      // create pair and parse
      mjCPair* ppair = model->AddPair(def);
      OnePair(elem, ppair);
    }

    // body pair to exclude
    else if (name=="exclude") {
      mjCBodyPair* pexclude = model->AddExclude();
      GetXMLPos(elem, pexclude);

      // read name and body names
      ReadAttrTxt(elem, "name", pexclude->name);
      ReadAttrTxt(elem, "body1", pexclude->bodyname1, true);
      ReadAttrTxt(elem, "body2", pexclude->bodyname2, true);
    }

    // advance to next element
    elem = elem->NextSiblingElement();
  }
}



// constraint section parser
void mjXReader::Equality(XMLElement* section) {
  XMLElement* elem;

  // iterate over child elements
  elem = section->FirstChildElement();
  while (elem) {
    // get class if specified, otherwise use default0
    mjCDef* def = GetClass(elem);
    if (!def) {
      def = model->defaults[0];
    }

    // create equality constraint and parse
    mjCEquality* pequality = model->AddEquality(def);
    OneEquality(elem, pequality);

    // advance to next element
    elem = elem->NextSiblingElement();
  }
}



// tendon section parser
void mjXReader::Tendon(XMLElement* section) {
  string text, text1;
  XMLElement* elem;
  double data;

  // iterate over child elements
  elem = section->FirstChildElement();
  while (elem) {
    // get class if specified, otherwise use default0
    mjCDef* def = GetClass(elem);
    if (!def) {
      def = model->defaults[0];
    }

    // create equality constraint and parse
    mjCTendon* pten = model->AddTendon(def);
    OneTendon(elem, pten);

    // process wrap sub-elements
    XMLElement* sub = elem->FirstChildElement();
    while (sub) {
      // get wrap type
      string wrap = sub->Value();

      // read attributes depending on type
      if (wrap=="site") {
        ReadAttrTxt(sub, "site", text, true);
        pten->WrapSite(text, sub->GetLineNum());
      }

      else if (wrap=="geom") {
        ReadAttrTxt(sub, "geom", text, true);
        if (!ReadAttrTxt(sub, "sidesite", text1)) {
          text1.clear();
        }
        pten->WrapGeom(text, text1, sub->GetLineNum());
      }

      else if (wrap=="pulley") {
        ReadAttr(sub, "divisor", 1, &data, text, true);
        pten->WrapPulley(data, sub->GetLineNum());
      }

      else if (wrap=="joint") {
        ReadAttrTxt(sub, "joint", text, true);
        ReadAttr(sub, "coef", 1, &data, text1, true);
        pten->WrapJoint(text, data, sub->GetLineNum());
      }

      else {
        throw mjXError(sub, "unknown wrap type");  // SHOULD NOT OCCUR
      }

      // advance to next sub-element
      sub = sub->NextSiblingElement();
    }

    // advance to next element
    elem = elem->NextSiblingElement();
  }
}



// actuator section parser
void mjXReader::Actuator(XMLElement* section) {
  XMLElement* elem;

  // iterate over child elements
  elem = section->FirstChildElement();
  while (elem) {
    // get class if specified, otherwise use default0
    mjCDef* def = GetClass(elem);
    if (!def) {
      def = model->defaults[0];
    }

    // create actuator and parse
    mjCActuator* pact = model->AddActuator(def);
    OneActuator(elem, pact);

    // advance to next element
    elem = elem->NextSiblingElement();
  }
}



// sensor section parser
void mjXReader::Sensor(XMLElement* section) {
  int n;
  string text;
  XMLElement* elem = section->FirstChildElement();
  while (elem) {
    // create sensor, get string type
    mjCSensor* psen = model->AddSensor();
    string type = elem->Value();

    // read name, noise, userdata
    ReadAttrTxt(elem, "name", psen->name);
    ReadAttr(elem, "cutoff", 1, &psen->cutoff, text);
    ReadAttr(elem, "noise", 1, &psen->noise, text);
    ReadVector(elem, "user", psen->userdata, text);

    // common robotic sensors, attached to a site
    if (type=="touch") {
      psen->type = mjSENS_TOUCH;
      psen->objtype = mjOBJ_SITE;
      ReadAttrTxt(elem, "site", psen->objname, true);
    } else if (type=="accelerometer") {
      psen->type = mjSENS_ACCELEROMETER;
      psen->objtype = mjOBJ_SITE;
      ReadAttrTxt(elem, "site", psen->objname, true);
    } else if (type=="velocimeter") {
      psen->type = mjSENS_VELOCIMETER;
      psen->objtype = mjOBJ_SITE;
      ReadAttrTxt(elem, "site", psen->objname, true);
    } else if (type=="gyro") {
      psen->type = mjSENS_GYRO;
      psen->objtype = mjOBJ_SITE;
      ReadAttrTxt(elem, "site", psen->objname, true);
    } else if (type=="force") {
      psen->type = mjSENS_FORCE;
      psen->objtype = mjOBJ_SITE;
      ReadAttrTxt(elem, "site", psen->objname, true);
    } else if (type=="torque") {
      psen->type = mjSENS_TORQUE;
      psen->objtype = mjOBJ_SITE;
      ReadAttrTxt(elem, "site", psen->objname, true);
    } else if (type=="magnetometer") {
      psen->type = mjSENS_MAGNETOMETER;
      psen->objtype = mjOBJ_SITE;
      ReadAttrTxt(elem, "site", psen->objname, true);
    } else if (type=="rangefinder") {
      psen->type = mjSENS_RANGEFINDER;
      psen->objtype = mjOBJ_SITE;
      ReadAttrTxt(elem, "site", psen->objname, true);
    }

    // sensors related to scalar joints, tendons, actuators
    else if (type=="jointpos") {
      psen->type = mjSENS_JOINTPOS;
      psen->objtype = mjOBJ_JOINT;
      ReadAttrTxt(elem, "joint", psen->objname, true);
    } else if (type=="jointvel") {
      psen->type = mjSENS_JOINTVEL;
      psen->objtype = mjOBJ_JOINT;
      ReadAttrTxt(elem, "joint", psen->objname, true);
    } else if (type=="tendonpos") {
      psen->type = mjSENS_TENDONPOS;
      psen->objtype = mjOBJ_TENDON;
      ReadAttrTxt(elem, "tendon", psen->objname, true);
    } else if (type=="tendonvel") {
      psen->type = mjSENS_TENDONVEL;
      psen->objtype = mjOBJ_TENDON;
      ReadAttrTxt(elem, "tendon", psen->objname, true);
    } else if (type=="actuatorpos") {
      psen->type = mjSENS_ACTUATORPOS;
      psen->objtype = mjOBJ_ACTUATOR;
      ReadAttrTxt(elem, "actuator", psen->objname, true);
    } else if (type=="actuatorvel") {
      psen->type = mjSENS_ACTUATORVEL;
      psen->objtype = mjOBJ_ACTUATOR;
      ReadAttrTxt(elem, "actuator", psen->objname, true);
    } else if (type=="actuatorfrc") {
      psen->type = mjSENS_ACTUATORFRC;
      psen->objtype = mjOBJ_ACTUATOR;
      ReadAttrTxt(elem, "actuator", psen->objname, true);
    }

    // sensors related to ball joints
    else if (type=="ballquat") {
      psen->type = mjSENS_BALLQUAT;
      psen->objtype = mjOBJ_JOINT;
      ReadAttrTxt(elem, "joint", psen->objname, true);
    } else if (type=="ballangvel") {
      psen->type = mjSENS_BALLANGVEL;
      psen->objtype = mjOBJ_JOINT;
      ReadAttrTxt(elem, "joint", psen->objname, true);
    }

    // joint and tendon limit sensors
    else if (type=="jointlimitpos") {
      psen->type = mjSENS_JOINTLIMITPOS;
      psen->objtype = mjOBJ_JOINT;
      ReadAttrTxt(elem, "joint", psen->objname, true);
    } else if (type=="jointlimitvel") {
      psen->type = mjSENS_JOINTLIMITVEL;
      psen->objtype = mjOBJ_JOINT;
      ReadAttrTxt(elem, "joint", psen->objname, true);
    } else if (type=="jointlimitfrc") {
      psen->type = mjSENS_JOINTLIMITFRC;
      psen->objtype = mjOBJ_JOINT;
      ReadAttrTxt(elem, "joint", psen->objname, true);
    } else if (type=="tendonlimitpos") {
      psen->type = mjSENS_TENDONLIMITPOS;
      psen->objtype = mjOBJ_TENDON;
      ReadAttrTxt(elem, "tendon", psen->objname, true);
    } else if (type=="tendonlimitvel") {
      psen->type = mjSENS_TENDONLIMITVEL;
      psen->objtype = mjOBJ_TENDON;
      ReadAttrTxt(elem, "tendon", psen->objname, true);
    } else if (type=="tendonlimitfrc") {
      psen->type = mjSENS_TENDONLIMITFRC;
      psen->objtype = mjOBJ_TENDON;
      ReadAttrTxt(elem, "tendon", psen->objname, true);
    }

    // sensors attached to an object with spatial frame: (x)body, geom, site, camera
    else if (type=="framepos") {
      psen->type = mjSENS_FRAMEPOS;
      ReadAttrTxt(elem, "objtype", text, true);
      psen->objtype = (mjtObj)mju_str2Type(text.c_str());
      ReadAttrTxt(elem, "objname", psen->objname, true);
      if (ReadAttrTxt(elem, "reftype", text)) {
        psen->reftype = (mjtObj)mju_str2Type(text.c_str());
        ReadAttrTxt(elem, "refname", psen->refname, true);
      } else if (ReadAttrTxt(elem, "refname", text)) {
        throw mjXError(elem, "refname '%s' given but reftype is missing", text.c_str());
      }
    } else if (type=="framequat") {
      psen->type = mjSENS_FRAMEQUAT;
      ReadAttrTxt(elem, "objtype", text, true);
      psen->objtype = (mjtObj)mju_str2Type(text.c_str());
      ReadAttrTxt(elem, "objname", psen->objname, true);
      if (ReadAttrTxt(elem, "reftype", text)) {
        psen->reftype = (mjtObj)mju_str2Type(text.c_str());
        ReadAttrTxt(elem, "refname", psen->refname, true);
      } else if (ReadAttrTxt(elem, "refname", text)) {
        throw mjXError(elem, "refname '%s' given but reftype is missing", text.c_str());
      }
    } else if (type=="framexaxis") {
      psen->type = mjSENS_FRAMEXAXIS;
      ReadAttrTxt(elem, "objtype", text, true);
      psen->objtype = (mjtObj)mju_str2Type(text.c_str());
      ReadAttrTxt(elem, "objname", psen->objname, true);
      if (ReadAttrTxt(elem, "reftype", text)) {
        psen->reftype = (mjtObj)mju_str2Type(text.c_str());
        ReadAttrTxt(elem, "refname", psen->refname, true);
      } else if (ReadAttrTxt(elem, "refname", text)) {
        throw mjXError(elem, "refname '%s' given but reftype is missing", text.c_str());
      }
    } else if (type=="frameyaxis") {
      psen->type = mjSENS_FRAMEYAXIS;
      ReadAttrTxt(elem, "objtype", text, true);
      psen->objtype = (mjtObj)mju_str2Type(text.c_str());
      ReadAttrTxt(elem, "objname", psen->objname, true);
      if (ReadAttrTxt(elem, "reftype", text)) {
        psen->reftype = (mjtObj)mju_str2Type(text.c_str());
        ReadAttrTxt(elem, "refname", psen->refname, true);
      } else if (ReadAttrTxt(elem, "refname", text)) {
        throw mjXError(elem, "refname '%s' given but reftype is missing", text.c_str());
      }
    } else if (type=="framezaxis") {
      psen->type = mjSENS_FRAMEZAXIS;
      ReadAttrTxt(elem, "objtype", text, true);
      psen->objtype = (mjtObj)mju_str2Type(text.c_str());
      ReadAttrTxt(elem, "objname", psen->objname, true);
      if (ReadAttrTxt(elem, "reftype", text)) {
        psen->reftype = (mjtObj)mju_str2Type(text.c_str());
        ReadAttrTxt(elem, "refname", psen->refname, true);
      } else if (ReadAttrTxt(elem, "refname", text)) {
        throw mjXError(elem, "refname '%s' given but reftype is missing", text.c_str());
      }
    } else if (type=="framelinvel") {
      psen->type = mjSENS_FRAMELINVEL;
      ReadAttrTxt(elem, "objtype", text, true);
      psen->objtype = (mjtObj)mju_str2Type(text.c_str());
      ReadAttrTxt(elem, "objname", psen->objname, true);
      if (ReadAttrTxt(elem, "reftype", text)) {
        psen->reftype = (mjtObj)mju_str2Type(text.c_str());
        ReadAttrTxt(elem, "refname", psen->refname, true);
      } else if (ReadAttrTxt(elem, "refname", text)) {
        throw mjXError(elem, "refname '%s' given but reftype is missing", text.c_str());
      }
    } else if (type=="frameangvel") {
      psen->type = mjSENS_FRAMEANGVEL;
      ReadAttrTxt(elem, "objtype", text, true);
      psen->objtype = (mjtObj)mju_str2Type(text.c_str());
      ReadAttrTxt(elem, "objname", psen->objname, true);
      if (ReadAttrTxt(elem, "reftype", text)) {
        psen->reftype = (mjtObj)mju_str2Type(text.c_str());
        ReadAttrTxt(elem, "refname", psen->refname, true);
      } else if (ReadAttrTxt(elem, "refname", text)) {
        throw mjXError(elem, "refname '%s' given but reftype is missing", text.c_str());
      }
    } else if (type=="framelinacc") {
      psen->type = mjSENS_FRAMELINACC;
      ReadAttrTxt(elem, "objtype", text, true);
      psen->objtype = (mjtObj)mju_str2Type(text.c_str());
      ReadAttrTxt(elem, "objname", psen->objname, true);
    } else if (type=="frameangacc") {
      psen->type = mjSENS_FRAMEANGACC;
      ReadAttrTxt(elem, "objtype", text, true);
      psen->objtype = (mjtObj)mju_str2Type(text.c_str());
      ReadAttrTxt(elem, "objname", psen->objname, true);
    }

    // sensors related to kinematic subtrees; attached to a body (which is the subtree root)
    else if (type=="subtreecom") {
      psen->type = mjSENS_SUBTREECOM;
      psen->objtype = mjOBJ_BODY;
      ReadAttrTxt(elem, "body", psen->objname, true);
    } else if (type=="subtreelinvel") {
      psen->type = mjSENS_SUBTREELINVEL;
      psen->objtype = mjOBJ_BODY;
      ReadAttrTxt(elem, "body", psen->objname, true);
    } else if (type=="subtreeangmom") {
      psen->type = mjSENS_SUBTREEANGMOM;
      psen->objtype = mjOBJ_BODY;
      ReadAttrTxt(elem, "body", psen->objname, true);
    }

    // global sensors
    else if (type=="clock") {
      psen->type = mjSENS_CLOCK;
      psen->objtype = mjOBJ_UNKNOWN;
    }

    // user-defined sensor
    else if (type=="user") {
      psen->type = mjSENS_USER;
      ReadAttrTxt(elem, "objtype", text, true);
      psen->objtype = (mjtObj)mju_str2Type(text.c_str());
      ReadAttrTxt(elem, "objname", psen->objname, true);
      ReadAttrInt(elem, "dim", &psen->dim, true);

      // keywords
      MapValue(elem, "needstage", &n, stage_map, stage_sz, true);
      psen->needstage = (mjtStage)n;
      MapValue(elem, "datatype", &n, datatype_map, datatype_sz, true);
      psen->datatype = (mjtDataType)n;
    }

    // advance to next element
    elem = elem->NextSiblingElement();
  }
}



// keyframe section parser
void mjXReader::Keyframe(XMLElement* section) {
  string text;
  XMLElement* elem;
  int n;
  double data[1000];

  // iterate over child elements
  elem = section->FirstChildElement();
  while (elem) {
    // add keyframe
    mjCKey* pk = model->AddKey();

    // read name, time
    ReadAttrTxt(elem, "name", pk->name);
    ReadAttr(elem, "time", 1, &pk->time, text);

    // read qpos
    n = ReadAttr(elem, "qpos", 1000, data, text, false, false);
    if (n) {
      pk->qpos.resize(n);
      mjuu_copyvec(pk->qpos.data(), data, n);
    }

    // read qvel
    n = ReadAttr(elem, "qvel", 1000, data, text, false, false);
    if (n) {
      pk->qvel.resize(n);
      mjuu_copyvec(pk->qvel.data(), data, n);
    }

    // read act
    n = ReadAttr(elem, "act", 1000, data, text, false, false);
    if (n) {
      pk->act.resize(n);
      mjuu_copyvec(pk->act.data(), data, n);
    }

    // read mpos
    n = ReadAttr(elem, "mpos", 1000, data, text, false, false);
    if (n) {
      pk->mpos.resize(n);
      mjuu_copyvec(pk->mpos.data(), data, n);
    }

    // read mquat
    n = ReadAttr(elem, "mquat", 1000, data, text, false, false);
    if (n) {
      pk->mquat.resize(n);
      mjuu_copyvec(pk->mquat.data(), data, n);
    }

    // read ctrl
    n = ReadAttr(elem, "ctrl", 1000, data, text, false, false);
    if (n) {
      pk->ctrl.resize(n);
      mjuu_copyvec(pk->ctrl.data(), data, n);
    }

    // advance to next element
    elem = elem->NextSiblingElement();
  }
}



// get defaults class
mjCDef* mjXReader::GetClass(XMLElement* section) {
  string text;
  mjCDef* def = 0;

  if (ReadAttrTxt(section, "class", text)) {
    def = model->FindDef(text);
    if (!def) {
      throw mjXError(section, "unknown default class");
    }
  }

  return def;
}



// get xml position
void mjXReader::GetXMLPos(XMLElement* elem, mjCBase* obj) {
  obj->xmlpos[0] = elem->GetLineNum();
  obj->xmlpos[1] = -1;
}
