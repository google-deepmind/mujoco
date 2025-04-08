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

#ifndef MUJOCO_MJVISUALIZE_H_
#define MUJOCO_MJVISUALIZE_H_

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>


#define mjNGROUP        6         // number of geom, site, joint, skin groups with visflags
#define mjMAXLIGHT      100       // maximum number of lights in a scene
#define mjMAXOVERLAY    500       // maximum number of characters in overlay text
#define mjMAXLINE       100       // maximum number of lines per plot
#define mjMAXLINEPNT    1000      // maximum number points per line
#define mjMAXPLANEGRID  200       // maximum number of grid divisions for plane


//---------------------------------- primitive types (mjt) -----------------------------------------

typedef enum mjtCatBit_ {         // bitflags for mjvGeom category
  mjCAT_STATIC        = 1,        // model elements in body 0
  mjCAT_DYNAMIC       = 2,        // model elements in all other bodies
  mjCAT_DECOR         = 4,        // decorative geoms
  mjCAT_ALL           = 7         // select all categories
} mjtCatBit;


typedef enum mjtMouse_ {          // mouse interaction mode
  mjMOUSE_NONE        = 0,        // no action
  mjMOUSE_ROTATE_V,               // rotate, vertical plane
  mjMOUSE_ROTATE_H,               // rotate, horizontal plane
  mjMOUSE_MOVE_V,                 // move, vertical plane
  mjMOUSE_MOVE_H,                 // move, horizontal plane
  mjMOUSE_ZOOM,                   // zoom
  mjMOUSE_SELECT                  // selection
} mjtMouse;


typedef enum mjtPertBit_ {        // mouse perturbations
  mjPERT_TRANSLATE    = 1,        // translation
  mjPERT_ROTATE       = 2         // rotation
} mjtPertBit;


typedef enum mjtCamera_ {         // abstract camera type
  mjCAMERA_FREE       = 0,        // free camera
  mjCAMERA_TRACKING,              // tracking camera; uses trackbodyid
  mjCAMERA_FIXED,                 // fixed camera; uses fixedcamid
  mjCAMERA_USER                   // user is responsible for setting OpenGL camera
} mjtCamera;


typedef enum mjtLabel_ {          // object labeling
  mjLABEL_NONE        = 0,        // nothing
  mjLABEL_BODY,                   // body labels
  mjLABEL_JOINT,                  // joint labels
  mjLABEL_GEOM,                   // geom labels
  mjLABEL_SITE,                   // site labels
  mjLABEL_CAMERA,                 // camera labels
  mjLABEL_LIGHT,                  // light labels
  mjLABEL_TENDON,                 // tendon labels
  mjLABEL_ACTUATOR,               // actuator labels
  mjLABEL_CONSTRAINT,             // constraint labels
  mjLABEL_FLEX,                   // flex labels
  mjLABEL_SKIN,                   // skin labels
  mjLABEL_SELECTION,              // selected object
  mjLABEL_SELPNT,                 // coordinates of selection point
  mjLABEL_CONTACTPOINT,           // contact information
  mjLABEL_CONTACTFORCE,           // magnitude of contact force
  mjLABEL_ISLAND,                 // id of island

  mjNLABEL                        // number of label types
} mjtLabel;


typedef enum mjtFrame_ {          // frame visualization
  mjFRAME_NONE        = 0,        // no frames
  mjFRAME_BODY,                   // body frames
  mjFRAME_GEOM,                   // geom frames
  mjFRAME_SITE,                   // site frames
  mjFRAME_CAMERA,                 // camera frames
  mjFRAME_LIGHT,                  // light frames
  mjFRAME_CONTACT,                // contact frames
  mjFRAME_WORLD,                  // world frame

  mjNFRAME                        // number of visualization frames
} mjtFrame;


typedef enum mjtVisFlag_ {        // flags enabling model element visualization
  mjVIS_CONVEXHULL    = 0,        // mesh convex hull
  mjVIS_TEXTURE,                  // textures
  mjVIS_JOINT,                    // joints
  mjVIS_CAMERA,                   // cameras
  mjVIS_ACTUATOR,                 // actuators
  mjVIS_ACTIVATION,               // activations
  mjVIS_LIGHT,                    // lights
  mjVIS_TENDON,                   // tendons
  mjVIS_RANGEFINDER,              // rangefinder sensors
  mjVIS_CONSTRAINT,               // point constraints
  mjVIS_INERTIA,                  // equivalent inertia boxes
  mjVIS_SCLINERTIA,               // scale equivalent inertia boxes with mass
  mjVIS_PERTFORCE,                // perturbation force
  mjVIS_PERTOBJ,                  // perturbation object
  mjVIS_CONTACTPOINT,             // contact points
  mjVIS_ISLAND,                   // constraint islands
  mjVIS_CONTACTFORCE,             // contact force
  mjVIS_CONTACTSPLIT,             // split contact force into normal and tangent
  mjVIS_TRANSPARENT,              // make dynamic geoms more transparent
  mjVIS_AUTOCONNECT,              // auto connect joints and body coms
  mjVIS_COM,                      // center of mass
  mjVIS_SELECT,                   // selection point
  mjVIS_STATIC,                   // static bodies
  mjVIS_SKIN,                     // skin
  mjVIS_FLEXVERT,                 // flex vertices
  mjVIS_FLEXEDGE,                 // flex edges
  mjVIS_FLEXFACE,                 // flex element faces
  mjVIS_FLEXSKIN,                 // flex smooth skin (disables the rest)
  mjVIS_BODYBVH,                  // body bounding volume hierarchy
  mjVIS_FLEXBVH,                  // flex bounding volume hierarchy
  mjVIS_MESHBVH,                  // mesh bounding volume hierarchy
  mjVIS_SDFITER,                  // iterations of SDF gradient descent

  mjNVISFLAG                      // number of visualization flags
} mjtVisFlag;


typedef enum mjtRndFlag_ {        // flags enabling rendering effects
  mjRND_SHADOW        = 0,        // shadows
  mjRND_WIREFRAME,                // wireframe
  mjRND_REFLECTION,               // reflections
  mjRND_ADDITIVE,                 // additive transparency
  mjRND_SKYBOX,                   // skybox
  mjRND_FOG,                      // fog
  mjRND_HAZE,                     // haze
  mjRND_SEGMENT,                  // segmentation with random color
  mjRND_IDCOLOR,                  // segmentation with segid+1 color
  mjRND_CULL_FACE,                // cull backward faces

  mjNRNDFLAG                      // number of rendering flags
} mjtRndFlag;


typedef enum mjtStereo_ {         // type of stereo rendering
  mjSTEREO_NONE       = 0,        // no stereo; use left eye only
  mjSTEREO_QUADBUFFERED,          // quad buffered; revert to side-by-side if no hardware support
  mjSTEREO_SIDEBYSIDE             // side-by-side
} mjtStereo;


//---------------------------------- mjvPerturb ----------------------------------------------------

struct mjvPerturb_ {              // object selection and perturbation
  int      select;                // selected body id; non-positive: none
  int      flexselect;            // selected flex id; negative: none
  int      skinselect;            // selected skin id; negative: none
  int      active;                // perturbation bitmask (mjtPertBit)
  int      active2;               // secondary perturbation bitmask (mjtPertBit)
  mjtNum   refpos[3];             // reference position for selected object
  mjtNum   refquat[4];            // reference orientation for selected object
  mjtNum   refselpos[3];          // reference position for selection point
  mjtNum   localpos[3];           // selection point in object coordinates
  mjtNum   localmass;             // spatial inertia at selection point
  mjtNum   scale;                 // relative mouse motion-to-space scaling (set by initPerturb)
};
typedef struct mjvPerturb_ mjvPerturb;


//---------------------------------- mjvCamera -----------------------------------------------------

struct mjvCamera_ {               // abstract camera
  // type and ids
  int      type;                  // camera type (mjtCamera)
  int      fixedcamid;            // fixed camera id
  int      trackbodyid;           // body id to track

  // abstract camera pose specification
  mjtNum   lookat[3];             // lookat point
  mjtNum   distance;              // distance to lookat point or tracked body
  mjtNum   azimuth;               // camera azimuth (deg)
  mjtNum   elevation;             // camera elevation (deg)

  // orthographic / perspective
  int      orthographic;          // 0: perspective; 1: orthographic
};
typedef struct mjvCamera_ mjvCamera;


//---------------------------------- mjvGLCamera ---------------------------------------------------

struct mjvGLCamera_ {             // OpenGL camera
  // camera frame
  float    pos[3];                // position
  float    forward[3];            // forward direction
  float    up[3];                 // up direction

  // camera projection
  float    frustum_center;        // hor. center (left,right set to match aspect)
  float    frustum_width;         // width (not used for rendering)
  float    frustum_bottom;        // bottom
  float    frustum_top;           // top
  float    frustum_near;          // near
  float    frustum_far;           // far

  // orthographic / perspective
  int      orthographic;          // 0: perspective; 1: orthographic
};
typedef struct mjvGLCamera_ mjvGLCamera;


//---------------------------------- mjvGeom -------------------------------------------------------

struct mjvGeom_ {                 // abstract geom
  // type info
  int      type;                  // geom type (mjtGeom)
  int      dataid;                // mesh, hfield or plane id; -1: none
  int      objtype;               // mujoco object type; mjOBJ_UNKNOWN for decor
  int      objid;                 // mujoco object id; -1 for decor
  int      category;              // visual category
  int      matid;                 // material id; -1: no textured material
  int      texcoord;              // mesh or flex geom has texture coordinates
  int      segid;                 // segmentation id; -1: not shown

  // spatial transform
  float    size[3];               // size parameters
  float    pos[3];                // Cartesian position
  float    mat[9];                // Cartesian orientation

  // material properties
  float    rgba[4];               // color and transparency
  float    emission;              // emission coef
  float    specular;              // specular coef
  float    shininess;             // shininess coef
  float    reflectance;           // reflectance coef

  char     label[100];            // text label

  // transparency rendering (set internally)
  float    camdist;               // distance to camera (used by sorter)
  float    modelrbound;           // geom rbound from model, 0 if not model geom
  mjtByte  transparent;           // treat geom as transparent
};
typedef struct mjvGeom_ mjvGeom;


//---------------------------------- mjvLight ------------------------------------------------------

struct mjvLight_ {                // OpenGL light
  float    pos[3];                // position rel. to body frame
  float    dir[3];                // direction rel. to body frame
  float    attenuation[3];        // OpenGL attenuation (quadratic model)
  float    cutoff;                // OpenGL cutoff
  float    exponent;              // OpenGL exponent
  float    ambient[3];            // ambient rgb (alpha=1)
  float    diffuse[3];            // diffuse rgb (alpha=1)
  float    specular[3];           // specular rgb (alpha=1)
  mjtByte  headlight;             // headlight
  mjtByte  directional;           // directional light
  mjtByte  castshadow;            // does light cast shadows
  float    bulbradius;            // bulb radius for soft shadows
};
typedef struct mjvLight_ mjvLight;


//---------------------------------- mjvOption -----------------------------------------------------

struct mjvOption_ {                  // abstract visualization options
  int      label;                    // what objects to label (mjtLabel)
  int      frame;                    // which frame to show (mjtFrame)
  mjtByte  geomgroup[mjNGROUP];      // geom visualization by group
  mjtByte  sitegroup[mjNGROUP];      // site visualization by group
  mjtByte  jointgroup[mjNGROUP];     // joint visualization by group
  mjtByte  tendongroup[mjNGROUP];    // tendon visualization by group
  mjtByte  actuatorgroup[mjNGROUP];  // actuator visualization by group
  mjtByte  flexgroup[mjNGROUP];      // flex visualization by group
  mjtByte  skingroup[mjNGROUP];      // skin visualization by group
  mjtByte  flags[mjNVISFLAG];        // visualization flags (indexed by mjtVisFlag)
  int      bvh_depth;                // depth of the bounding volume hierarchy to be visualized
  int      flex_layer;               // element layer to be visualized for 3D flex
};
typedef struct mjvOption_ mjvOption;


//---------------------------------- mjvScene ------------------------------------------------------

struct mjvScene_ {                // abstract scene passed to OpenGL renderer
  // abstract geoms
  int      maxgeom;               // size of allocated geom buffer
  int      ngeom;                 // number of geoms currently in buffer
  mjvGeom* geoms;                 // buffer for geoms (ngeom)
  int*     geomorder;             // buffer for ordering geoms by distance to camera (ngeom)

  // flex data
  int      nflex;                 // number of flexes
  int*     flexedgeadr;           // address of flex edges (nflex)
  int*     flexedgenum;           // number of edges in flex (nflex)
  int*     flexvertadr;           // address of flex vertices (nflex)
  int*     flexvertnum;           // number of vertices in flex (nflex)
  int*     flexfaceadr;           // address of flex faces (nflex)
  int*     flexfacenum;           // number of flex faces allocated (nflex)
  int*     flexfaceused;          // number of flex faces currently in use (nflex)
  int*     flexedge;              // flex edge data (2*nflexedge)
  float*   flexvert;              // flex vertices (3*nflexvert)
  float*   flexface;              // flex faces vertices (9*sum(flexfacenum))
  float*   flexnormal;            // flex face normals (9*sum(flexfacenum))
  float*   flextexcoord;          // flex face texture coordinates (6*sum(flexfacenum))
  mjtByte  flexvertopt;           // copy of mjVIS_FLEXVERT mjvOption flag
  mjtByte  flexedgeopt;           // copy of mjVIS_FLEXEDGE mjvOption flag
  mjtByte  flexfaceopt;           // copy of mjVIS_FLEXFACE mjvOption flag
  mjtByte  flexskinopt;           // copy of mjVIS_FLEXSKIN mjvOption flag

  // skin data
  int      nskin;                 // number of skins
  int*     skinfacenum;           // number of faces in skin (nskin)
  int*     skinvertadr;           // address of skin vertices (nskin)
  int*     skinvertnum;           // number of vertices in skin (nskin)
  float*   skinvert;              // skin vertex data (3*nskinvert)
  float*   skinnormal;            // skin normal data (3*nskinvert)

  // OpenGL lights
  int      nlight;                // number of lights currently in buffer
  mjvLight lights[mjMAXLIGHT];    // buffer for lights (nlight)

  // OpenGL cameras
  mjvGLCamera camera[2];          // left and right camera

  // OpenGL model transformation
  mjtByte  enabletransform;       // enable model transformation
  float    translate[3];          // model translation
  float    rotate[4];             // model quaternion rotation
  float    scale;                 // model scaling

  // OpenGL rendering effects
  int      stereo;                // stereoscopic rendering (mjtStereo)
  mjtByte  flags[mjNRNDFLAG];     // rendering flags (indexed by mjtRndFlag)

  // framing
  int      framewidth;            // frame pixel width; 0: disable framing
  float    framergb[3];           // frame color
};
typedef struct mjvScene_ mjvScene;


//---------------------------------- mjvFigure -----------------------------------------------------

struct mjvFigure_ {               // abstract 2D figure passed to OpenGL renderer
  // enable flags
  int     flg_legend;             // show legend
  int     flg_ticklabel[2];       // show grid tick labels (x,y)
  int     flg_extend;             // automatically extend axis ranges to fit data
  int     flg_barplot;            // isolated line segments (i.e. GL_LINES)
  int     flg_selection;          // vertical selection line
  int     flg_symmetric;          // symmetric y-axis

  // style settings
  float   linewidth;              // line width
  float   gridwidth;              // grid line width
  int     gridsize[2];            // number of grid points in (x,y)
  float   gridrgb[3];             // grid line rgb
  float   figurergba[4];          // figure color and alpha
  float   panergba[4];            // pane color and alpha
  float   legendrgba[4];          // legend color and alpha
  float   textrgb[3];             // text color
  float   linergb[mjMAXLINE][3];  // line colors
  float   range[2][2];            // axis ranges; (min>=max) automatic
  char    xformat[20];            // x-tick label format for sprintf
  char    yformat[20];            // y-tick label format for sprintf
  char    minwidth[20];           // string used to determine min y-tick width

  // text labels
  char    title[1000];            // figure title; subplots separated with 2+ spaces
  char    xlabel[100];            // x-axis label
  char    linename[mjMAXLINE][100];  // line names for legend

  // dynamic settings
  int     legendoffset;           // number of lines to offset legend
  int     subplot;                // selected subplot (for title rendering)
  int     highlight[2];           // if point is in legend rect, highlight line
  int     highlightid;            // if id>=0 and no point, highlight id
  float   selection;              // selection line x-value

  // line data
  int     linepnt[mjMAXLINE];     // number of points in line; (0) disable
  float   linedata[mjMAXLINE][2*mjMAXLINEPNT];  // line data (x,y)

  // output from renderer
  int     xaxispixel[2];          // range of x-axis in pixels
  int     yaxispixel[2];          // range of y-axis in pixels
  float   xaxisdata[2];           // range of x-axis in data units
  float   yaxisdata[2];           // range of y-axis in data units
};
typedef struct mjvFigure_ mjvFigure;


//---------------------------------- mjvSceneState -------------------------------------------------

struct mjvSceneState_ {
  int nbuffer;                     // size of the buffer in bytes
  void* buffer;                    // heap-allocated memory for all arrays in this struct
  int maxgeom;                     // maximum number of mjvGeom supported by this state object
  mjvScene scratch;                // scratch space for vis geoms inserted by the user and plugins

  // fields in mjModel that are necessary to re-render a scene
  struct {
    int nv;
    int nu;
    int na;
    int nbody;
    int nbvh;
    int nbvhstatic;
    int njnt;
    int ngeom;
    int nsite;
    int ncam;
    int nlight;
    int nmesh;
    int nskin;
    int nflex;
    int nflexvert;
    int nflextexcoord;
    int nskinvert;
    int nskinface;
    int nskinbone;
    int nskinbonevert;
    int nmat;
    int neq;
    int ntendon;
    int ntree;
    int nwrap;
    int nsensor;
    int nnames;
    int npaths;
    int nsensordata;
    int narena;

    mjOption opt;
    mjVisual vis;
    mjStatistic stat;

    int* body_parentid;
    int* body_rootid;
    int* body_weldid;
    int* body_mocapid;
    int* body_jntnum;
    int* body_jntadr;
    int* body_dofnum;
    int* body_dofadr;
    int* body_geomnum;
    int* body_geomadr;
    mjtNum* body_iquat;
    mjtNum* body_mass;
    mjtNum* body_inertia;
    int* body_bvhadr;
    int* body_bvhnum;

    int* bvh_depth;
    int* bvh_child;
    int* bvh_nodeid;
    mjtNum* bvh_aabb;

    int* jnt_type;
    int* jnt_bodyid;
    int* jnt_group;

    int* geom_type;
    int* geom_bodyid;
    int* geom_contype;
    int* geom_conaffinity;
    int* geom_dataid;
    int* geom_matid;
    int* geom_group;
    mjtNum* geom_size;
    mjtNum* geom_aabb;
    mjtNum* geom_rbound;
    float* geom_rgba;

    int* site_type;
    int* site_bodyid;
    int* site_matid;
    int* site_group;
    mjtNum* site_size;
    float* site_rgba;

    int* cam_orthographic;
    mjtNum* cam_fovy;
    mjtNum* cam_ipd;
    int* cam_resolution;
    float* cam_sensorsize;
    float* cam_intrinsic;

    mjtByte* light_directional;
    mjtByte* light_castshadow;
    float* light_bulbradius;
    mjtByte* light_active;
    float* light_attenuation;
    float* light_cutoff;
    float* light_exponent;
    float* light_ambient;
    float* light_diffuse;
    float* light_specular;

    mjtByte* flex_flatskin;
    int* flex_dim;
    int* flex_matid;
    int* flex_group;
    int* flex_interp;
    int* flex_nodeadr;
    int* flex_nodenum;
    int* flex_nodebodyid;
    int* flex_vertadr;
    int* flex_vertnum;
    int* flex_elem;
    int* flex_elemtexcoord;
    int* flex_elemlayer;
    int* flex_elemadr;
    int* flex_elemnum;
    int* flex_elemdataadr;
    int* flex_shell;
    int* flex_shellnum;
    int* flex_shelldataadr;
    int* flex_texcoordadr;
    int* flex_bvhadr;
    int* flex_bvhnum;
    mjtByte* flex_centered;
    mjtNum* flex_node;
    mjtNum* flex_radius;
    float* flex_rgba;
    float* flex_texcoord;

    int* hfield_pathadr;

    int* mesh_bvhadr;
    int* mesh_bvhnum;
    int* mesh_texcoordadr;
    int* mesh_graphadr;
    int* mesh_pathadr;

    int* skin_matid;
    int* skin_group;
    float* skin_rgba;
    float* skin_inflate;
    int* skin_vertadr;
    int* skin_vertnum;
    int* skin_texcoordadr;
    int* skin_faceadr;
    int* skin_facenum;
    int* skin_boneadr;
    int* skin_bonenum;
    float* skin_vert;
    int* skin_face;
    int* skin_bonevertadr;
    int* skin_bonevertnum;
    float* skin_bonebindpos;
    float* skin_bonebindquat;
    int* skin_bonebodyid;
    int* skin_bonevertid;
    float* skin_bonevertweight;
    int* skin_pathadr;

    int* tex_pathadr;

    int* mat_texid;
    mjtByte* mat_texuniform;
    float* mat_texrepeat;
    float* mat_emission;
    float* mat_specular;
    float* mat_shininess;
    float* mat_reflectance;
    float* mat_metallic;
    float* mat_roughness;
    float* mat_rgba;

    int* eq_type;
    int* eq_obj1id;
    int* eq_obj2id;
    int* eq_objtype;
    mjtNum* eq_data;

    int* tendon_num;
    int* tendon_matid;
    int* tendon_group;
    mjtByte* tendon_limited;
    mjtByte* tendon_actfrclimited;
    mjtNum* tendon_width;
    mjtNum* tendon_range;
    mjtNum* tendon_actfrcrange;
    mjtNum* tendon_stiffness;
    mjtNum* tendon_damping;
    mjtNum* tendon_frictionloss;
    mjtNum* tendon_lengthspring;
    float* tendon_rgba;

    int* actuator_trntype;
    int* actuator_dyntype;
    int* actuator_trnid;
    int* actuator_actadr;
    int* actuator_actnum;
    int* actuator_group;
    mjtByte* actuator_ctrllimited;
    mjtByte* actuator_actlimited;
    mjtNum* actuator_ctrlrange;
    mjtNum* actuator_actrange;
    mjtNum* actuator_cranklength;

    int* sensor_type;
    int* sensor_objid;
    int* sensor_adr;

    int* name_bodyadr;
    int* name_jntadr;
    int* name_geomadr;
    int* name_siteadr;
    int* name_camadr;
    int* name_lightadr;
    int* name_eqadr;
    int* name_tendonadr;
    int* name_actuatoradr;
    char* names;
    char* paths;
  } model;

  // fields in mjData that are necessary to re-render a scene
  struct {
    mjWarningStat warning[mjNWARNING];

    int nefc;
    int ncon;
    int nisland;

    mjtNum time;

    mjtNum* act;

    mjtNum* ctrl;
    mjtNum* xfrc_applied;
    mjtByte* eq_active;

    mjtNum* sensordata;

    mjtNum* xpos;
    mjtNum* xquat;
    mjtNum* xmat;
    mjtNum* xipos;
    mjtNum* ximat;
    mjtNum* xanchor;
    mjtNum* xaxis;
    mjtNum* geom_xpos;
    mjtNum* geom_xmat;
    mjtNum* site_xpos;
    mjtNum* site_xmat;
    mjtNum* cam_xpos;
    mjtNum* cam_xmat;
    mjtNum* light_xpos;
    mjtNum* light_xdir;

    mjtNum* subtree_com;

    int* ten_wrapadr;
    int* ten_wrapnum;
    int* wrap_obj;
    mjtNum* ten_length;
    mjtNum* wrap_xpos;

    mjtNum* bvh_aabb_dyn;
    mjtByte* bvh_active;
    int* island_dofadr;
    int* island_dofind;
    int* dof_island;
    int* efc_island;
    int* tendon_efcadr;

    mjtNum* flexvert_xpos;

    mjContact* contact;
    mjtNum* efc_force;
    void* arena;
  } data;
};
typedef struct mjvSceneState_ mjvSceneState;

#endif  // MUJOCO_MJVISUALIZE_H_
