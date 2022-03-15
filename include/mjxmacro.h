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

#ifndef MUJOCO_MJXMACRO_H_
#define MUJOCO_MJXMACRO_H_


//-------------------------------- mjOption --------------------------------------------------------

// scalar fields of mjOption
#define MJOPTION_FLOATS             \
    X( mjtNum,  timestep        )   \
    X( mjtNum,  apirate         )   \
    X( mjtNum,  impratio        )   \
    X( mjtNum,  tolerance       )   \
    X( mjtNum,  noslip_tolerance )  \
    X( mjtNum,  mpr_tolerance   )   \
    X( mjtNum,  density         )   \
    X( mjtNum,  viscosity       )   \
    X( mjtNum,  o_margin        )   \


#define MJOPTION_INTS               \
    X( int,     integrator      )   \
    X( int,     collision       )   \
    X( int,     cone            )   \
    X( int,     jacobian        )   \
    X( int,     solver          )   \
    X( int,     iterations      )   \
    X( int,     noslip_iterations ) \
    X( int,     mpr_iterations  )   \
    X( int,     disableflags    )   \
    X( int,     enableflags     )


#define MJOPTION_SCALARS            \
    MJOPTION_FLOATS                 \
    MJOPTION_INTS


// vector fields of mjOption
#define MJOPTION_VECTORS            \
    X( gravity,         3       )   \
    X( wind,            3       )   \
    X( magnetic,        3       )   \
    X( o_solref,        mjNREF  )   \
    X( o_solimp,        mjNIMP  )


//-------------------------------- mjModel ---------------------------------------------------------

// int fields of mjModel
#define MJMODEL_INTS        \
    X( nq )                 \
    X( nv )                 \
    X( nu )                 \
    X( na )                 \
    X( nbody )              \
    X( njnt )               \
    X( ngeom )              \
    X( nsite )              \
    X( ncam )               \
    X( nlight )             \
    X( nmesh )              \
    X( nmeshvert )          \
    X( nmeshtexvert )       \
    X( nmeshface )          \
    X( nmeshgraph )         \
    X( nskin )              \
    X( nskinvert )          \
    X( nskintexvert )       \
    X( nskinface )          \
    X( nskinbone )          \
    X( nskinbonevert )      \
    X( nhfield )            \
    X( nhfielddata )        \
    X( ntex )               \
    X( ntexdata )           \
    X( nmat )               \
    X( npair )              \
    X( nexclude )           \
    X( neq )                \
    X( ntendon )            \
    X( nwrap )              \
    X( nsensor )            \
    X( nnumeric )           \
    X( nnumericdata )       \
    X( ntext )              \
    X( ntextdata )          \
    X( ntuple )             \
    X( ntupledata )         \
    X( nkey )               \
    X( nmocap )             \
    X( nuser_body )         \
    X( nuser_jnt )          \
    X( nuser_geom )         \
    X( nuser_site )         \
    X( nuser_cam )          \
    X( nuser_tendon )       \
    X( nuser_actuator )     \
    X( nuser_sensor )       \
    X( nnames )             \
    X( nM )                 \
    X( nemax )              \
    X( njmax )              \
    X( nconmax )            \
    X( nstack )             \
    X( nuserdata )          \
    X( nsensordata )        \
    X( nbuffer )


// define symbols needed in MJMODEL_POINTERS (corresponding to number of columns)
#define MJMODEL_POINTERS_PREAMBLE( m )      \
    int nuser_body = m->nuser_body;         \
    int nuser_jnt = m->nuser_jnt;           \
    int nuser_geom = m->nuser_geom;         \
    int nuser_site = m->nuser_site;         \
    int nuser_cam = m->nuser_cam;           \
    int nuser_tendon = m->nuser_tendon;     \
    int nuser_actuator = m->nuser_actuator; \
    int nuser_sensor = m->nuser_sensor;     \
    int nq = m->nq;                         \
    int nv = m->nv;                         \
    int na = m->na;                         \
    int nmocap = m->nmocap;


// macro for annotating that an array size in an X macro is a member of mjModel
// by default this macro does nothing, but users can redefine it as necessary
#define MJ_M(n) n


// pointer fields of mjModel
#define MJMODEL_POINTERS                                                     \
    X( mjtNum,  qpos0,                 nq,            1                    ) \
    X( mjtNum,  qpos_spring,           nq,            1                    ) \
    X( int,     body_parentid,         nbody,         1                    ) \
    X( int,     body_rootid,           nbody,         1                    ) \
    X( int,     body_weldid,           nbody,         1                    ) \
    X( int,     body_mocapid,          nbody,         1                    ) \
    X( int,     body_jntnum,           nbody,         1                    ) \
    X( int,     body_jntadr,           nbody,         1                    ) \
    X( int,     body_dofnum,           nbody,         1                    ) \
    X( int,     body_dofadr,           nbody,         1                    ) \
    X( int,     body_geomnum,          nbody,         1                    ) \
    X( int,     body_geomadr,          nbody,         1                    ) \
    X( mjtByte, body_simple,           nbody,         1                    ) \
    X( mjtByte, body_sameframe,        nbody,         1                    ) \
    X( mjtNum,  body_pos,              nbody,         3                    ) \
    X( mjtNum,  body_quat,             nbody,         4                    ) \
    X( mjtNum,  body_ipos,             nbody,         3                    ) \
    X( mjtNum,  body_iquat,            nbody,         4                    ) \
    X( mjtNum,  body_mass,             nbody,         1                    ) \
    X( mjtNum,  body_subtreemass,      nbody,         1                    ) \
    X( mjtNum,  body_inertia,          nbody,         3                    ) \
    X( mjtNum,  body_invweight0,       nbody,         2                    ) \
    X( mjtNum,  body_user,             nbody,         MJ_M(nuser_body)     ) \
    X( int,     jnt_type,              njnt,          1                    ) \
    X( int,     jnt_qposadr,           njnt,          1                    ) \
    X( int,     jnt_dofadr,            njnt,          1                    ) \
    X( int,     jnt_bodyid,            njnt,          1                    ) \
    X( int,     jnt_group,             njnt,          1                    ) \
    X( mjtByte, jnt_limited,           njnt,          1                    ) \
    X( mjtNum,  jnt_solref,            njnt,          mjNREF               ) \
    X( mjtNum,  jnt_solimp,            njnt,          mjNIMP               ) \
    X( mjtNum,  jnt_pos,               njnt,          3                    ) \
    X( mjtNum,  jnt_axis,              njnt,          3                    ) \
    X( mjtNum,  jnt_stiffness,         njnt,          1                    ) \
    X( mjtNum,  jnt_range,             njnt,          2                    ) \
    X( mjtNum,  jnt_margin,            njnt,          1                    ) \
    X( mjtNum,  jnt_user,              njnt,          MJ_M(nuser_jnt)      ) \
    X( int,     dof_bodyid,            nv,            1                    ) \
    X( int,     dof_jntid,             nv,            1                    ) \
    X( int,     dof_parentid,          nv,            1                    ) \
    X( int,     dof_Madr,              nv,            1                    ) \
    X( int,     dof_simplenum,         nv,            1                    ) \
    X( mjtNum,  dof_solref,            nv,            mjNREF               ) \
    X( mjtNum,  dof_solimp,            nv,            mjNIMP               ) \
    X( mjtNum,  dof_frictionloss,      nv,            1                    ) \
    X( mjtNum,  dof_armature,          nv,            1                    ) \
    X( mjtNum,  dof_damping,           nv,            1                    ) \
    X( mjtNum,  dof_invweight0,        nv,            1                    ) \
    X( mjtNum,  dof_M0,                nv,            1                    ) \
    X( int,     geom_type,             ngeom,         1                    ) \
    X( int,     geom_contype,          ngeom,         1                    ) \
    X( int,     geom_conaffinity,      ngeom,         1                    ) \
    X( int,     geom_condim,           ngeom,         1                    ) \
    X( int,     geom_bodyid,           ngeom,         1                    ) \
    X( int,     geom_dataid,           ngeom,         1                    ) \
    X( int,     geom_matid,            ngeom,         1                    ) \
    X( int,     geom_group,            ngeom,         1                    ) \
    X( int,     geom_priority,         ngeom,         1                    ) \
    X( mjtByte, geom_sameframe,        ngeom,         1                    ) \
    X( mjtNum,  geom_solmix,           ngeom,         1                    ) \
    X( mjtNum,  geom_solref,           ngeom,         mjNREF               ) \
    X( mjtNum,  geom_solimp,           ngeom,         mjNIMP               ) \
    X( mjtNum,  geom_size,             ngeom,         3                    ) \
    X( mjtNum,  geom_rbound,           ngeom,         1                    ) \
    X( mjtNum,  geom_pos,              ngeom,         3                    ) \
    X( mjtNum,  geom_quat,             ngeom,         4                    ) \
    X( mjtNum,  geom_friction,         ngeom,         3                    ) \
    X( mjtNum,  geom_margin,           ngeom,         1                    ) \
    X( mjtNum,  geom_gap,              ngeom,         1                    ) \
    X( mjtNum,  geom_fluid,            ngeom,         mjNFLUID             ) \
    X( mjtNum,  geom_user,             ngeom,         MJ_M(nuser_geom)     ) \
    X( float,   geom_rgba,             ngeom,         4                    ) \
    X( int,     site_type,             nsite,         1                    ) \
    X( int,     site_bodyid,           nsite,         1                    ) \
    X( int,     site_matid,            nsite,         1                    ) \
    X( int,     site_group,            nsite,         1                    ) \
    X( mjtByte, site_sameframe,        nsite,         1                    ) \
    X( mjtNum,  site_size,             nsite,         3                    ) \
    X( mjtNum,  site_pos,              nsite,         3                    ) \
    X( mjtNum,  site_quat,             nsite,         4                    ) \
    X( mjtNum,  site_user,             nsite,         MJ_M(nuser_site)     ) \
    X( float,   site_rgba,             nsite,         4                    ) \
    X( int,     cam_mode,              ncam,          1                    ) \
    X( int,     cam_bodyid,            ncam,          1                    ) \
    X( int,     cam_targetbodyid,      ncam,          1                    ) \
    X( mjtNum,  cam_pos,               ncam,          3                    ) \
    X( mjtNum,  cam_quat,              ncam,          4                    ) \
    X( mjtNum,  cam_poscom0,           ncam,          3                    ) \
    X( mjtNum,  cam_pos0,              ncam,          3                    ) \
    X( mjtNum,  cam_mat0,              ncam,          9                    ) \
    X( mjtNum,  cam_fovy,              ncam,          1                    ) \
    X( mjtNum,  cam_ipd,               ncam,          1                    ) \
    X( mjtNum,  cam_user,              ncam,          MJ_M(nuser_cam)      ) \
    X( int,     light_mode,            nlight,        1                    ) \
    X( int,     light_bodyid,          nlight,        1                    ) \
    X( int,     light_targetbodyid,    nlight,        1                    ) \
    X( mjtByte, light_directional,     nlight,        1                    ) \
    X( mjtByte, light_castshadow,      nlight,        1                    ) \
    X( mjtByte, light_active,          nlight,        1                    ) \
    X( mjtNum,  light_pos,             nlight,        3                    ) \
    X( mjtNum,  light_dir,             nlight,        3                    ) \
    X( mjtNum,  light_poscom0,         nlight,        3                    ) \
    X( mjtNum,  light_pos0,            nlight,        3                    ) \
    X( mjtNum,  light_dir0,            nlight,        3                    ) \
    X( float,   light_attenuation,     nlight,        3                    ) \
    X( float,   light_cutoff,          nlight,        1                    ) \
    X( float,   light_exponent,        nlight,        1                    ) \
    X( float,   light_ambient,         nlight,        3                    ) \
    X( float,   light_diffuse,         nlight,        3                    ) \
    X( float,   light_specular,        nlight,        3                    ) \
    X( int,     mesh_vertadr,          nmesh,         1                    ) \
    X( int,     mesh_vertnum,          nmesh,         1                    ) \
    X( int,     mesh_texcoordadr,      nmesh,         1                    ) \
    X( int,     mesh_faceadr,          nmesh,         1                    ) \
    X( int,     mesh_facenum,          nmesh,         1                    ) \
    X( int,     mesh_graphadr,         nmesh,         1                    ) \
    X( float,   mesh_vert,             nmeshvert,     3                    ) \
    X( float,   mesh_normal,           nmeshvert,     3                    ) \
    X( float,   mesh_texcoord,         nmeshtexvert,  2                    ) \
    X( int,     mesh_face,             nmeshface,     3                    ) \
    X( int,     mesh_graph,            nmeshgraph,    1                    ) \
    X( int,     skin_matid,            nskin,         1                    ) \
    X( float,   skin_rgba,             nskin,         4                    ) \
    X( float,   skin_inflate,          nskin,         1                    ) \
    X( int,     skin_vertadr,          nskin,         1                    ) \
    X( int,     skin_vertnum,          nskin,         1                    ) \
    X( int,     skin_texcoordadr,      nskin,         1                    ) \
    X( int,     skin_faceadr,          nskin,         1                    ) \
    X( int,     skin_facenum,          nskin,         1                    ) \
    X( int,     skin_boneadr,          nskin,         1                    ) \
    X( int,     skin_bonenum,          nskin,         1                    ) \
    X( float,   skin_vert,             nskinvert,     3                    ) \
    X( float,   skin_texcoord,         nskintexvert,  2                    ) \
    X( int,     skin_face,             nskinface,     3                    ) \
    X( int,     skin_bonevertadr,      nskinbone,     1                    ) \
    X( int,     skin_bonevertnum,      nskinbone,     1                    ) \
    X( float,   skin_bonebindpos,      nskinbone,     3                    ) \
    X( float,   skin_bonebindquat,     nskinbone,     4                    ) \
    X( int,     skin_bonebodyid,       nskinbone,     1                    ) \
    X( int,     skin_bonevertid,       nskinbonevert, 1                    ) \
    X( float,   skin_bonevertweight,   nskinbonevert, 1                    ) \
    X( mjtNum,  hfield_size,           nhfield,       4                    ) \
    X( int,     hfield_nrow,           nhfield,       1                    ) \
    X( int,     hfield_ncol,           nhfield,       1                    ) \
    X( int,     hfield_adr,            nhfield,       1                    ) \
    X( float,   hfield_data,           nhfielddata,   1                    ) \
    X( int,     tex_type,              ntex,          1                    ) \
    X( int,     tex_height,            ntex,          1                    ) \
    X( int,     tex_width,             ntex,          1                    ) \
    X( int,     tex_adr,               ntex,          1                    ) \
    X( mjtByte, tex_rgb,               ntexdata,      1                    ) \
    X( int,     mat_texid,             nmat,          1                    ) \
    X( mjtByte, mat_texuniform,        nmat,          1                    ) \
    X( float,   mat_texrepeat,         nmat,          2                    ) \
    X( float,   mat_emission,          nmat,          1                    ) \
    X( float,   mat_specular,          nmat,          1                    ) \
    X( float,   mat_shininess,         nmat,          1                    ) \
    X( float,   mat_reflectance,       nmat,          1                    ) \
    X( float,   mat_rgba,              nmat,          4                    ) \
    X( int,     pair_dim,              npair,         1                    ) \
    X( int,     pair_geom1,            npair,         1                    ) \
    X( int,     pair_geom2,            npair,         1                    ) \
    X( int,     pair_signature,        npair,         1                    ) \
    X( mjtNum,  pair_solref,           npair,         mjNREF               ) \
    X( mjtNum,  pair_solimp,           npair,         mjNIMP               ) \
    X( mjtNum,  pair_margin,           npair,         1                    ) \
    X( mjtNum,  pair_gap,              npair,         1                    ) \
    X( mjtNum,  pair_friction,         npair,         5                    ) \
    X( int,     exclude_signature,     nexclude,      1                    ) \
    X( int,     eq_type,               neq,           1                    ) \
    X( int,     eq_obj1id,             neq,           1                    ) \
    X( int,     eq_obj2id,             neq,           1                    ) \
    X( mjtByte, eq_active,             neq,           1                    ) \
    X( mjtNum,  eq_solref,             neq,           mjNREF               ) \
    X( mjtNum,  eq_solimp,             neq,           mjNIMP               ) \
    X( mjtNum,  eq_data,               neq,           mjNEQDATA            ) \
    X( int,     tendon_adr,            ntendon,       1                    ) \
    X( int,     tendon_num,            ntendon,       1                    ) \
    X( int,     tendon_matid,          ntendon,       1                    ) \
    X( int,     tendon_group,          ntendon,       1                    ) \
    X( mjtByte, tendon_limited,        ntendon,       1                    ) \
    X( mjtNum,  tendon_width,          ntendon,       1                    ) \
    X( mjtNum,  tendon_solref_lim,     ntendon,       mjNREF               ) \
    X( mjtNum,  tendon_solimp_lim,     ntendon,       mjNIMP               ) \
    X( mjtNum,  tendon_solref_fri,     ntendon,       mjNREF               ) \
    X( mjtNum,  tendon_solimp_fri,     ntendon,       mjNIMP               ) \
    X( mjtNum,  tendon_range,          ntendon,       2                    ) \
    X( mjtNum,  tendon_margin,         ntendon,       1                    ) \
    X( mjtNum,  tendon_stiffness,      ntendon,       1                    ) \
    X( mjtNum,  tendon_damping,        ntendon,       1                    ) \
    X( mjtNum,  tendon_frictionloss,   ntendon,       1                    ) \
    X( mjtNum,  tendon_lengthspring,   ntendon,       1                    ) \
    X( mjtNum,  tendon_length0,        ntendon,       1                    ) \
    X( mjtNum,  tendon_invweight0,     ntendon,       1                    ) \
    X( mjtNum,  tendon_user,           ntendon,       MJ_M(nuser_tendon)   ) \
    X( float,   tendon_rgba,           ntendon,       4                    ) \
    X( int,     wrap_type,             nwrap,         1                    ) \
    X( int,     wrap_objid,            nwrap,         1                    ) \
    X( mjtNum,  wrap_prm,              nwrap,         1                    ) \
    X( int,     actuator_trntype,      nu,            1                    ) \
    X( int,     actuator_dyntype,      nu,            1                    ) \
    X( int,     actuator_gaintype,     nu,            1                    ) \
    X( int,     actuator_biastype,     nu,            1                    ) \
    X( int,     actuator_trnid,        nu,            2                    ) \
    X( int,     actuator_group,        nu,            1                    ) \
    X( mjtByte, actuator_ctrllimited,  nu,            1                    ) \
    X( mjtByte, actuator_forcelimited, nu,            1                    ) \
    X( mjtNum,  actuator_dynprm,       nu,            mjNDYN               ) \
    X( mjtNum,  actuator_gainprm,      nu,            mjNGAIN              ) \
    X( mjtNum,  actuator_biasprm,      nu,            mjNBIAS              ) \
    X( mjtNum,  actuator_ctrlrange,    nu,            2                    ) \
    X( mjtNum,  actuator_forcerange,   nu,            2                    ) \
    X( mjtNum,  actuator_gear,         nu,            6                    ) \
    X( mjtNum,  actuator_cranklength,  nu,            1                    ) \
    X( mjtNum,  actuator_acc0,         nu,            1                    ) \
    X( mjtNum,  actuator_length0,      nu,            1                    ) \
    X( mjtNum,  actuator_lengthrange,  nu,            2                    ) \
    X( mjtNum,  actuator_user,         nu,            MJ_M(nuser_actuator) ) \
    X( int,     sensor_type,           nsensor,       1                    ) \
    X( int,     sensor_datatype,       nsensor,       1                    ) \
    X( int,     sensor_needstage,      nsensor,       1                    ) \
    X( int,     sensor_objtype,        nsensor,       1                    ) \
    X( int,     sensor_objid,          nsensor,       1                    ) \
    X( int,     sensor_reftype,        nsensor,       1                    ) \
    X( int,     sensor_refid,          nsensor,       1                    ) \
    X( int,     sensor_dim,            nsensor,       1                    ) \
    X( int,     sensor_adr,            nsensor,       1                    ) \
    X( mjtNum,  sensor_cutoff,         nsensor,       1                    ) \
    X( mjtNum,  sensor_noise,          nsensor,       1                    ) \
    X( mjtNum,  sensor_user,           nsensor,       MJ_M(nuser_sensor)   ) \
    X( int,     numeric_adr,           nnumeric,      1                    ) \
    X( int,     numeric_size,          nnumeric,      1                    ) \
    X( mjtNum,  numeric_data,          nnumericdata,  1                    ) \
    X( int,     text_adr,              ntext,         1                    ) \
    X( int,     text_size,             ntext,         1                    ) \
    X( char,    text_data,             ntextdata,     1                    ) \
    X( int,     tuple_adr,             ntuple,        1                    ) \
    X( int,     tuple_size,            ntuple,        1                    ) \
    X( int,     tuple_objtype,         ntupledata,    1                    ) \
    X( int,     tuple_objid,           ntupledata,    1                    ) \
    X( mjtNum,  tuple_objprm,          ntupledata,    1                    ) \
    X( mjtNum,  key_time,              nkey,          1                    ) \
    X( mjtNum,  key_qpos,              nkey,          MJ_M(nq)             ) \
    X( mjtNum,  key_qvel,              nkey,          MJ_M(nv)             ) \
    X( mjtNum,  key_act,               nkey,          MJ_M(na)             ) \
    X( mjtNum,  key_mpos,              nkey,          MJ_M(nmocap)*3       ) \
    X( mjtNum,  key_mquat,             nkey,          MJ_M(nmocap)*4       ) \
    X( int,     name_bodyadr,          nbody,         1                    ) \
    X( int,     name_jntadr,           njnt,          1                    ) \
    X( int,     name_geomadr,          ngeom,         1                    ) \
    X( int,     name_siteadr,          nsite,         1                    ) \
    X( int,     name_camadr,           ncam,          1                    ) \
    X( int,     name_lightadr,         nlight,        1                    ) \
    X( int,     name_meshadr,          nmesh,         1                    ) \
    X( int,     name_skinadr,          nskin,         1                    ) \
    X( int,     name_hfieldadr,        nhfield,       1                    ) \
    X( int,     name_texadr,           ntex,          1                    ) \
    X( int,     name_matadr,           nmat,          1                    ) \
    X( int,     name_pairadr,          npair,         1                    ) \
    X( int,     name_excludeadr,       nexclude,      1                    ) \
    X( int,     name_eqadr,            neq,           1                    ) \
    X( int,     name_tendonadr,        ntendon,       1                    ) \
    X( int,     name_actuatoradr,      nu,            1                    ) \
    X( int,     name_sensoradr,        nsensor,       1                    ) \
    X( int,     name_numericadr,       nnumeric,      1                    ) \
    X( int,     name_textadr,          ntext,         1                    ) \
    X( int,     name_tupleadr,         ntuple,        1                    ) \
    X( int,     name_keyadr,           nkey,          1                    ) \
    X( char,    names,                 nnames,        1                    )


//-------------------------------- mjData ----------------------------------------------------------

// define symbols needed in MJDATA_POINTERS (corresponding to number of columns)
#define MJDATA_POINTERS_PREAMBLE( m ) \
    int nv = m->nv;                   \
    int njmax = m->njmax;


// pointer fields of mjData
#define MJDATA_POINTERS                                         \
    X( mjtNum,    qpos,              nq,          1           ) \
    X( mjtNum,    qvel,              nv,          1           ) \
    X( mjtNum,    act,               na,          1           ) \
    X( mjtNum,    qacc_warmstart,    nv,          1           ) \
    X( mjtNum,    ctrl,              nu,          1           ) \
    X( mjtNum,    qfrc_applied,      nv,          1           ) \
    X( mjtNum,    xfrc_applied,      nbody,       6           ) \
    X( mjtNum,    mocap_pos,         nmocap,      3           ) \
    X( mjtNum,    mocap_quat,        nmocap,      4           ) \
    X( mjtNum,    qacc,              nv,          1           ) \
    X( mjtNum,    act_dot,           na,          1           ) \
    X( mjtNum,    userdata,          nuserdata,   1           ) \
    X( mjtNum,    sensordata,        nsensordata, 1           ) \
    X( mjtNum,    xpos,              nbody,       3           ) \
    X( mjtNum,    xquat,             nbody,       4           ) \
    X( mjtNum,    xmat,              nbody,       9           ) \
    X( mjtNum,    xipos,             nbody,       3           ) \
    X( mjtNum,    ximat,             nbody,       9           ) \
    X( mjtNum,    xanchor,           njnt,        3           ) \
    X( mjtNum,    xaxis,             njnt,        3           ) \
    X( mjtNum,    geom_xpos,         ngeom,       3           ) \
    X( mjtNum,    geom_xmat,         ngeom,       9           ) \
    X( mjtNum,    site_xpos,         nsite,       3           ) \
    X( mjtNum,    site_xmat,         nsite,       9           ) \
    X( mjtNum,    cam_xpos,          ncam,        3           ) \
    X( mjtNum,    cam_xmat,          ncam,        9           ) \
    X( mjtNum,    light_xpos,        nlight,      3           ) \
    X( mjtNum,    light_xdir,        nlight,      3           ) \
    X( mjtNum,    subtree_com,       nbody,       3           ) \
    X( mjtNum,    cdof,              nv,          6           ) \
    X( mjtNum,    cinert,            nbody,       10          ) \
    X( int,       ten_wrapadr,       ntendon,     1           ) \
    X( int,       ten_wrapnum,       ntendon,     1           ) \
    X( int,       ten_J_rownnz,      ntendon,     1           ) \
    X( int,       ten_J_rowadr,      ntendon,     1           ) \
    X( int,       ten_J_colind,      ntendon,     MJ_M(nv)    ) \
    X( mjtNum,    ten_length,        ntendon,     1           ) \
    X( mjtNum,    ten_J,             ntendon,     MJ_M(nv)    ) \
    X( int,       wrap_obj,          nwrap,       2           ) \
    X( mjtNum,    wrap_xpos,         nwrap,       6           ) \
    X( mjtNum,    actuator_length,   nu,          1           ) \
    X( mjtNum,    actuator_moment,   nu,          MJ_M(nv)    ) \
    X( mjtNum,    crb,               nbody,       10          ) \
    X( mjtNum,    qM,                nM,          1           ) \
    X( mjtNum,    qLD,               nM,          1           ) \
    X( mjtNum,    qLDiagInv,         nv,          1           ) \
    X( mjtNum,    qLDiagSqrtInv,     nv,          1           ) \
    X( mjContact, contact,           nconmax,     1           ) \
    X( int,       efc_type,          njmax,       1           ) \
    X( int,       efc_id,            njmax,       1           ) \
    X( int,       efc_J_rownnz,      njmax,       1           ) \
    X( int,       efc_J_rowadr,      njmax,       1           ) \
    X( int,       efc_J_rowsuper,    njmax,       1           ) \
    X( int,       efc_J_colind,      njmax,       MJ_M(nv)    ) \
    X( int,       efc_JT_rownnz,     nv,          1           ) \
    X( int,       efc_JT_rowadr,     nv,          1           ) \
    X( int,       efc_JT_rowsuper,   nv,          1           ) \
    X( int,       efc_JT_colind,     nv,          MJ_M(njmax) ) \
    X( mjtNum,    efc_J,             njmax,       MJ_M(nv)    ) \
    X( mjtNum,    efc_JT,            nv,          MJ_M(njmax) ) \
    X( mjtNum,    efc_pos,           njmax,       1           ) \
    X( mjtNum,    efc_margin,        njmax,       1           ) \
    X( mjtNum,    efc_frictionloss,  njmax,       1           ) \
    X( mjtNum,    efc_diagApprox,    njmax,       1           ) \
    X( mjtNum,    efc_KBIP,          njmax,       4           ) \
    X( mjtNum,    efc_D,             njmax,       1           ) \
    X( mjtNum,    efc_R,             njmax,       1           ) \
    X( int,       efc_AR_rownnz,     njmax,       1           ) \
    X( int,       efc_AR_rowadr,     njmax,       1           ) \
    X( int,       efc_AR_colind,     njmax,       MJ_M(njmax) ) \
    X( mjtNum,    efc_AR,            njmax,       MJ_M(njmax) ) \
    X( mjtNum,    ten_velocity,      ntendon,     1           ) \
    X( mjtNum,    actuator_velocity, nu,          1           ) \
    X( mjtNum,    cvel,              nbody,       6           ) \
    X( mjtNum,    cdof_dot,          nv,          6           ) \
    X( mjtNum,    qfrc_bias,         nv,          1           ) \
    X( mjtNum,    qfrc_passive,      nv,          1           ) \
    X( mjtNum,    efc_vel,           njmax,       1           ) \
    X( mjtNum,    efc_aref,          njmax,       1           ) \
    X( mjtNum,    subtree_linvel,    nbody,       3           ) \
    X( mjtNum,    subtree_angmom,    nbody,       3           ) \
    X( mjtNum,    actuator_force,    nu,          1           ) \
    X( mjtNum,    qfrc_actuator,     nv,          1           ) \
    X( mjtNum,    qfrc_unc,          nv,          1           ) \
    X( mjtNum,    qacc_unc,          nv,          1           ) \
    X( mjtNum,    efc_b,             njmax,       1           ) \
    X( mjtNum,    efc_force,         njmax,       1           ) \
    X( int,       efc_state,         njmax,       1           ) \
    X( mjtNum,    qfrc_constraint,   nv,          1           ) \
    X( mjtNum,    qfrc_inverse,      nv,          1           ) \
    X( mjtNum,    cacc,              nbody,       6           ) \
    X( mjtNum,    cfrc_int,          nbody,       6           ) \
    X( mjtNum,    cfrc_ext,          nbody,       6           )


// scalar fields of mjData
#define MJDATA_SCALAR            \
    X( int,       nstack       ) \
    X( int,       nbuffer      ) \
    X( int,       pstack       ) \
    X( int,       maxuse_stack ) \
    X( int,       maxuse_con   ) \
    X( int,       maxuse_efc   ) \
    X( int,       solver_iter  ) \
    X( int,       solver_nnz   ) \
    X( int,       ne           ) \
    X( int,       nf           ) \
    X( int,       nefc         ) \
    X( int,       ncon         ) \
    X( mjtNum,    time         )


// vector fields of mjData
#define MJDATA_VECTOR                                   \
    X( mjWarningStat,  warning,        mjNWARNING,  1 ) \
    X( mjTimerStat,    timer,          mjNTIMER,    1 ) \
    X( mjSolverStat,   solver,         mjNSOLVER,   1 ) \
    X( mjtNum,         solver_fwdinv,  2,           1 ) \
    X( mjtNum,         energy,         2,           1 )


#endif  // MUJOCO_MJXMACRO_H_
