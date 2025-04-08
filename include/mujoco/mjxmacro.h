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
    X( mjtNum,  timestep         )  \
    X( mjtNum,  apirate          )  \
    X( mjtNum,  impratio         )  \
    X( mjtNum,  tolerance        )  \
    X( mjtNum,  ls_tolerance     )  \
    X( mjtNum,  noslip_tolerance )  \
    X( mjtNum,  ccd_tolerance    )  \
    X( mjtNum,  density          )  \
    X( mjtNum,  viscosity        )  \
    X( mjtNum,  o_margin         )  \


#define MJOPTION_INTS               \
    X( int,     integrator        ) \
    X( int,     cone              ) \
    X( int,     jacobian          ) \
    X( int,     solver            ) \
    X( int,     iterations        ) \
    X( int,     ls_iterations     ) \
    X( int,     noslip_iterations ) \
    X( int,     ccd_iterations    ) \
    X( int,     disableflags      ) \
    X( int,     enableflags       ) \
    X( int,     disableactuator   ) \
    X( int,     sdf_initpoints    ) \
    X( int,     sdf_iterations    )


#define MJOPTION_SCALARS            \
    MJOPTION_FLOATS                 \
    MJOPTION_INTS


// vector fields of mjOption
#define MJOPTION_VECTORS            \
    X( gravity,         3       )   \
    X( wind,            3       )   \
    X( magnetic,        3       )   \
    X( o_solref,        mjNREF  )   \
    X( o_solimp,        mjNIMP  )   \
    X( o_friction,      5       )


//-------------------------------- mjModel ---------------------------------------------------------

// int fields of mjModel
#define MJMODEL_INTS           \
    X   ( nq )                 \
    XMJV( nv )                 \
    XMJV( nu )                 \
    XMJV( na )                 \
    XMJV( nbody )              \
    XMJV( nbvh )               \
    XMJV( nbvhstatic )         \
    X   ( nbvhdynamic )        \
    XMJV( njnt )               \
    XMJV( ngeom )              \
    XMJV( nsite )              \
    XMJV( ncam )               \
    XMJV( nlight )             \
    XMJV( nflex )              \
    X   ( nflexnode )          \
    XMJV( nflexvert )          \
    X   ( nflexedge )          \
    X   ( nflexelem )          \
    X   ( nflexelemdata )      \
    X   ( nflexelemedge )      \
    X   ( nflexshelldata )     \
    X   ( nflexevpair )        \
    XMJV( nflextexcoord )      \
    XMJV( nmesh )              \
    X   ( nmeshvert )          \
    X   ( nmeshnormal )        \
    X   ( nmeshtexcoord )      \
    X   ( nmeshface )          \
    X   ( nmeshgraph )         \
    X   ( nmeshpoly )          \
    X   ( nmeshpolyvert )      \
    X   ( nmeshpolymap )       \
    XMJV( nskin )              \
    XMJV( nskinvert )          \
    X   ( nskintexvert )       \
    XMJV( nskinface )          \
    XMJV( nskinbone )          \
    XMJV( nskinbonevert )      \
    X   ( nhfield )            \
    X   ( nhfielddata )        \
    X   ( ntex )               \
    X   ( ntexdata )           \
    XMJV( nmat )               \
    X   ( npair )              \
    X   ( nexclude )           \
    XMJV( neq )                \
    XMJV( ntendon )            \
    XMJV( nwrap )              \
    XMJV( nsensor )            \
    X   ( nnumeric )           \
    X   ( nnumericdata )       \
    X   ( ntext )              \
    X   ( ntextdata )          \
    X   ( ntuple )             \
    X   ( ntupledata )         \
    X   ( nkey )               \
    X   ( nmocap )             \
    X   ( nplugin )            \
    X   ( npluginattr )        \
    X   ( nuser_body )         \
    X   ( nuser_jnt )          \
    X   ( nuser_geom )         \
    X   ( nuser_site )         \
    X   ( nuser_cam )          \
    X   ( nuser_tendon )       \
    X   ( nuser_actuator )     \
    X   ( nuser_sensor )       \
    XMJV( nnames )             \
    XMJV( npaths )             \
    X   ( nnames_map )         \
    X   ( nM )                 \
    X   ( nB )                 \
    X   ( nC )                 \
    X   ( nD )                 \
    X   ( nJmom )              \
    XMJV( ntree )              \
    X   ( ngravcomp )          \
    X   ( nemax )              \
    X   ( njmax )              \
    X   ( nconmax )            \
    X   ( nuserdata )          \
    XMJV( nsensordata )        \
    X   ( npluginstate )       \
    X   ( narena )             \
    X   ( nbuffer )

    /* nbuffer needs to be the final field */


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
    int nu = m->nu;                         \
    int nmocap = m->nmocap;

// macro for annotating that an array size in an X macro is a member of mjModel
// by default this macro does nothing, but users can redefine it as necessary
#define MJ_M(n) n


// pointer fields of mjModel
// XMJV means that the field is required to construct mjvScene
// (by default we define XMJV to be the same as X)
#define MJMODEL_POINTERS                                                        \
    X   ( mjtNum,  qpos0,                 nq,            1                    ) \
    X   ( mjtNum,  qpos_spring,           nq,            1                    ) \
    XMJV( int,     body_parentid,         nbody,         1                    ) \
    XMJV( int,     body_rootid,           nbody,         1                    ) \
    XMJV( int,     body_weldid,           nbody,         1                    ) \
    XMJV( int,     body_mocapid,          nbody,         1                    ) \
    XMJV( int,     body_jntnum,           nbody,         1                    ) \
    XMJV( int,     body_jntadr,           nbody,         1                    ) \
    XMJV( int,     body_dofnum,           nbody,         1                    ) \
    XMJV( int,     body_dofadr,           nbody,         1                    ) \
    X   ( int,     body_treeid,           nbody,         1                    ) \
    XMJV( int,     body_geomnum,          nbody,         1                    ) \
    XMJV( int,     body_geomadr,          nbody,         1                    ) \
    X   ( mjtByte, body_simple,           nbody,         1                    ) \
    X   ( mjtByte, body_sameframe,        nbody,         1                    ) \
    X   ( mjtNum,  body_pos,              nbody,         3                    ) \
    X   ( mjtNum,  body_quat,             nbody,         4                    ) \
    X   ( mjtNum,  body_ipos,             nbody,         3                    ) \
    XMJV( mjtNum,  body_iquat,            nbody,         4                    ) \
    XMJV( mjtNum,  body_mass,             nbody,         1                    ) \
    X   ( mjtNum,  body_subtreemass,      nbody,         1                    ) \
    XMJV( mjtNum,  body_inertia,          nbody,         3                    ) \
    X   ( mjtNum,  body_invweight0,       nbody,         2                    ) \
    X   ( mjtNum,  body_gravcomp,         nbody,         1                    ) \
    X   ( mjtNum,  body_margin,           nbody,         1                    ) \
    X   ( mjtNum,  body_user,             nbody,         MJ_M(nuser_body)     ) \
    X   ( int,     body_plugin,           nbody,         1                    ) \
    X   ( int,     body_contype,          nbody,         1                    ) \
    X   ( int,     body_conaffinity,      nbody,         1                    ) \
    XMJV( int,     body_bvhadr,           nbody,         1                    ) \
    XMJV( int,     body_bvhnum,           nbody,         1                    ) \
    XMJV( int,     bvh_depth,             nbvh,          1                    ) \
    XMJV( int,     bvh_child,             nbvh,          2                    ) \
    XMJV( int,     bvh_nodeid,            nbvh,          1                    ) \
    XMJV( mjtNum,  bvh_aabb,              nbvhstatic,    6                    ) \
    XMJV( int,     jnt_type,              njnt,          1                    ) \
    X   ( int,     jnt_qposadr,           njnt,          1                    ) \
    X   ( int,     jnt_dofadr,            njnt,          1                    ) \
    XMJV( int,     jnt_bodyid,            njnt,          1                    ) \
    XMJV( int,     jnt_group,             njnt,          1                    ) \
    X   ( mjtByte, jnt_limited,           njnt,          1                    ) \
    X   ( mjtByte, jnt_actfrclimited,     njnt,          1                    ) \
    X   ( mjtByte, jnt_actgravcomp,       njnt,          1                    ) \
    X   ( mjtNum,  jnt_solref,            njnt,          mjNREF               ) \
    X   ( mjtNum,  jnt_solimp,            njnt,          mjNIMP               ) \
    X   ( mjtNum,  jnt_pos,               njnt,          3                    ) \
    X   ( mjtNum,  jnt_axis,              njnt,          3                    ) \
    X   ( mjtNum,  jnt_stiffness,         njnt,          1                    ) \
    X   ( mjtNum,  jnt_range,             njnt,          2                    ) \
    X   ( mjtNum,  jnt_actfrcrange,       njnt,          2                    ) \
    X   ( mjtNum,  jnt_margin,            njnt,          1                    ) \
    X   ( mjtNum,  jnt_user,              njnt,          MJ_M(nuser_jnt)      ) \
    X   ( int,     dof_bodyid,            nv,            1                    ) \
    X   ( int,     dof_jntid,             nv,            1                    ) \
    X   ( int,     dof_parentid,          nv,            1                    ) \
    X   ( int,     dof_treeid,            nv,            1                    ) \
    X   ( int,     dof_Madr,              nv,            1                    ) \
    X   ( int,     dof_simplenum,         nv,            1                    ) \
    X   ( mjtNum,  dof_solref,            nv,            mjNREF               ) \
    X   ( mjtNum,  dof_solimp,            nv,            mjNIMP               ) \
    X   ( mjtNum,  dof_frictionloss,      nv,            1                    ) \
    X   ( mjtNum,  dof_armature,          nv,            1                    ) \
    X   ( mjtNum,  dof_damping,           nv,            1                    ) \
    X   ( mjtNum,  dof_invweight0,        nv,            1                    ) \
    X   ( mjtNum,  dof_M0,                nv,            1                    ) \
    XMJV( int,     geom_type,             ngeom,         1                    ) \
    XMJV( int,     geom_contype,          ngeom,         1                    ) \
    XMJV( int,     geom_conaffinity,      ngeom,         1                    ) \
    X   ( int,     geom_condim,           ngeom,         1                    ) \
    XMJV( int,     geom_bodyid,           ngeom,         1                    ) \
    XMJV( int,     geom_dataid,           ngeom,         1                    ) \
    XMJV( int,     geom_matid,            ngeom,         1                    ) \
    XMJV( int,     geom_group,            ngeom,         1                    ) \
    X   ( int,     geom_priority,         ngeom,         1                    ) \
    X   ( int,     geom_plugin,           ngeom,         1                    ) \
    X   ( mjtByte, geom_sameframe,        ngeom,         1                    ) \
    X   ( mjtNum,  geom_solmix,           ngeom,         1                    ) \
    X   ( mjtNum,  geom_solref,           ngeom,         mjNREF               ) \
    X   ( mjtNum,  geom_solimp,           ngeom,         mjNIMP               ) \
    XMJV( mjtNum,  geom_size,             ngeom,         3                    ) \
    XMJV( mjtNum,  geom_aabb,             ngeom,         6                    ) \
    XMJV( mjtNum,  geom_rbound,           ngeom,         1                    ) \
    X   ( mjtNum,  geom_pos,              ngeom,         3                    ) \
    X   ( mjtNum,  geom_quat,             ngeom,         4                    ) \
    X   ( mjtNum,  geom_friction,         ngeom,         3                    ) \
    X   ( mjtNum,  geom_margin,           ngeom,         1                    ) \
    X   ( mjtNum,  geom_gap,              ngeom,         1                    ) \
    X   ( mjtNum,  geom_fluid,            ngeom,         mjNFLUID             ) \
    X   ( mjtNum,  geom_user,             ngeom,         MJ_M(nuser_geom)     ) \
    XMJV( float,   geom_rgba,             ngeom,         4                    ) \
    XMJV( int,     site_type,             nsite,         1                    ) \
    XMJV( int,     site_bodyid,           nsite,         1                    ) \
    XMJV( int,     site_matid,            nsite,         1                    ) \
    XMJV( int,     site_group,            nsite,         1                    ) \
    X   ( mjtByte, site_sameframe,        nsite,         1                    ) \
    XMJV( mjtNum,  site_size,             nsite,         3                    ) \
    X   ( mjtNum,  site_pos,              nsite,         3                    ) \
    X   ( mjtNum,  site_quat,             nsite,         4                    ) \
    X   ( mjtNum,  site_user,             nsite,         MJ_M(nuser_site)     ) \
    XMJV( float,   site_rgba,             nsite,         4                    ) \
    X   ( int,     cam_mode,              ncam,          1                    ) \
    X   ( int,     cam_bodyid,            ncam,          1                    ) \
    X   ( int,     cam_targetbodyid,      ncam,          1                    ) \
    X   ( mjtNum,  cam_pos,               ncam,          3                    ) \
    X   ( mjtNum,  cam_quat,              ncam,          4                    ) \
    X   ( mjtNum,  cam_poscom0,           ncam,          3                    ) \
    X   ( mjtNum,  cam_pos0,              ncam,          3                    ) \
    X   ( mjtNum,  cam_mat0,              ncam,          9                    ) \
    XMJV( int,     cam_orthographic,      ncam,          1                    ) \
    XMJV( mjtNum,  cam_fovy,              ncam,          1                    ) \
    XMJV( mjtNum,  cam_ipd,               ncam,          1                    ) \
    XMJV( int,     cam_resolution,        ncam,          2                    ) \
    XMJV( float,   cam_sensorsize,        ncam,          2                    ) \
    XMJV( float,   cam_intrinsic,         ncam,          4                    ) \
    X   ( mjtNum,  cam_user,              ncam,          MJ_M(nuser_cam)      ) \
    X   ( int,     light_mode,            nlight,        1                    ) \
    X   ( int,     light_bodyid,          nlight,        1                    ) \
    X   ( int,     light_targetbodyid,    nlight,        1                    ) \
    XMJV( mjtByte, light_directional,     nlight,        1                    ) \
    XMJV( mjtByte, light_castshadow,      nlight,        1                    ) \
    XMJV( float,   light_bulbradius,      nlight,        1                    ) \
    XMJV( mjtByte, light_active,          nlight,        1                    ) \
    X   ( mjtNum,  light_pos,             nlight,        3                    ) \
    X   ( mjtNum,  light_dir,             nlight,        3                    ) \
    X   ( mjtNum,  light_poscom0,         nlight,        3                    ) \
    X   ( mjtNum,  light_pos0,            nlight,        3                    ) \
    X   ( mjtNum,  light_dir0,            nlight,        3                    ) \
    XMJV( float,   light_attenuation,     nlight,        3                    ) \
    XMJV( float,   light_cutoff,          nlight,        1                    ) \
    XMJV( float,   light_exponent,        nlight,        1                    ) \
    XMJV( float,   light_ambient,         nlight,        3                    ) \
    XMJV( float,   light_diffuse,         nlight,        3                    ) \
    XMJV( float,   light_specular,        nlight,        3                    ) \
    X   ( int,     flex_contype,          nflex,         1                    ) \
    X   ( int,     flex_conaffinity,      nflex,         1                    ) \
    X   ( int,     flex_condim,           nflex,         1                    ) \
    X   ( int,     flex_priority,         nflex,         1                    ) \
    X   ( mjtNum,  flex_solmix,           nflex,         1                    ) \
    X   ( mjtNum,  flex_solref,           nflex,         mjNREF               ) \
    X   ( mjtNum,  flex_solimp,           nflex,         mjNIMP               ) \
    X   ( mjtNum,  flex_friction,         nflex,         3                    ) \
    X   ( mjtNum,  flex_margin,           nflex,         1                    ) \
    X   ( mjtNum,  flex_gap,              nflex,         1                    ) \
    X   ( mjtByte, flex_internal,         nflex,         1                    ) \
    X   ( int,     flex_selfcollide,      nflex,         1                    ) \
    X   ( int,     flex_activelayers,     nflex,         1                    ) \
    XMJV( int,     flex_dim,              nflex,         1                    ) \
    XMJV( int,     flex_matid,            nflex,         1                    ) \
    XMJV( int,     flex_group,            nflex,         1                    ) \
    XMJV( int,     flex_interp,           nflex,         1                    ) \
    XMJV( int,     flex_nodeadr,          nflex,         1                    ) \
    XMJV( int,     flex_nodenum,          nflex,         1                    ) \
    XMJV( int,     flex_vertadr,          nflex,         1                    ) \
    XMJV( int,     flex_vertnum,          nflex,         1                    ) \
    X   ( int,     flex_edgeadr,          nflex,         1                    ) \
    X   ( int,     flex_edgenum,          nflex,         1                    ) \
    XMJV( int,     flex_elemadr,          nflex,         1                    ) \
    XMJV( int,     flex_elemnum,          nflex,         1                    ) \
    XMJV( int,     flex_elemdataadr,      nflex,         1                    ) \
    X   ( int,     flex_elemedgeadr,      nflex,         1                    ) \
    XMJV( int,     flex_shellnum,         nflex,         1                    ) \
    XMJV( int,     flex_shelldataadr,     nflex,         1                    ) \
    X   ( int,     flex_evpairadr,        nflex,         1                    ) \
    X   ( int,     flex_evpairnum,        nflex,         1                    ) \
    XMJV( int,     flex_texcoordadr,      nflex,         1                    ) \
    XMJV( int,     flex_nodebodyid,       nflexnode,     1                    ) \
    X   ( int,     flex_vertbodyid,       nflexvert,     1                    ) \
    X   ( int,     flex_edge,             nflexedge,     2                    ) \
    XMJV( int,     flex_elem,             nflexelemdata, 1                    ) \
    XMJV( int,     flex_elemtexcoord,     nflexelemdata, 1                    ) \
    X   ( int,     flex_elemedge,         nflexelemedge, 1                    ) \
    XMJV( int,     flex_elemlayer,        nflexelem,     1                    ) \
    XMJV( int,     flex_shell,            nflexshelldata,1                    ) \
    X   ( int,     flex_evpair,           nflexevpair,   2                    ) \
    X   ( mjtNum,  flex_vert,             nflexvert,     3                    ) \
    X   ( mjtNum,  flex_vert0,            nflexvert,     3                    ) \
    XMJV( mjtNum,  flex_node,             nflexnode,     3                    ) \
    X   ( mjtNum,  flex_node0,            nflexnode,     3                    ) \
    X   ( mjtNum,  flexedge_length0,      nflexedge,     1                    ) \
    X   ( mjtNum,  flexedge_invweight0,   nflexedge,     1                    ) \
    XMJV( mjtNum,  flex_radius,           nflex,         1                    ) \
    X   ( mjtNum,  flex_stiffness,        nflexelem,     21                   ) \
    X   ( mjtNum,  flex_damping,          nflex,         1                    ) \
    X   ( mjtNum,  flex_edgestiffness,    nflex,         1                    ) \
    X   ( mjtNum,  flex_edgedamping,      nflex,         1                    ) \
    X   ( mjtByte, flex_edgeequality,     nflex,         1                    ) \
    X   ( mjtByte, flex_rigid,            nflex,         1                    ) \
    X   ( mjtByte, flexedge_rigid,        nflexedge,     1                    ) \
    XMJV( mjtByte, flex_centered,         nflex,         1                    ) \
    XMJV( mjtByte, flex_flatskin,         nflex,         1                    ) \
    XMJV( int,     flex_bvhadr,           nflex,         1                    ) \
    XMJV( int,     flex_bvhnum,           nflex,         1                    ) \
    XMJV( float,   flex_rgba,             nflex,         4                    ) \
    XMJV( float,   flex_texcoord,         nflextexcoord, 2                    ) \
    X   ( int,     mesh_vertadr,          nmesh,         1                    ) \
    X   ( int,     mesh_vertnum,          nmesh,         1                    ) \
    X   ( int,     mesh_normaladr,        nmesh,         1                    ) \
    X   ( int,     mesh_normalnum,        nmesh,         1                    ) \
    XMJV( int,     mesh_texcoordadr,      nmesh,         1                    ) \
    X   ( int,     mesh_texcoordnum,      nmesh,         1                    ) \
    X   ( int,     mesh_faceadr,          nmesh,         1                    ) \
    X   ( int,     mesh_facenum,          nmesh,         1                    ) \
    XMJV( int,     mesh_bvhadr,           nmesh,         1                    ) \
    XMJV( int,     mesh_bvhnum,           nmesh,         1                    ) \
    XMJV( int,     mesh_graphadr,         nmesh,         1                    ) \
    X   ( mjtNum,  mesh_scale,            nmesh,         3                    ) \
    X   ( mjtNum,  mesh_pos,              nmesh,         3                    ) \
    X   ( mjtNum,  mesh_quat,             nmesh,         4                    ) \
    XNV ( float,   mesh_vert,             nmeshvert,     3                    ) \
    XNV ( float,   mesh_normal,           nmeshnormal,   3                    ) \
    XNV ( float,   mesh_texcoord,         nmeshtexcoord, 2                    ) \
    XNV ( int,     mesh_face,             nmeshface,     3                    ) \
    XNV ( int,     mesh_facenormal,       nmeshface,     3                    ) \
    XNV ( int,     mesh_facetexcoord,     nmeshface,     3                    ) \
    XNV ( int,     mesh_graph,            nmeshgraph,    1                    ) \
    XMJV( int,     mesh_pathadr,          nmesh,         1                    ) \
    X   ( int,     mesh_polynum,          nmesh,         1                    ) \
    X   ( int,     mesh_polyadr,          nmesh,         1                    ) \
    X   ( mjtNum,  mesh_polynormal,       nmeshpoly,     3                    ) \
    X   ( int,     mesh_polyvertadr,      nmeshpoly,     1                    ) \
    X   ( int,     mesh_polyvertnum,      nmeshpoly,     1                    ) \
    X   ( int,     mesh_polyvert,         nmeshpolyvert, 1                    ) \
    X   ( int,     mesh_polymapadr,       nmeshvert,     1                    ) \
    X   ( int,     mesh_polymapnum,       nmeshvert,     1                    ) \
    X   ( int,     mesh_polymap,          nmeshpolymap,  1                    ) \
    XMJV( int,     skin_matid,            nskin,         1                    ) \
    XMJV( int,     skin_group,            nskin,         1                    ) \
    XMJV( float,   skin_rgba,             nskin,         4                    ) \
    XMJV( float,   skin_inflate,          nskin,         1                    ) \
    XMJV( int,     skin_vertadr,          nskin,         1                    ) \
    XMJV( int,     skin_vertnum,          nskin,         1                    ) \
    XMJV( int,     skin_texcoordadr,      nskin,         1                    ) \
    XMJV( int,     skin_faceadr,          nskin,         1                    ) \
    XMJV( int,     skin_facenum,          nskin,         1                    ) \
    XMJV( int,     skin_boneadr,          nskin,         1                    ) \
    XMJV( int,     skin_bonenum,          nskin,         1                    ) \
    XMJV( float,   skin_vert,             nskinvert,     3                    ) \
    X   ( float,   skin_texcoord,         nskintexvert,  2                    ) \
    XMJV( int,     skin_face,             nskinface,     3                    ) \
    XMJV( int,     skin_bonevertadr,      nskinbone,     1                    ) \
    XMJV( int,     skin_bonevertnum,      nskinbone,     1                    ) \
    XMJV( float,   skin_bonebindpos,      nskinbone,     3                    ) \
    XMJV( float,   skin_bonebindquat,     nskinbone,     4                    ) \
    XMJV( int,     skin_bonebodyid,       nskinbone,     1                    ) \
    XMJV( int,     skin_bonevertid,       nskinbonevert, 1                    ) \
    XMJV( float,   skin_bonevertweight,   nskinbonevert, 1                    ) \
    XMJV( int,     skin_pathadr,          nskin,         1                    ) \
    X   ( mjtNum,  hfield_size,           nhfield,       4                    ) \
    X   ( int,     hfield_nrow,           nhfield,       1                    ) \
    X   ( int,     hfield_ncol,           nhfield,       1                    ) \
    X   ( int,     hfield_adr,            nhfield,       1                    ) \
    XNV ( float,   hfield_data,           nhfielddata,   1                    ) \
    XMJV( int,     hfield_pathadr,        nhfield,       1                    ) \
    X   ( int,     tex_type,              ntex,          1                    ) \
    X   ( int,     tex_height,            ntex,          1                    ) \
    X   ( int,     tex_width,             ntex,          1                    ) \
    X   ( int,     tex_nchannel,          ntex,          1                    ) \
    X   ( int,     tex_adr,               ntex,          1                    ) \
    XNV ( mjtByte, tex_data,              ntexdata,      1                    ) \
    XMJV( int,     tex_pathadr,           ntex,          1                    ) \
    XMJV( int,     mat_texid,             nmat,          mjNTEXROLE           ) \
    XMJV( mjtByte, mat_texuniform,        nmat,          1                    ) \
    XMJV( float,   mat_texrepeat,         nmat,          2                    ) \
    XMJV( float,   mat_emission,          nmat,          1                    ) \
    XMJV( float,   mat_specular,          nmat,          1                    ) \
    XMJV( float,   mat_shininess,         nmat,          1                    ) \
    XMJV( float,   mat_reflectance,       nmat,          1                    ) \
    XMJV( float,   mat_metallic,          nmat,          1                    ) \
    XMJV( float,   mat_roughness,         nmat,          1                    ) \
    XMJV( float,   mat_rgba,              nmat,          4                    ) \
    X   ( int,     pair_dim,              npair,         1                    ) \
    X   ( int,     pair_geom1,            npair,         1                    ) \
    X   ( int,     pair_geom2,            npair,         1                    ) \
    X   ( int,     pair_signature,        npair,         1                    ) \
    X   ( mjtNum,  pair_solref,           npair,         mjNREF               ) \
    X   ( mjtNum,  pair_solreffriction,   npair,         mjNREF               ) \
    X   ( mjtNum,  pair_solimp,           npair,         mjNIMP               ) \
    X   ( mjtNum,  pair_margin,           npair,         1                    ) \
    X   ( mjtNum,  pair_gap,              npair,         1                    ) \
    X   ( mjtNum,  pair_friction,         npair,         5                    ) \
    X   ( int,     exclude_signature,     nexclude,      1                    ) \
    XMJV( int,     eq_type,               neq,           1                    ) \
    XMJV( int,     eq_obj1id,             neq,           1                    ) \
    XMJV( int,     eq_obj2id,             neq,           1                    ) \
    XMJV( int,     eq_objtype,            neq,           1                    ) \
    X   ( mjtByte, eq_active0,            neq,           1                    ) \
    X   ( mjtNum,  eq_solref,             neq,           mjNREF               ) \
    X   ( mjtNum,  eq_solimp,             neq,           mjNIMP               ) \
    XMJV( mjtNum,  eq_data,               neq,           mjNEQDATA            ) \
    X   ( int,     tendon_adr,            ntendon,       1                    ) \
    XMJV( int,     tendon_num,            ntendon,       1                    ) \
    XMJV( int,     tendon_matid,          ntendon,       1                    ) \
    XMJV( int,     tendon_group,          ntendon,       1                    ) \
    XMJV( mjtByte, tendon_limited,        ntendon,       1                    ) \
    XMJV( mjtByte, tendon_actfrclimited,  ntendon,       1                    ) \
    XMJV( mjtNum,  tendon_width,          ntendon,       1                    ) \
    X   ( mjtNum,  tendon_solref_lim,     ntendon,       mjNREF               ) \
    X   ( mjtNum,  tendon_solimp_lim,     ntendon,       mjNIMP               ) \
    X   ( mjtNum,  tendon_solref_fri,     ntendon,       mjNREF               ) \
    X   ( mjtNum,  tendon_solimp_fri,     ntendon,       mjNIMP               ) \
    XMJV( mjtNum,  tendon_range,          ntendon,       2                    ) \
    XMJV( mjtNum,  tendon_actfrcrange,    ntendon,       2                    ) \
    X   ( mjtNum,  tendon_margin,         ntendon,       1                    ) \
    XMJV( mjtNum,  tendon_stiffness,      ntendon,       1                    ) \
    XMJV( mjtNum,  tendon_damping,        ntendon,       1                    ) \
    X   ( mjtNum,  tendon_armature,       ntendon,       1                    ) \
    XMJV( mjtNum,  tendon_frictionloss,   ntendon,       1                    ) \
    XMJV( mjtNum,  tendon_lengthspring,   ntendon,       2                    ) \
    X   ( mjtNum,  tendon_length0,        ntendon,       1                    ) \
    X   ( mjtNum,  tendon_invweight0,     ntendon,       1                    ) \
    X   ( mjtNum,  tendon_user,           ntendon,       MJ_M(nuser_tendon)   ) \
    XMJV( float,   tendon_rgba,           ntendon,       4                    ) \
    X   ( int,     wrap_type,             nwrap,         1                    ) \
    X   ( int,     wrap_objid,            nwrap,         1                    ) \
    X   ( mjtNum,  wrap_prm,              nwrap,         1                    ) \
    XMJV( int,     actuator_trntype,      nu,            1                    ) \
    XMJV( int,     actuator_dyntype,      nu,            1                    ) \
    X   ( int,     actuator_gaintype,     nu,            1                    ) \
    X   ( int,     actuator_biastype,     nu,            1                    ) \
    XMJV( int,     actuator_trnid,        nu,            2                    ) \
    XMJV( int,     actuator_actadr,       nu,            1                    ) \
    XMJV( int,     actuator_actnum,       nu,            1                    ) \
    XMJV( int,     actuator_group,        nu,            1                    ) \
    XMJV( mjtByte, actuator_ctrllimited,  nu,            1                    ) \
    X   ( mjtByte, actuator_forcelimited, nu,            1                    ) \
    XMJV( mjtByte, actuator_actlimited,   nu,            1                    ) \
    X   ( mjtNum,  actuator_dynprm,       nu,            mjNDYN               ) \
    X   ( mjtNum,  actuator_gainprm,      nu,            mjNGAIN              ) \
    X   ( mjtNum,  actuator_biasprm,      nu,            mjNBIAS              ) \
    X   ( mjtByte, actuator_actearly,     nu,            1                    ) \
    XMJV( mjtNum,  actuator_ctrlrange,    nu,            2                    ) \
    X   ( mjtNum,  actuator_forcerange,   nu,            2                    ) \
    XMJV( mjtNum,  actuator_actrange,     nu,            2                    ) \
    X   ( mjtNum,  actuator_gear,         nu,            6                    ) \
    XMJV( mjtNum,  actuator_cranklength,  nu,            1                    ) \
    X   ( mjtNum,  actuator_acc0,         nu,            1                    ) \
    X   ( mjtNum,  actuator_length0,      nu,            1                    ) \
    X   ( mjtNum,  actuator_lengthrange,  nu,            2                    ) \
    X   ( mjtNum,  actuator_user,         nu,            MJ_M(nuser_actuator) ) \
    X   ( int,     actuator_plugin,       nu,            1                    ) \
    XMJV( int,     sensor_type,           nsensor,       1                    ) \
    X   ( int,     sensor_datatype,       nsensor,       1                    ) \
    X   ( int,     sensor_needstage,      nsensor,       1                    ) \
    X   ( int,     sensor_objtype,        nsensor,       1                    ) \
    XMJV( int,     sensor_objid,          nsensor,       1                    ) \
    X   ( int,     sensor_reftype,        nsensor,       1                    ) \
    X   ( int,     sensor_refid,          nsensor,       1                    ) \
    X   ( int,     sensor_dim,            nsensor,       1                    ) \
    XMJV( int,     sensor_adr,            nsensor,       1                    ) \
    X   ( mjtNum,  sensor_cutoff,         nsensor,       1                    ) \
    X   ( mjtNum,  sensor_noise,          nsensor,       1                    ) \
    X   ( mjtNum,  sensor_user,           nsensor,       MJ_M(nuser_sensor)   ) \
    X   ( int,     sensor_plugin,         nsensor,       1                    ) \
    X   ( int,     plugin,                nplugin,       1                    ) \
    X   ( int,     plugin_stateadr,       nplugin,       1                    ) \
    X   ( int,     plugin_statenum,       nplugin,       1                    ) \
    X   ( char,    plugin_attr,           npluginattr,   1                    ) \
    X   ( int,     plugin_attradr,        nplugin,       1                    ) \
    X   ( int,     numeric_adr,           nnumeric,      1                    ) \
    X   ( int,     numeric_size,          nnumeric,      1                    ) \
    X   ( mjtNum,  numeric_data,          nnumericdata,  1                    ) \
    X   ( int,     text_adr,              ntext,         1                    ) \
    X   ( int,     text_size,             ntext,         1                    ) \
    X   ( char,    text_data,             ntextdata,     1                    ) \
    X   ( int,     tuple_adr,             ntuple,        1                    ) \
    X   ( int,     tuple_size,            ntuple,        1                    ) \
    X   ( int,     tuple_objtype,         ntupledata,    1                    ) \
    X   ( int,     tuple_objid,           ntupledata,    1                    ) \
    X   ( mjtNum,  tuple_objprm,          ntupledata,    1                    ) \
    X   ( mjtNum,  key_time,              nkey,          1                    ) \
    X   ( mjtNum,  key_qpos,              nkey,          MJ_M(nq)             ) \
    X   ( mjtNum,  key_qvel,              nkey,          MJ_M(nv)             ) \
    X   ( mjtNum,  key_act,               nkey,          MJ_M(na)             ) \
    X   ( mjtNum,  key_mpos,              nkey,          MJ_M(nmocap)*3       ) \
    X   ( mjtNum,  key_mquat,             nkey,          MJ_M(nmocap)*4       ) \
    X   ( mjtNum,  key_ctrl,              nkey,          MJ_M(nu)             ) \
    XMJV( int,     name_bodyadr,          nbody,         1                    ) \
    XMJV( int,     name_jntadr,           njnt,          1                    ) \
    XMJV( int,     name_geomadr,          ngeom,         1                    ) \
    XMJV( int,     name_siteadr,          nsite,         1                    ) \
    XMJV( int,     name_camadr,           ncam,          1                    ) \
    XMJV( int,     name_lightadr,         nlight,        1                    ) \
    X   ( int,     name_flexadr,          nflex,         1                    ) \
    X   ( int,     name_meshadr,          nmesh,         1                    ) \
    X   ( int,     name_skinadr,          nskin,         1                    ) \
    X   ( int,     name_hfieldadr,        nhfield,       1                    ) \
    X   ( int,     name_texadr,           ntex,          1                    ) \
    X   ( int,     name_matadr,           nmat,          1                    ) \
    X   ( int,     name_pairadr,          npair,         1                    ) \
    X   ( int,     name_excludeadr,       nexclude,      1                    ) \
    XMJV( int,     name_eqadr,            neq,           1                    ) \
    XMJV( int,     name_tendonadr,        ntendon,       1                    ) \
    XMJV( int,     name_actuatoradr,      nu,            1                    ) \
    X   ( int,     name_sensoradr,        nsensor,       1                    ) \
    X   ( int,     name_numericadr,       nnumeric,      1                    ) \
    X   ( int,     name_textadr,          ntext,         1                    ) \
    X   ( int,     name_tupleadr,         ntuple,        1                    ) \
    X   ( int,     name_keyadr,           nkey,          1                    ) \
    X   ( int,     name_pluginadr,        nplugin,       1                    ) \
    XMJV( char,    names,                 nnames,        1                    ) \
    X   ( int,     names_map,             nnames_map,    1                    ) \
    XMJV( char,    paths,                 npaths,        1                    ) \

//-------------------------------- mjData ----------------------------------------------------------

// define symbols needed in MJDATA_POINTERS (corresponding to number of columns)
#define MJDATA_POINTERS_PREAMBLE( m ) \
    int nv = m->nv;


// pointer fields of mjData
// XMJV means that the field is required to construct mjvScene
// (by default we define XMJV to be the same as X)
#define MJDATA_POINTERS                                            \
    X   ( mjtNum,    qpos,              nq,          1           ) \
    X   ( mjtNum,    qvel,              nv,          1           ) \
    XMJV( mjtNum,    act,               na,          1           ) \
    X   ( mjtNum,    qacc_warmstart,    nv,          1           ) \
    X   ( mjtNum,    plugin_state,      npluginstate, 1          ) \
    XMJV( mjtNum,    ctrl,              nu,          1           ) \
    X   ( mjtNum,    qfrc_applied,      nv,          1           ) \
    XMJV( mjtNum,    xfrc_applied,      nbody,       6           ) \
    XMJV( mjtByte,   eq_active,         neq,         1           ) \
    X   ( mjtNum,    mocap_pos,         nmocap,      3           ) \
    X   ( mjtNum,    mocap_quat,        nmocap,      4           ) \
    X   ( mjtNum,    qacc,              nv,          1           ) \
    X   ( mjtNum,    act_dot,           na,          1           ) \
    X   ( mjtNum,    userdata,          nuserdata,   1           ) \
    XMJV( mjtNum,    sensordata,        nsensordata, 1           ) \
    X   ( int,       plugin,            nplugin,     1           ) \
    X   ( uintptr_t, plugin_data,       nplugin,     1           ) \
    XMJV( mjtNum,    xpos,              nbody,       3           ) \
    XMJV( mjtNum,    xquat,             nbody,       4           ) \
    XMJV( mjtNum,    xmat,              nbody,       9           ) \
    XMJV( mjtNum,    xipos,             nbody,       3           ) \
    XMJV( mjtNum,    ximat,             nbody,       9           ) \
    XMJV( mjtNum,    xanchor,           njnt,        3           ) \
    XMJV( mjtNum,    xaxis,             njnt,        3           ) \
    XMJV( mjtNum,    geom_xpos,         ngeom,       3           ) \
    XMJV( mjtNum,    geom_xmat,         ngeom,       9           ) \
    XMJV( mjtNum,    site_xpos,         nsite,       3           ) \
    XMJV( mjtNum,    site_xmat,         nsite,       9           ) \
    XMJV( mjtNum,    cam_xpos,          ncam,        3           ) \
    XMJV( mjtNum,    cam_xmat,          ncam,        9           ) \
    XMJV( mjtNum,    light_xpos,        nlight,      3           ) \
    XMJV( mjtNum,    light_xdir,        nlight,      3           ) \
    XMJV( mjtNum,    subtree_com,       nbody,       3           ) \
    X   ( mjtNum,    cdof,              nv,          6           ) \
    X   ( mjtNum,    cinert,            nbody,       10          ) \
    XMJV( mjtNum,    flexvert_xpos,     nflexvert,   3           ) \
    X   ( mjtNum,    flexelem_aabb,     nflexelem,   6           ) \
    X   ( int,       flexedge_J_rownnz, nflexedge,   1           ) \
    X   ( int,       flexedge_J_rowadr, nflexedge,   1           ) \
    X   ( int,       flexedge_J_colind, nflexedge,   MJ_M(nv)    ) \
    X   ( mjtNum,    flexedge_J,        nflexedge,   MJ_M(nv)    ) \
    X   ( mjtNum,    flexedge_length,   nflexedge,   1           ) \
    XMJV( int,       ten_wrapadr,       ntendon,     1           ) \
    XMJV( int,       ten_wrapnum,       ntendon,     1           ) \
    X   ( int,       ten_J_rownnz,      ntendon,     1           ) \
    X   ( int,       ten_J_rowadr,      ntendon,     1           ) \
    X   ( int,       ten_J_colind,      ntendon,     MJ_M(nv)    ) \
    XMJV( mjtNum,    ten_length,        ntendon,     1           ) \
    X   ( mjtNum,    ten_J,             ntendon,     MJ_M(nv)    ) \
    XMJV( int,       wrap_obj,          nwrap,       2           ) \
    XMJV( mjtNum,    wrap_xpos,         nwrap,       6           ) \
    X   ( mjtNum,    actuator_length,   nu,          1           ) \
    X   ( int,       moment_rownnz,     nu,          1           ) \
    X   ( int,       moment_rowadr,     nu,          1           ) \
    X   ( int,       moment_colind,     nJmom,       1           ) \
    X   ( mjtNum,    actuator_moment,   nJmom,       1           ) \
    X   ( mjtNum,    crb,               nbody,       10          ) \
    X   ( mjtNum,    qM,                nM,          1           ) \
    X   ( mjtNum,    qLD,               nM,          1           ) \
    X   ( mjtNum,    qLDiagInv,         nv,          1           ) \
    XMJV( mjtNum,    bvh_aabb_dyn,      nbvhdynamic, 6           ) \
    XMJV( mjtByte,   bvh_active,        nbvh,        1           ) \
    X   ( mjtNum,    flexedge_velocity, nflexedge,   1           ) \
    X   ( mjtNum,    ten_velocity,      ntendon,     1           ) \
    X   ( mjtNum,    actuator_velocity, nu,          1           ) \
    X   ( mjtNum,    cvel,              nbody,       6           ) \
    X   ( mjtNum,    cdof_dot,          nv,          6           ) \
    X   ( mjtNum,    qfrc_bias,         nv,          1           ) \
    X   ( mjtNum,    qfrc_spring,       nv,          1           ) \
    X   ( mjtNum,    qfrc_damper,       nv,          1           ) \
    X   ( mjtNum,    qfrc_gravcomp,     nv,          1           ) \
    X   ( mjtNum,    qfrc_fluid,        nv,          1           ) \
    X   ( mjtNum,    qfrc_passive,      nv,          1           ) \
    X   ( mjtNum,    subtree_linvel,    nbody,       3           ) \
    X   ( mjtNum,    subtree_angmom,    nbody,       3           ) \
    X   ( mjtNum,    qH,                nM,          1           ) \
    X   ( mjtNum,    qHDiagInv,         nv,          1           ) \
    X   ( int,       B_rownnz,          nbody,       1           ) \
    X   ( int,       B_rowadr,          nbody,       1           ) \
    X   ( int,       B_colind,          nB,          1           ) \
    X   ( int,       M_rownnz,          nv,          1           ) \
    X   ( int,       M_rowadr,          nv,          1           ) \
    X   ( int,       M_colind,          nM,          1           ) \
    X   ( int,       mapM2M,            nM,          1           ) \
    X   ( int,       C_rownnz,          nv,          1           ) \
    X   ( int,       C_rowadr,          nv,          1           ) \
    X   ( int,       C_colind,          nC,          1           ) \
    X   ( int,       mapM2C,            nC,          1           ) \
    X   ( int,       D_rownnz,          nv,          1           ) \
    X   ( int,       D_rowadr,          nv,          1           ) \
    X   ( int,       D_diag,            nv,          1           ) \
    X   ( int,       D_colind,          nD,          1           ) \
    X   ( int,       mapM2D,            nD,          1           ) \
    X   ( int,       mapD2M,            nM,          1           ) \
    X   ( mjtNum,    qDeriv,            nD,          1           ) \
    X   ( mjtNum,    qLU,               nD,          1           ) \
    X   ( mjtNum,    actuator_force,    nu,          1           ) \
    X   ( mjtNum,    qfrc_actuator,     nv,          1           ) \
    X   ( mjtNum,    qfrc_smooth,       nv,          1           ) \
    X   ( mjtNum,    qacc_smooth,       nv,          1           ) \
    X   ( mjtNum,    qfrc_constraint,   nv,          1           ) \
    X   ( mjtNum,    qfrc_inverse,      nv,          1           ) \
    X   ( mjtNum,    cacc,              nbody,       6           ) \
    X   ( mjtNum,    cfrc_int,          nbody,       6           ) \
    X   ( mjtNum,    cfrc_ext,          nbody,       6           )


// macro for annotating that an array size in an X macro is a member of mjData
// by default this macro does nothing, but users can redefine it as necessary
#define MJ_D(n) n

// array of contacts
#define MJDATA_ARENA_POINTERS_CONTACT \
    X( mjContact, contact, MJ_D(ncon), 1 )

// array fields of mjData that are used in the primal problem
#define MJDATA_ARENA_POINTERS_SOLVER                   \
    X( int,      efc_type,          MJ_D(nefc),    1 ) \
    X( int,      efc_id,            MJ_D(nefc),    1 ) \
    X( int,      efc_J_rownnz,      MJ_D(nefc),    1 ) \
    X( int,      efc_J_rowadr,      MJ_D(nefc),    1 ) \
    X( int,      efc_J_rowsuper,    MJ_D(nefc),    1 ) \
    X( int,      efc_J_colind,      MJ_D(nJ),      1 ) \
    X( int,      efc_JT_rownnz,     MJ_M(nv),      1 ) \
    X( int,      efc_JT_rowadr,     MJ_M(nv),      1 ) \
    X( int,      efc_JT_rowsuper,   MJ_M(nv),      1 ) \
    X( int,      efc_JT_colind,     MJ_D(nJ),      1 ) \
    X( mjtNum,   efc_J,             MJ_D(nJ),      1 ) \
    X( mjtNum,   efc_JT,            MJ_D(nJ),      1 ) \
    X( mjtNum,   efc_pos,           MJ_D(nefc),    1 ) \
    X( mjtNum,   efc_margin,        MJ_D(nefc),    1 ) \
    X( mjtNum,   efc_frictionloss,  MJ_D(nefc),    1 ) \
    X( mjtNum,   efc_diagApprox,    MJ_D(nefc),    1 ) \
    X( mjtNum,   efc_KBIP,          MJ_D(nefc),    4 ) \
    X( mjtNum,   efc_D,             MJ_D(nefc),    1 ) \
    X( mjtNum,   efc_R,             MJ_D(nefc),    1 ) \
    X( int,      tendon_efcadr,     MJ_M(ntendon), 1 ) \
    X( mjtNum,   efc_vel,           MJ_D(nefc),    1 ) \
    X( mjtNum,   efc_aref,          MJ_D(nefc),    1 ) \
    X( mjtNum,   efc_b,             MJ_D(nefc),    1 ) \
    X( mjtNum,   efc_force,         MJ_D(nefc),    1 ) \
    X( int,      efc_state,         MJ_D(nefc),    1 )

// array fields of mjData that are used in the dual problem
#define MJDATA_ARENA_POINTERS_DUAL                           \
    X( int,      efc_AR_rownnz,     MJ_D(nefc), 1          ) \
    X( int,      efc_AR_rowadr,     MJ_D(nefc), 1          ) \
    X( int,      efc_AR_colind,     MJ_D(nA),   1          ) \
    X( mjtNum,   efc_AR,            MJ_D(nA),   1          )

// array fields of mjData that are used for constraint islands
#define MJDATA_ARENA_POINTERS_ISLAND                 \
    X( int,   dof_island,         MJ_M(nv),      1 ) \
    X( int,   island_dofnum,      MJ_D(nisland), 1 ) \
    X( int,   island_dofadr,      MJ_D(nisland), 1 ) \
    X( int,   island_dofind,      MJ_M(nv),      1 ) \
    X( int,   dof_islandind,      MJ_M(nv),      1 ) \
    X( int,   efc_island,         MJ_D(nefc),    1 ) \
    X( int,   island_efcnum,      MJ_D(nisland), 1 ) \
    X( int,   island_efcadr,      MJ_D(nisland), 1 ) \
    X( int,   island_efcind,      MJ_D(nefc),    1 )

// array fields of mjData that live in d->arena
#define MJDATA_ARENA_POINTERS          \
    MJDATA_ARENA_POINTERS_CONTACT      \
    MJDATA_ARENA_POINTERS_SOLVER       \
    MJDATA_ARENA_POINTERS_DUAL         \
    MJDATA_ARENA_POINTERS_ISLAND


// scalar fields of mjData
#define MJDATA_SCALAR                  \
    X( size_t,    narena             ) \
    X( size_t,    nbuffer            ) \
    X( int,       nplugin            ) \
    X( size_t,    pstack             ) \
    X( size_t,    pbase              ) \
    X( size_t,    parena             ) \
    X( size_t,    maxuse_stack       ) \
    X( size_t,    maxuse_arena       ) \
    X( int,       maxuse_con         ) \
    X( int,       maxuse_efc         ) \
    X( int,       solver_nisland     ) \
    X( int,       ncon               ) \
    X( int,       ne                 ) \
    X( int,       nf                 ) \
    X( int,       nl                 ) \
    X( int,       nefc               ) \
    X( int,       nJ                 ) \
    X( int,       nA                 ) \
    X( int,       nisland            ) \
    X( mjtNum,    time               ) \
    X( uintptr_t, threadpool         )


// vector fields of mjData
#define MJDATA_VECTOR                                                \
    X( size_t,         maxuse_threadstack, mjMAXTHREAD,  1         ) \
    X( mjWarningStat,  warning,            mjNWARNING,   1         ) \
    X( mjTimerStat,    timer,              mjNTIMER,     1         ) \
    X( mjSolverStat,   solver,             mjNISLAND,    mjNSOLVER ) \
    X( int,            solver_niter,       mjNISLAND,    1         ) \
    X( int,            solver_nnz,         mjNISLAND,    1         ) \
    X( mjtNum,         solver_fwdinv,      2,            1         ) \
    X( mjtNum,         energy,             2,            1         )


// alias XMJV to be the same as X
// to obtain only X macros for fields that are relevant for mjvScene creation,
// redefine X to expand to nothing, and XMJV to do what's required
#define XMJV X

// alias XNV to be the same as X
// to obtain only X macros for fields that are relevant for mjvScene creation,
// redefine XNV to expand to nothing
#define XNV X

#endif  // MUJOCO_MJXMACRO_H_
