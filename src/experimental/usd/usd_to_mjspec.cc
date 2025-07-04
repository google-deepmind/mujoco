// Copyright 2025 DeepMind Technologies Limited
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

#include <cmath>
#include <cstddef>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <mujoco/experimental/usd/mjcPhysics/actuatorAPI.h>
#include <mujoco/experimental/usd/mjcPhysics/collisionAPI.h>
#include <mujoco/experimental/usd/mjcPhysics/jointAPI.h>
#include <mujoco/experimental/usd/mjcPhysics/keyframe.h>
#include <mujoco/experimental/usd/mjcPhysics/meshCollisionAPI.h>
#include <mujoco/experimental/usd/mjcPhysics/sceneAPI.h>
#include <mujoco/experimental/usd/mjcPhysics/siteAPI.h>
#include <mujoco/experimental/usd/mjcPhysics/tokens.h>
#include <mujoco/experimental/usd/usd.h>
#include <mujoco/experimental/usd/utils.h>
#include <mujoco/mujoco.h>
#include "experimental/usd/kinematic_tree.h"
#include <pxr/base/gf/declare.h>
#include <pxr/base/gf/matrix4d.h>
#include <pxr/base/gf/vec3d.h>
#include <pxr/base/tf/token.h>
#include <pxr/base/vt/types.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/common.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/primFlags.h>
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/capsule.h>
#include <pxr/usd/usdGeom/cube.h>
#include <pxr/usd/usdGeom/cylinder.h>
#include <pxr/usd/usdGeom/gprim.h>
#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/usd/usdGeom/plane.h>
#include <pxr/usd/usdGeom/primvar.h>
#include <pxr/usd/usdGeom/primvarsAPI.h>
#include <pxr/usd/usdGeom/sphere.h>
#include <pxr/usd/usdGeom/tokens.h>
#include <pxr/usd/usdGeom/xformCache.h>
#include <pxr/usd/usdPhysics/collisionAPI.h>
#include <pxr/usd/usdPhysics/fixedJoint.h>
#include <pxr/usd/usdPhysics/joint.h>
#include <pxr/usd/usdPhysics/massAPI.h>
#include <pxr/usd/usdPhysics/prismaticJoint.h>
#include <pxr/usd/usdPhysics/revoluteJoint.h>
#include <pxr/usd/usdPhysics/rigidBodyAPI.h>
#include <pxr/usd/usdPhysics/scene.h>
namespace {

using pxr::MjcPhysicsTokens;
using pxr::TfToken;

void SetDoubleArrFromGfVec3d(double* to, const pxr::GfVec3d& from) {
  to[0] = from[0];
  to[1] = from[1];
  to[2] = from[2];
}

void SetMjtNumArrFromGfVec3d(mjtNum* to, const pxr::GfVec3d& from) {
  to[0] = from[0];
  to[1] = from[1];
  to[2] = from[2];
}

void SetDoubleArrFromGfQuatd(double* to, const pxr::GfQuatd& from) {
  // pxr::GfQuatd uses wxyz stored as a real and an imaginary component.
  to[0] = from.GetReal();
  to[1] = from.GetImaginary()[0];
  to[2] = from.GetImaginary()[1];
  to[3] = from.GetImaginary()[2];
}

template <typename T>
void SetLocalPoseFromPrim(const pxr::UsdPrim& prim,
                          const pxr::UsdPrim& parent_prim, T* element,
                          pxr::UsdGeomXformCache& xform_cache) {
  pxr::GfMatrix4d xform = xform_cache.GetLocalToWorldTransform(prim);
  pxr::GfMatrix4d parent_xform =
      xform_cache.GetLocalToWorldTransform(parent_prim);
  pxr::GfMatrix4d relative_xform = xform * parent_xform.GetInverse();

  pxr::GfVec3d translation = relative_xform.ExtractTranslation();
  pxr::GfQuatd rotation =
      relative_xform.RemoveScaleShear().ExtractRotationQuat();

  SetDoubleArrFromGfVec3d(element->pos, translation);
  SetDoubleArrFromGfQuatd(element->quat, rotation);
}

pxr::GfVec3d GetScale(const pxr::GfMatrix4d& matrix) {
  pxr::GfMatrix4d rotation;
  pxr::GfVec3d scale;
  pxr::GfMatrix4d u;
  pxr::GfVec3d translation;
  pxr::GfMatrix4d p;
  if (!matrix.Factor(&rotation, &scale, &u, &translation, &p)) {
    // In the event that we could not factorize, return the identity.
    mju_error("Could not factorise matrix.");
    return pxr::GfVec3d(1, 1, 1);
  }
  return scale;
}

bool IsUniformScale(const pxr::GfVec3d& scale) {
  static const double epsilon = 1e-6;
  return fabs(scale[0] - scale[1]) < epsilon &&
         fabs(scale[1] - scale[2]) < epsilon;
}

template <typename T>
void SetScale(T* element, const pxr::GfMatrix4d& world_transform,
              const pxr::GfVec3d& scale) {
  pxr::GfVec3d transform_scale =
      pxr::GfCompMult(GetScale(world_transform), scale);
  element->size[0] = transform_scale[0];
  element->size[1] = transform_scale[1];
  element->size[2] = transform_scale[2];
}

template <typename T>
bool MaybeParseGeomPrimitive(const pxr::UsdPrim& prim, T* element,
                             pxr::UsdGeomXformCache& xform_cache) {
  auto world_xform = xform_cache.GetLocalToWorldTransform(prim);
  auto scale = GetScale(world_xform);
  if (prim.IsA<pxr::UsdGeomSphere>()) {
    double radius;
    if (!pxr::UsdGeomSphere(prim).GetRadiusAttr().Get<double>(&radius)) {
      mju_error("Could not get sphere radius attr.");
      return false;
    }
    // If scale is uniform (or *very close*) then create a sphere, otherwise
    // this is an ellipsoid.
    if (IsUniformScale(scale)) {
      element->type = mjGEOM_SPHERE;
      element->size[0] = scale[0] * radius;
      element->size[1] = scale[0] * radius;
      element->size[2] = scale[0] * radius;
    } else {
      element->type = mjGEOM_ELLIPSOID;
      element->size[0] = scale[0] * radius;
      element->size[1] = scale[1] * radius;
      element->size[2] = scale[2] * radius;
    }
  } else if (prim.IsA<pxr::UsdGeomCylinder>()) {
    auto cylinder = pxr::UsdGeomCylinder(prim);
    element->type = mjGEOM_CYLINDER;
    double radius;
    if (!cylinder.GetRadiusAttr().Get<double>(&radius)) {
      mju_error("Could not get cylinder radius attr.");
      return false;
    }

    double height;
    if (!cylinder.GetHeightAttr().Get<double>(&height)) {
      mju_error("Could not get cylinder height attr.");
      return false;
    }
    element->size[0] = scale[0] * radius;
    element->size[1] = scale[1] * height / 2.0f;
    element->size[2] = 0;
  } else if (prim.IsA<pxr::UsdGeomCapsule>()) {
    auto capsule = pxr::UsdGeomCapsule(prim);
    element->type = mjGEOM_CAPSULE;
    double radius;
    if (!capsule.GetRadiusAttr().Get<double>(&radius)) {
      mju_error("Could not get capsule radius attr.");
      return false;
    }

    double height;
    if (!capsule.GetHeightAttr().Get<double>(&height)) {
      mju_error("Could not get capsule height attr.");
      return false;
    }
    element->size[0] = scale[0] * radius;
    element->size[1] = scale[1] * height / 2.0f;
    element->size[2] = 0;
  } else if (prim.IsA<pxr::UsdGeomCube>()) {
    element->type = mjGEOM_BOX;
    auto cube = pxr::UsdGeomCube(prim);
    double size;
    if (!cube.GetSizeAttr().Get<double>(&size)) {
      mju_error("Could not get cube size attr.");
      return false;
    }
    // MuJoCo uses half-length for box size.
    SetScale(element, xform_cache.GetLocalToWorldTransform(prim),
             pxr::GfVec3d(size / 2));

  } else {
    return false;
  }

  return true;
}

void ParseUsdPhysicsScene(mjSpec* spec,
                          const pxr::UsdPhysicsScene& physics_scene) {
  // Parse gravity and gravity direction.
  pxr::GfVec3f gravity_direction;
  physics_scene.GetGravityDirectionAttr().Get(&gravity_direction);

  float gravity_magnitude;
  physics_scene.GetGravityMagnitudeAttr().Get(&gravity_magnitude);

  gravity_direction *= gravity_magnitude;

  spec->option.gravity[0] = gravity_direction[0];
  spec->option.gravity[1] = gravity_direction[1];
  spec->option.gravity[2] = gravity_direction[2];

  // Early exit if theres no MjcPhysicsSceneAPI applied.
  if (!physics_scene.GetPrim().HasAPI<pxr::MjcPhysicsSceneAPI>()) {
    return;
  }
  auto mjc_physics_scene = pxr::MjcPhysicsSceneAPI(physics_scene.GetPrim());

  double timestep;
  mjc_physics_scene.GetTimestepAttr().Get(&timestep);
  spec->option.timestep = timestep;

  double apirate;
  mjc_physics_scene.GetApiRateAttr().Get(&apirate);
  spec->option.apirate = apirate;

  double impratio;
  mjc_physics_scene.GetImpRatioAttr().Get(&impratio);
  spec->option.impratio = impratio;

  double tolerance;
  mjc_physics_scene.GetToleranceAttr().Get(&tolerance);
  spec->option.tolerance = tolerance;

  double ls_tolerance;
  mjc_physics_scene.GetLSToleranceAttr().Get(&ls_tolerance);
  spec->option.ls_tolerance = ls_tolerance;

  double noslip_tolerance;
  mjc_physics_scene.GetNoslipToleranceAttr().Get(&noslip_tolerance);
  spec->option.noslip_tolerance = noslip_tolerance;

  double ccd_tolerance;
  mjc_physics_scene.GetCCDToleranceAttr().Get(&ccd_tolerance);
  spec->option.ccd_tolerance = ccd_tolerance;

  pxr::GfVec3d wind;
  mjc_physics_scene.GetWindAttr().Get(&wind);
  SetMjtNumArrFromGfVec3d(spec->option.wind, wind);

  pxr::GfVec3d magnetic;
  mjc_physics_scene.GetMagneticAttr().Get(&magnetic);
  SetMjtNumArrFromGfVec3d(spec->option.magnetic, magnetic);

  double density;
  mjc_physics_scene.GetDensityAttr().Get(&density);
  spec->option.density = density;

  double viscosity;
  mjc_physics_scene.GetViscosityAttr().Get(&viscosity);
  spec->option.viscosity = viscosity;

  double o_margin;
  mjc_physics_scene.GetOMarginAttr().Get(&o_margin);
  spec->option.o_margin = o_margin;

  pxr::VtDoubleArray o_solref;
  mjc_physics_scene.GetOSolRefAttr().Get(&o_solref);
  if (o_solref.size() != mjNREF) {
    mju_error("Invalid size for o_solref attribute: %zu expected %d",
              o_solref.size(), mjNREF);
    return;
  }
  for (int i = 0; i < mjNREF; ++i) {
    spec->option.o_solref[i] = o_solref[i];
  }

  pxr::VtDoubleArray o_solimp;
  mjc_physics_scene.GetOSolImpAttr().Get(&o_solimp);
  if (o_solimp.size() != mjNIMP) {
    mju_error("Invalid size for o_solimp attribute: %zu expected %d",
              o_solimp.size(), mjNIMP);
    return;
  }
  for (int i = 0; i < mjNIMP; ++i) {
    spec->option.o_solimp[i] = o_solimp[i];
  }

  pxr::VtDoubleArray o_friction;
  mjc_physics_scene.GetOFrictionAttr().Get(&o_friction);
  if (o_friction.size() != 5) {
    mju_error("Invalid size for o_friction attribute: %zu expected %d",
              o_friction.size(), 5);
    return;
  }
  for (int i = 0; i < 5; ++i) {
    spec->option.o_friction[i] = o_friction[i];
  }

  TfToken integrator;
  mjc_physics_scene.GetIntegratorAttr().Get(&integrator);
  if (integrator == MjcPhysicsTokens->euler) {
    spec->option.integrator = mjINT_EULER;
  } else if (integrator == MjcPhysicsTokens->rk4) {
    spec->option.integrator = mjINT_RK4;
  } else if (integrator == MjcPhysicsTokens->implicit) {
    spec->option.integrator = mjINT_IMPLICIT;
  } else if (integrator == MjcPhysicsTokens->implicitfast) {
    spec->option.integrator = mjINT_IMPLICITFAST;
  }

  TfToken cone;
  mjc_physics_scene.GetConeAttr().Get(&cone);
  if (cone == MjcPhysicsTokens->elliptic) {
    spec->option.cone = mjCONE_ELLIPTIC;
  } else if (cone == MjcPhysicsTokens->pyramidal) {
    spec->option.cone = mjCONE_PYRAMIDAL;
  }

  TfToken jacobian;
  mjc_physics_scene.GetJacobianAttr().Get(&jacobian);
  if (jacobian == MjcPhysicsTokens->auto_) {
    spec->option.jacobian = mjJAC_AUTO;
  } else if (jacobian == MjcPhysicsTokens->dense) {
    spec->option.jacobian = mjJAC_DENSE;
  } else if (jacobian == MjcPhysicsTokens->sparse) {
    spec->option.jacobian = mjJAC_SPARSE;
  }

  TfToken solver;
  mjc_physics_scene.GetSolverAttr().Get(&solver);
  if (solver == MjcPhysicsTokens->newton) {
    spec->option.solver = mjSOL_NEWTON;
  } else if (solver == MjcPhysicsTokens->cg) {
    spec->option.solver = mjSOL_CG;
  } else if (solver == MjcPhysicsTokens->pgs) {
    spec->option.solver = mjSOL_PGS;
  }

  int iterations;
  mjc_physics_scene.GetIterationsAttr().Get(&iterations);
  spec->option.iterations = iterations;

  int ls_iterations;
  mjc_physics_scene.GetLSIterationsAttr().Get(&ls_iterations);
  spec->option.ls_iterations = ls_iterations;

  int noslip_iterations;
  mjc_physics_scene.GetNoslipIterationsAttr().Get(&noslip_iterations);
  spec->option.noslip_iterations = noslip_iterations;

  int ccd_iterations;
  mjc_physics_scene.GetCCDIterationsAttr().Get(&ccd_iterations);
  spec->option.ccd_iterations = ccd_iterations;

  int sdf_initpoints;
  mjc_physics_scene.GetSDFInitPointsAttr().Get(&sdf_initpoints);
  spec->option.sdf_initpoints = sdf_initpoints;

  int sdf_iterations;
  mjc_physics_scene.GetSDFIterationsAttr().Get(&sdf_iterations);
  spec->option.sdf_iterations = sdf_iterations;

  bool constraint_flag;
  mjc_physics_scene.GetConstraintFlagAttr().Get(&constraint_flag);
  spec->option.disableflags |= (!constraint_flag ? mjDSBL_CONSTRAINT : 0);

  bool equality_flag;
  mjc_physics_scene.GetEqualityFlagAttr().Get(&equality_flag);
  spec->option.disableflags |= (!equality_flag ? mjDSBL_EQUALITY : 0);

  bool frictionloss_flag;
  mjc_physics_scene.GetFrictionLossFlagAttr().Get(&frictionloss_flag);
  spec->option.disableflags |= (!frictionloss_flag ? mjDSBL_FRICTIONLOSS : 0);

  bool limit_flag;
  mjc_physics_scene.GetLimitFlagAttr().Get(&limit_flag);
  spec->option.disableflags |= (!limit_flag ? mjDSBL_LIMIT : 0);

  bool contact_flag;
  mjc_physics_scene.GetContactFlagAttr().Get(&contact_flag);
  spec->option.disableflags |= (!contact_flag ? mjDSBL_CONTACT : 0);

  bool passive_flag;
  mjc_physics_scene.GetPassiveFlagAttr().Get(&passive_flag);
  spec->option.disableflags |= (!passive_flag ? mjDSBL_PASSIVE : 0);

  bool gravity_flag;
  mjc_physics_scene.GetGravityFlagAttr().Get(&gravity_flag);
  spec->option.disableflags |= (!gravity_flag ? mjDSBL_GRAVITY : 0);

  bool clampctrl_flag;
  mjc_physics_scene.GetClampCtrlFlagAttr().Get(&clampctrl_flag);
  spec->option.disableflags |= (!clampctrl_flag ? mjDSBL_CLAMPCTRL : 0);

  bool warmstart_flag;
  mjc_physics_scene.GetWarmStartFlagAttr().Get(&warmstart_flag);
  spec->option.disableflags |= (!warmstart_flag ? mjDSBL_WARMSTART : 0);

  bool filterparent_flag;
  mjc_physics_scene.GetFilterParentFlagAttr().Get(&filterparent_flag);
  spec->option.disableflags |= (!filterparent_flag ? mjDSBL_FILTERPARENT : 0);

  bool actuation_flag;
  mjc_physics_scene.GetActuationFlagAttr().Get(&actuation_flag);
  spec->option.disableflags |= (!actuation_flag ? mjDSBL_ACTUATION : 0);

  bool refsafe_flag;
  mjc_physics_scene.GetRefSafeFlagAttr().Get(&refsafe_flag);
  spec->option.disableflags |= (!refsafe_flag ? mjDSBL_REFSAFE : 0);

  bool sensor_flag;
  mjc_physics_scene.GetSensorFlagAttr().Get(&sensor_flag);
  spec->option.disableflags |= (!sensor_flag ? mjDSBL_SENSOR : 0);

  bool midphase_flag;
  mjc_physics_scene.GetMidPhaseFlagAttr().Get(&midphase_flag);
  spec->option.disableflags |= (!midphase_flag ? mjDSBL_MIDPHASE : 0);

  bool nativeccd_flag;
  mjc_physics_scene.GetNativeCCDFlagAttr().Get(&nativeccd_flag);
  spec->option.disableflags |= (!nativeccd_flag ? mjDSBL_NATIVECCD : 0);

  bool eulerdamp_flag;
  mjc_physics_scene.GetEulerDampFlagAttr().Get(&eulerdamp_flag);
  spec->option.disableflags |= (!eulerdamp_flag ? mjDSBL_EULERDAMP : 0);

  bool autoreset_flag;
  mjc_physics_scene.GetAutoResetFlagAttr().Get(&autoreset_flag);
  spec->option.disableflags |= (!autoreset_flag ? mjDSBL_AUTORESET : 0);

  bool override_flag;
  mjc_physics_scene.GetOverrideFlagAttr().Get(&override_flag);
  spec->option.enableflags |= (override_flag ? mjENBL_OVERRIDE : 0);

  bool energy_flag;
  mjc_physics_scene.GetEnergyFlagAttr().Get(&energy_flag);
  spec->option.enableflags |= (energy_flag ? mjENBL_ENERGY : 0);

  bool fwdinv_flag;
  mjc_physics_scene.GetFwdinvFlagAttr().Get(&fwdinv_flag);
  spec->option.enableflags |= (fwdinv_flag ? mjENBL_FWDINV : 0);

  bool invdiscrete_flag;
  mjc_physics_scene.GetInvDiscreteFlagAttr().Get(&invdiscrete_flag);
  spec->option.enableflags |= (invdiscrete_flag ? mjENBL_INVDISCRETE : 0);

  bool multiccd_flag;
  mjc_physics_scene.GetMultiCCDFlagAttr().Get(&multiccd_flag);
  spec->option.enableflags |= (multiccd_flag ? mjENBL_MULTICCD : 0);

  bool island_flag;
  mjc_physics_scene.GetIslandFlagAttr().Get(&island_flag);
  spec->option.enableflags |= (island_flag ? mjENBL_ISLAND : 0);
}

void ParseUsdPhysicsMassAPIForBody(mjsBody* body,
                                   const pxr::UsdPhysicsMassAPI& mass_api) {
  auto mass_attr = mass_api.GetMassAttr();
  if (mass_attr.HasAuthoredValue()) {
    float mass;
    mass_attr.Get(&mass);
    body->mass = mass;
  }

  auto com_attr = mass_api.GetCenterOfMassAttr();
  if (com_attr.HasAuthoredValue()) {
    pxr::GfVec3f com;
    com_attr.Get(&com);
    SetDoubleArrFromGfVec3d(body->ipos, com);
  }

  auto principle_axes_attr = mass_api.GetPrincipalAxesAttr();
  if (principle_axes_attr.HasAuthoredValue()) {
    pxr::GfQuatf principle_axes;
    principle_axes_attr.Get(&principle_axes);
    SetDoubleArrFromGfQuatd(body->iquat, principle_axes);
  }

  auto diag_inertia_attr = mass_api.GetDiagonalInertiaAttr();
  if (diag_inertia_attr.HasAuthoredValue()) {
    pxr::GfVec3f diag_inertia;
    diag_inertia_attr.Get(&diag_inertia);
    SetDoubleArrFromGfVec3d(body->inertia, diag_inertia);
  }
}

void ParseUsdPhysicsMassAPIForGeom(mjsGeom* geom,
                                   const pxr::UsdPhysicsMassAPI& mass_api) {
  auto mass_attr = mass_api.GetMassAttr();
  if (mass_attr.HasAuthoredValue()) {
    float mass;
    mass_attr.Get(&mass);
    geom->mass = mass;
  }

  auto density_attr = mass_api.GetDensityAttr();
  if (density_attr.HasAuthoredValue()) {
    float density;
    density_attr.Get(&density);
    geom->density = density;
  }
}

void ParseMjcPhysicsCollisionAPI(
    mjsGeom* geom, const pxr::MjcPhysicsCollisionAPI& collision_api) {
  auto shell_inertia_attr = collision_api.GetShellInertiaAttr();
  if (shell_inertia_attr.HasAuthoredValue()) {
    bool shell_inertia;
    shell_inertia_attr.Get(&shell_inertia);
    geom->typeinertia = shell_inertia ? mjtGeomInertia::mjINERTIA_SHELL
                                      : mjtGeomInertia::mjINERTIA_VOLUME;
  }
}

void ParseMjcPhysicsMeshCollisionAPI(
    mjsMesh* mesh, const pxr::MjcPhysicsMeshCollisionAPI& mesh_collision_api) {
  auto inertia_attr = mesh_collision_api.GetInertiaAttr();
  if (inertia_attr.HasAuthoredValue()) {
    pxr::TfToken inertia;
    inertia_attr.Get(&inertia);
    if (inertia == MjcPhysicsTokens->shell) {
      mesh->inertia = mjtMeshInertia::mjMESH_INERTIA_SHELL;
    } else if (inertia == MjcPhysicsTokens->exact) {
      mesh->inertia = mjtMeshInertia::mjMESH_INERTIA_EXACT;
    } else if (inertia == MjcPhysicsTokens->convex) {
      mesh->inertia = mjtMeshInertia::mjMESH_INERTIA_CONVEX;
    } else {
      mesh->inertia = mjtMeshInertia::mjMESH_INERTIA_LEGACY;
    }
  }
}

void ParseMjcPhysicsGeneralActuatorAPI(mjSpec* spec,
                                       const pxr::MjcPhysicsActuatorAPI& act,
                                       mjtTrn tran_type,
                                       const std::string* name) {
  pxr::UsdPrim prim = act.GetPrim();
  mjsActuator* mj_act = mjs_addActuator(spec, nullptr);
  mjs_setName(mj_act->element, prim.GetPath().GetAsString().c_str());
  mjs_setString(mj_act->target, name->c_str());
  mj_act->trntype = tran_type;

  if (tran_type == mjtTrn::mjTRN_SLIDERCRANK) {
    pxr::SdfPathVector slider_sites;
    act.GetMjcSliderSiteRel().GetTargets(&slider_sites);
    if (slider_sites.size() > 1) {
      mju_warning(
          "Slider crank slider site relationship has more than one target, "
          "using the first.");
    }
    mjs_setString(mj_act->slidersite, slider_sites[0].GetAsString().c_str());
  }

  auto setLimitedField = [](mjsActuator* mj_act,
                            const pxr::UsdAttribute& usd_attribute,
                            int* mj_limited_field) {
    if (usd_attribute.HasAuthoredValue()) {
      pxr::TfToken limited;
      usd_attribute.Get(&limited);
      if (limited == MjcPhysicsTokens->true_) {
        *mj_limited_field = mjLIMITED_TRUE;
      } else if (limited == MjcPhysicsTokens->false_) {
        *mj_limited_field = mjLIMITED_FALSE;
      } else if (limited == MjcPhysicsTokens->auto_) {
        *mj_limited_field = mjLIMITED_AUTO;
      }
    }
  };

  setLimitedField(mj_act, act.GetMjcCtrlLimitedAttr(), &mj_act->ctrllimited);
  setLimitedField(mj_act, act.GetMjcForceLimitedAttr(), &mj_act->forcelimited);
  setLimitedField(mj_act, act.GetMjcActLimitedAttr(), &mj_act->actlimited);

  auto setRangeField =
      [](mjsActuator* mj_act, const pxr::UsdAttribute& usd_min_attribute,
         const pxr::UsdAttribute& usd_max_attribute, double* range) {
        if (usd_min_attribute.HasAuthoredValue()) {
          double range_min;
          usd_min_attribute.Get(&range_min);
          range[0] = range_min;
        }
        if (usd_max_attribute.HasAuthoredValue()) {
          double range_max;
          usd_max_attribute.Get(&range_max);
          range[1] = range_max;
        }
      };

  setRangeField(mj_act, act.GetMjcCtrlRangeMinAttr(),
                act.GetMjcCtrlRangeMaxAttr(), mj_act->ctrlrange);
  setRangeField(mj_act, act.GetMjcForceRangeMinAttr(),
                act.GetMjcForceRangeMaxAttr(), mj_act->forcerange);
  setRangeField(mj_act, act.GetMjcActRangeMinAttr(),
                act.GetMjcActRangeMaxAttr(), mj_act->actrange);
  setRangeField(mj_act, act.GetMjcLengthRangeMinAttr(),
                act.GetMjcLengthRangeMaxAttr(), mj_act->lengthrange);

  auto gear_attr = act.GetMjcGearAttr();
  if (gear_attr.HasAuthoredValue()) {
    pxr::VtDoubleArray gear;
    gear_attr.Get(&gear);
    for (int i = 0; i < 6; ++i) {
      mj_act->gear[i] = gear[i];
    }
  }

  auto crank_length_attr = act.GetMjcCrankLengthAttr();
  if (crank_length_attr.HasAuthoredValue()) {
    double crank_length;
    crank_length_attr.Get(&crank_length);
    mj_act->cranklength = crank_length;
  }

  auto dyn_type_attr = act.GetMjcDynTypeAttr();
  if (dyn_type_attr.HasAuthoredValue()) {
    pxr::TfToken dyn_type;
    dyn_type_attr.Get(&dyn_type);
    if (dyn_type == MjcPhysicsTokens->none) {
      mj_act->dyntype = mjtDyn::mjDYN_NONE;
    } else if (dyn_type == MjcPhysicsTokens->integrator) {
      mj_act->dyntype = mjtDyn::mjDYN_INTEGRATOR;
    } else if (dyn_type == MjcPhysicsTokens->filter) {
      mj_act->dyntype = mjtDyn::mjDYN_FILTER;
    } else if (dyn_type == MjcPhysicsTokens->filterexact) {
      mj_act->dyntype = mjtDyn::mjDYN_FILTEREXACT;
    } else if (dyn_type == MjcPhysicsTokens->muscle) {
      mj_act->dyntype = mjtDyn::mjDYN_MUSCLE;
    } else if (dyn_type == MjcPhysicsTokens->user) {
      mj_act->dyntype = mjtDyn::mjDYN_USER;
    }
  }

  auto gain_type_attr = act.GetMjcGainTypeAttr();
  if (gain_type_attr.HasAuthoredValue()) {
    pxr::TfToken gain_type;
    gain_type_attr.Get(&gain_type);
    if (gain_type == MjcPhysicsTokens->fixed) {
      mj_act->gaintype = mjtGain::mjGAIN_FIXED;
    } else if (gain_type == MjcPhysicsTokens->affine) {
      mj_act->gaintype = mjtGain::mjGAIN_AFFINE;
    } else if (gain_type == MjcPhysicsTokens->muscle) {
      mj_act->gaintype = mjtGain::mjGAIN_MUSCLE;
    } else if (gain_type == MjcPhysicsTokens->user) {
      mj_act->gaintype = mjtGain::mjGAIN_USER;
    }
  }

  auto biastype_attr = act.GetMjcBiasTypeAttr();
  if (biastype_attr.HasAuthoredValue()) {
    pxr::TfToken biastype;
    biastype_attr.Get(&biastype);
    if (biastype == MjcPhysicsTokens->none) {
      mj_act->biastype = mjtBias::mjBIAS_NONE;
    } else if (biastype == MjcPhysicsTokens->affine) {
      mj_act->biastype = mjtBias::mjBIAS_AFFINE;
    } else if (biastype == MjcPhysicsTokens->muscle) {
      mj_act->biastype = mjtBias::mjBIAS_MUSCLE;
    } else if (biastype == MjcPhysicsTokens->user) {
      mj_act->biastype = mjtBias::mjBIAS_USER;
    }
  }

  auto setPrmField = [](mjsActuator* mj_act,
                        const pxr::UsdAttribute& usd_attribute, double* prm) {
    if (usd_attribute.HasAuthoredValue()) {
      pxr::VtDoubleArray usd_prm;
      usd_attribute.Get(&usd_prm);
      int n_elems = usd_prm.size() < 10 ? usd_prm.size() : 10;
      for (int i = 0; i < n_elems; ++i) {
        prm[i] = usd_prm[i];
      }
    }
  };

  setPrmField(mj_act, act.GetMjcDynPrmAttr(), mj_act->dynprm);
  setPrmField(mj_act, act.GetMjcBiasPrmAttr(), mj_act->biasprm);
  setPrmField(mj_act, act.GetMjcGainPrmAttr(), mj_act->gainprm);

  auto act_dim_attr = act.GetMjcActDimAttr();
  if (act_dim_attr.HasAuthoredValue()) {
    int act_dim;
    act_dim_attr.Get(&act_dim);
    mj_act->actdim = act_dim;
  }

  auto act_early_attr = act.GetMjcActEarlyAttr();
  if (act_early_attr.HasAuthoredValue()) {
    bool act_early;
    act_early_attr.Get(&act_early);
    mj_act->actearly = (int)act_early;
  }

  auto ref_site_rel = act.GetMjcRefSiteRel();
  if (ref_site_rel.HasAuthoredTargets()) {
    pxr::SdfPathVector targets;
    ref_site_rel.GetTargets(&targets);
    pxr::SdfPath first_path = targets[0];
    mjs_setString(mj_act->refsite, first_path.GetString().c_str());
  }
}

void ParseMjcPhysicsJointAPI(mjsJoint* mj_joint,
                             const pxr::MjcPhysicsJointAPI& joint_api) {
  auto springdamper_attr = joint_api.GetMjcSpringdamperAttr();
  if (springdamper_attr.HasAuthoredValue()) {
    pxr::VtDoubleArray springdamper;
    springdamper_attr.Get(&springdamper);
    if (springdamper.size() == 2) {
      mj_joint->springdamper[0] = springdamper[0];
      mj_joint->springdamper[1] = springdamper[1];
    } else {
      mju_warning(
          "springdamper attribute for joint %s has incorrect size %zu, "
          "expected 2.",
          mjs_getName(mj_joint->element)->c_str(), springdamper.size());
    }
  }

  auto solreflimit_attr = joint_api.GetMjcSolreflimitAttr();
  if (solreflimit_attr.HasAuthoredValue()) {
    pxr::VtDoubleArray solreflimit;
    solreflimit_attr.Get(&solreflimit);
    if (solreflimit.size() == mjNREF) {
      for (int i = 0; i < mjNREF; ++i) {
        mj_joint->solref_limit[i] = solreflimit[i];
      }
    } else {
      mju_warning(
          "solreflimit attribute for joint %s has incorrect size %zu, "
          "expected %d.",
          mjs_getName(mj_joint->element)->c_str(), solreflimit.size(), mjNREF);
    }
  }

  auto solimplimit_attr = joint_api.GetMjcSolimplimitAttr();
  if (solimplimit_attr.HasAuthoredValue()) {
    pxr::VtDoubleArray solimplimit;
    solimplimit_attr.Get(&solimplimit);
    if (solimplimit.size() == mjNIMP) {
      for (int i = 0; i < mjNIMP; ++i) {
        mj_joint->solimp_limit[i] = solimplimit[i];
      }
    } else {
      mju_warning(
          "solimplimit attribute for joint %s has incorrect size %zu, "
          "expected %d.",
          mjs_getName(mj_joint->element)->c_str(), solimplimit.size(), mjNIMP);
    }
  }

  auto solreffriction_attr = joint_api.GetMjcSolreffrictionAttr();
  if (solreffriction_attr.HasAuthoredValue()) {
    pxr::VtDoubleArray solreffriction;
    solreffriction_attr.Get(&solreffriction);
    if (solreffriction.size() == mjNREF) {
      for (int i = 0; i < mjNREF; ++i) {
        mj_joint->solref_friction[i] = solreffriction[i];
      }
    } else {
      mju_warning(
          "solreffriction attribute for joint %s has incorrect size %zu, "
          "expected %d.",
          mjs_getName(mj_joint->element)->c_str(), solreffriction.size(),
          mjNREF);
    }
  }

  auto solimpfriction_attr = joint_api.GetMjcSolimpfrictionAttr();
  if (solimpfriction_attr.HasAuthoredValue()) {
    pxr::VtDoubleArray solimpfriction;
    solimpfriction_attr.Get(&solimpfriction);
    if (solimpfriction.size() == mjNIMP) {
      for (int i = 0; i < mjNIMP; ++i) {
        mj_joint->solimp_friction[i] = solimpfriction[i];
      }
    } else {
      mju_warning(
          "solimpfriction attribute for joint %s has incorrect size %zu, "
          "expected %d.",
          mjs_getName(mj_joint->element)->c_str(), solimpfriction.size(),
          mjNIMP);
    }
  }

  auto stiffness_attr = joint_api.GetMjcStiffnessAttr();
  if (stiffness_attr.HasAuthoredValue()) {
    double stiffness;
    stiffness_attr.Get(&stiffness);
    mj_joint->stiffness = stiffness;
  }

  auto actuatorfrcrange_min_attr = joint_api.GetMjcActuatorfrcrangeMinAttr();
  if (actuatorfrcrange_min_attr.HasAuthoredValue()) {
    double min_val;
    actuatorfrcrange_min_attr.Get(&min_val);
    mj_joint->actfrcrange[0] = min_val;
  }
  auto actuatorfrcrange_max_attr = joint_api.GetMjcActuatorfrcrangeMaxAttr();
  if (actuatorfrcrange_max_attr.HasAuthoredValue()) {
    double max_val;
    actuatorfrcrange_max_attr.Get(&max_val);
    mj_joint->actfrcrange[1] = max_val;
  }

  auto actuatorfrclimited_attr = joint_api.GetMjcActuatorfrclimitedAttr();
  if (actuatorfrclimited_attr.HasAuthoredValue()) {
    pxr::TfToken limited;
    actuatorfrclimited_attr.Get(&limited);
    if (limited == MjcPhysicsTokens->true_) {
      mj_joint->actfrclimited = mjLIMITED_TRUE;
    } else if (limited == MjcPhysicsTokens->false_) {
      mj_joint->actfrclimited = mjLIMITED_FALSE;
    } else if (limited == MjcPhysicsTokens->auto_) {
      mj_joint->actfrclimited = mjLIMITED_AUTO;
    }
  }

  auto actuatorgravcomp_attr = joint_api.GetMjcActuatorgravcompAttr();
  if (actuatorgravcomp_attr.HasAuthoredValue()) {
    bool gravcomp;
    actuatorgravcomp_attr.Get(&gravcomp);
    mj_joint->actgravcomp = gravcomp;
  }

  auto margin_attr = joint_api.GetMjcMarginAttr();
  if (margin_attr.HasAuthoredValue()) {
    double margin;
    margin_attr.Get(&margin);
    mj_joint->margin = margin;
  }

  auto ref_attr = joint_api.GetMjcRefAttr();
  if (ref_attr.HasAuthoredValue()) {
    double ref;
    ref_attr.Get(&ref);
    mj_joint->ref = ref;
  }

  auto springref_attr = joint_api.GetMjcSpringrefAttr();
  if (springref_attr.HasAuthoredValue()) {
    double springref;
    springref_attr.Get(&springref);
    mj_joint->springref = springref;
  }

  auto armature_attr = joint_api.GetMjcArmatureAttr();
  if (armature_attr.HasAuthoredValue()) {
    double armature;
    armature_attr.Get(&armature);
    mj_joint->armature = armature;
  }

  auto damping_attr = joint_api.GetMjcDampingAttr();
  if (damping_attr.HasAuthoredValue()) {
    double damping;
    damping_attr.Get(&damping);
    mj_joint->damping = damping;
  }

  auto frictionloss_attr = joint_api.GetMjcFrictionlossAttr();
  if (frictionloss_attr.HasAuthoredValue()) {
    double frictionloss;
    frictionloss_attr.Get(&frictionloss);
    mj_joint->frictionloss = frictionloss;
  }
}

void ParseUsdPhysicsCollider(mjSpec* spec,
                             const pxr::UsdPhysicsCollisionAPI& collision_api,
                             const pxr::UsdPrim& body_prim, mjsBody* parent,
                             pxr::UsdGeomXformCache& xform_cache) {
  pxr::UsdPrim prim = collision_api.GetPrim();
  // UsdPhysicsCollisionAPI can only be applied to gprim primitives.
  if (!prim.IsA<pxr::UsdGeomGprim>()) {
    mju_warning(
        "UsdPhysicsCollisionAPI applied to a non-UsdGeomGprim prim: %s. "
        "Skipping.",
        prim.GetPath().GetAsString().c_str());
    return;
  }

  mjsGeom* geom = mjs_addGeom(parent, nullptr);
  mjs_setName(geom->element, prim.GetPath().GetAsString().c_str());
  geom->contype = 1;
  geom->conaffinity = 1;

  if (prim.HasAPI<pxr::UsdPhysicsMassAPI>()) {
    ParseUsdPhysicsMassAPIForGeom(geom, pxr::UsdPhysicsMassAPI(prim));
  }

  if (prim.HasAPI<pxr::MjcPhysicsCollisionAPI>()) {
    ParseMjcPhysicsCollisionAPI(geom, pxr::MjcPhysicsCollisionAPI(prim));
  }

  // Convert displayColor and displayOpacity to rgba.
  // We want to support primvar inheritance, hence FindPrimvarWithInheritance.
  pxr::UsdGeomPrimvarsAPI primvarsAPI(prim);
  pxr::UsdGeomPrimvar displayColorPrimvar =
      primvarsAPI.FindPrimvarWithInheritance(
          pxr::UsdGeomTokens->primvarsDisplayColor);
  pxr::UsdGeomPrimvar displayOpacityPrimvar =
      primvarsAPI.FindPrimvarWithInheritance(
          pxr::UsdGeomTokens->primvarsDisplayOpacity);
  if (displayColorPrimvar.HasAuthoredValue()) {
    pxr::VtArray<pxr::GfVec3f> display_color;
    displayColorPrimvar.Get(&display_color);
    if (!display_color.empty()) {
      geom->rgba[0] = display_color[0][0];
      geom->rgba[1] = display_color[0][1];
      geom->rgba[2] = display_color[0][2];
    }
  }
  if (displayOpacityPrimvar.HasAuthoredValue()) {
    pxr::VtArray<float> display_opacity;
    displayOpacityPrimvar.Get(&display_opacity);
    if (!display_opacity.empty()) {
      geom->rgba[3] = display_opacity[0];
    }
  }

  SetLocalPoseFromPrim(prim, body_prim, geom, xform_cache);

  if (!MaybeParseGeomPrimitive(prim, geom, xform_cache)) {
    if (prim.IsA<pxr::UsdGeomMesh>()) {
      geom->type = mjGEOM_MESH;
      pxr::UsdGeomMesh usd_mesh(prim);
      std::vector<float> uservert;
      std::vector<int> userface;

      pxr::VtVec3fArray points;
      usd_mesh.GetPointsAttr().Get(&points);

      uservert.reserve(points.size() * 3);
      for (const auto& pt : points) {
        uservert.push_back(pt[0]);
        uservert.push_back(pt[1]);
        uservert.push_back(pt[2]);
      }

      pxr::VtIntArray indices;
      usd_mesh.GetFaceVertexIndicesAttr().Get(&indices);
      pxr::VtIntArray counts;
      usd_mesh.GetFaceVertexCountsAttr().Get(&counts);

      userface.reserve(indices.size());
      int vtx_idx = 0;
      for (int count : counts) {
        int k = 1;
        // If the prim is a triangle create a triangle fan rooted
        // at the first index.
        while (k < count - 1) {
          userface.push_back(indices[vtx_idx]);
          userface.push_back(indices[vtx_idx + k]);
          userface.push_back(indices[vtx_idx + k + 1]);
          k++;
        }
        vtx_idx += count;
      }

      mjsMesh* mesh = mjs_addMesh(spec, nullptr);

      if (prim.HasAPI<pxr::MjcPhysicsMeshCollisionAPI>()) {
        ParseMjcPhysicsMeshCollisionAPI(mesh,
                                        pxr::MjcPhysicsMeshCollisionAPI(prim));
      }

      std::string mesh_name = usd_mesh.GetPath().GetAsString();
      mjs_setName(mesh->element, mesh_name.c_str());
      mjs_setFloat(mesh->uservert, uservert.data(), uservert.size());
      mjs_setInt(mesh->userface, userface.data(), userface.size());

      mjs_setString(geom->meshname, mesh_name.c_str());
    } else if (prim.IsA<pxr::UsdGeomPlane>()) {
      geom->type = mjGEOM_PLANE;

      pxr::UsdGeomPlane plane(prim);
      TfToken axis;
      if (!plane.GetAxisAttr().Get(&axis)) {
        mju_error("Could not get plane axis attr.");
        return;
      }
      if (axis != pxr::UsdGeomTokens->z) {
        mju_error("Only z-axis planes are supported.");
        return;
      }

      // This block of code distributes the plane length and width along the
      // scale as per the specification here:
      // https://openusd.org/dev/api/class_usd_geom_plane.html#a89fa6076111984682db77fc8a4e57496.
      double length;
      if (!plane.GetLengthAttr().Get(&length)) {
        mju_error("Could not get plane length attr.");
        return;
      }
      double width;
      if (!plane.GetWidthAttr().Get(&width)) {
        mju_error("Could not get plane width attr.");
        return;
      }
      // Plane geoms in mjc are always infinite. We set the scale here just
      // for visualization.
      SetScale(geom, xform_cache.GetLocalToWorldTransform(prim),
               pxr::GfVec3d(width, length, 1));
    }
  }
}

void ParseUsdPhysicsJoint(mjSpec* spec, const pxr::UsdPrim& prim, mjsBody* body,
                          pxr::UsdGeomXformCache& xform_cache) {
  pxr::UsdPhysicsJoint joint(prim);

  // A fixed joint means the bodies are welded.
  if (prim.IsA<pxr::UsdPhysicsFixedJoint>()) {
    // No joint needed for welded bodies.
    return;
  }

  mjtJoint type;
  if (prim.IsA<pxr::UsdPhysicsRevoluteJoint>()) {
    type = mjJNT_HINGE;
  } else if (prim.IsA<pxr::UsdPhysicsPrismaticJoint>()) {
    type = mjJNT_SLIDE;
  } else {
    // Unsupported joint type.
    mju_warning("Unsupported joint type for %s",
                prim.GetPath().GetAsString().c_str());
    return;
  }

  mjsJoint* mj_joint = mjs_addJoint(body, nullptr);
  mj_joint->type = type;
  mjs_setName(mj_joint->element, prim.GetPath().GetAsString().c_str());

  if (prim.IsA<pxr::UsdPhysicsRevoluteJoint>()) {
    pxr::UsdPhysicsRevoluteJoint revolute(prim);
    TfToken axis;
    revolute.GetAxisAttr().Get(&axis);
    if (axis == pxr::UsdGeomTokens->x) {
      mj_joint->axis[0] = 1;
      mj_joint->axis[1] = 0;
      mj_joint->axis[2] = 0;
    } else if (axis == pxr::UsdGeomTokens->y) {
      mj_joint->axis[0] = 0;
      mj_joint->axis[1] = 1;
      mj_joint->axis[2] = 0;
    } else {  // Z is default
      mj_joint->axis[0] = 0;
      mj_joint->axis[1] = 0;
      mj_joint->axis[2] = 1;
    }

    float lower, upper;
    if (revolute.GetLowerLimitAttr().Get(&lower) &&
        revolute.GetUpperLimitAttr().Get(&upper)) {
      mj_joint->limited = mjLIMITED_TRUE;
      // As per the XML Reference, the default unit for mjSpec is degrees, so we
      // don't need to convert from USD (which is degrees).
      mj_joint->range[0] = lower;
      mj_joint->range[1] = upper;
    }
  } else if (prim.IsA<pxr::UsdPhysicsPrismaticJoint>()) {
    pxr::UsdPhysicsPrismaticJoint prismatic(prim);
    TfToken axis;
    prismatic.GetAxisAttr().Get(&axis);
    if (axis == pxr::UsdGeomTokens->x) {
      mj_joint->axis[0] = 1;
      mj_joint->axis[1] = 0;
      mj_joint->axis[2] = 0;
    } else if (axis == pxr::UsdGeomTokens->y) {
      mj_joint->axis[0] = 0;
      mj_joint->axis[1] = 1;
      mj_joint->axis[2] = 0;
    } else {  // Z is default
      mj_joint->axis[0] = 0;
      mj_joint->axis[1] = 0;
      mj_joint->axis[2] = 1;
    }
    float lower, upper;
    if (prismatic.GetLowerLimitAttr().Get(&lower) &&
        prismatic.GetUpperLimitAttr().Get(&upper)) {
      mj_joint->limited = mjLIMITED_TRUE;
      mj_joint->range[0] = lower;
      mj_joint->range[1] = upper;
    }
  }

  // localPose1 is joint frame in child body (body1) coordinates.
  pxr::GfVec3f localPos1;
  joint.GetLocalPos1Attr().Get(&localPos1);
  pxr::GfVec3d pos(localPos1);

  pxr::GfQuatf localRot1;
  joint.GetLocalRot1Attr().Get(&localRot1);
  pxr::GfQuatd rot(localRot1);

  SetDoubleArrFromGfVec3d(mj_joint->pos, pos);

  // Transform joint axis by localPose1 rotation.
  if (mj_joint->type == mjJNT_HINGE || mj_joint->type == mjJNT_SLIDE) {
    pxr::GfVec3d axis_vec(mj_joint->axis[0], mj_joint->axis[1],
                          mj_joint->axis[2]);
    pxr::GfVec3d rotated_axis = rot.Transform(axis_vec);
    SetDoubleArrFromGfVec3d(mj_joint->axis, rotated_axis);
  }

  if (prim.HasAPI<pxr::MjcPhysicsActuatorAPI>()) {
    ParseMjcPhysicsGeneralActuatorAPI(spec, pxr::MjcPhysicsActuatorAPI(prim),
                                      mjtTrn::mjTRN_JOINT,
                                      mjs_getName(mj_joint->element));
  }

  if (prim.HasAPI<pxr::MjcPhysicsJointAPI>()) {
    ParseMjcPhysicsJointAPI(mj_joint, pxr::MjcPhysicsJointAPI(prim));
  }
}

void ParseMjcPhysicsSite(mjSpec* spec, const pxr::MjcPhysicsSiteAPI& site_api,
                         const pxr::UsdPrim& parent_prim, mjsBody* parent,
                         pxr::UsdGeomXformCache& xform_cache) {
  auto prim = site_api.GetPrim();
  mjsSite* site = mjs_addSite(parent, 0);
  mjs_setName(site->element,
              site_api.GetPrim().GetPath().GetAsString().c_str());
  SetLocalPoseFromPrim(site_api.GetPrim(), parent_prim, site, xform_cache);

  // Convert USD type to MuJoCo type.
  if (!MaybeParseGeomPrimitive(prim, site, xform_cache)) {
    mju_error("Prim with SiteAPI has unsupported typej %s",
              prim.GetTypeName().GetString().c_str());
    return;
  }

  if (prim.HasAPI<pxr::MjcPhysicsActuatorAPI>()) {
    auto act_api = pxr::MjcPhysicsActuatorAPI(prim);
    bool slider_crank = act_api.GetMjcSliderSiteRel().HasAuthoredTargets();
    ParseMjcPhysicsGeneralActuatorAPI(
        spec, pxr::MjcPhysicsActuatorAPI(prim),
        slider_crank ? mjtTrn::mjTRN_SLIDERCRANK : mjtTrn::mjTRN_SITE,
        mjs_getName(site->element));
  }
}

void ParseMjcPhysicsKeyframe(mjSpec* spec,
                             const pxr::MjcPhysicsKeyframe& keyframe) {
  auto prim = keyframe.GetPrim();
  auto qpos_attr = keyframe.GetMjcQposAttr();
  auto qvel_attr = keyframe.GetMjcQvelAttr();
  auto act_attr = keyframe.GetMjcActAttr();
  auto ctrl_attr = keyframe.GetMjcCtrlAttr();
  auto mpos_attr = keyframe.GetMjcMposAttr();
  auto mquat_attr = keyframe.GetMjcMquatAttr();

  auto setKeyframeData = [](mjsKey* key, const pxr::UsdAttribute& attr,
                            std::vector<double>** key_data,
                            double* time = nullptr) {
    if (attr.HasAuthoredValue()) {
      pxr::VtDoubleArray data;
      if (time == nullptr) {
        attr.Get(&data);
      } else {
        attr.Get(&data, *time);
      }
      *key_data = new std::vector<double>(data.begin(), data.end());
    }
  };

  size_t n_time_samples = 0;
  if (qpos_attr.HasAuthoredValue()) {
    n_time_samples = qpos_attr.GetNumTimeSamples();
  }

  if (n_time_samples == 0) {
    // If no time samples, we create a single keyframe.
    mjsKey* key = mjs_addKey(spec);
    mjs_setName(key->element, prim.GetName().GetString().c_str());
    setKeyframeData(key, qpos_attr, &key->qpos);
    setKeyframeData(key, qvel_attr, &key->qvel);
    setKeyframeData(key, act_attr, &key->act);
    setKeyframeData(key, ctrl_attr, &key->ctrl);
    setKeyframeData(key, mpos_attr, &key->mpos);
    setKeyframeData(key, mquat_attr, &key->mquat);
  } else {
    // If time samples, we create a keyframe for each time sample.
    std::vector<double> times;
    qpos_attr.GetTimeSamples(&times);
    int keyframe_id = 0;
    for (double time : times) {
      mjsKey* key = mjs_addKey(spec);
      std::string key_name =
          prim.GetName().GetString() + "_" + std::to_string(keyframe_id++);
      mjs_setName(key->element, key_name.c_str());
      key->time = time;
      setKeyframeData(key, qpos_attr, &key->qpos, &time);
      setKeyframeData(key, qvel_attr, &key->qvel, &time);
      setKeyframeData(key, act_attr, &key->act, &time);
      setKeyframeData(key, ctrl_attr, &key->ctrl, &time);
      setKeyframeData(key, mpos_attr, &key->mpos, &time);
      setKeyframeData(key, mquat_attr, &key->mquat, &time);
    }
  }
}

mjsBody* ParseUsdPhysicsRigidbody(
    mjSpec* spec, const pxr::UsdPhysicsRigidBodyAPI& rigidbody_api,
    const pxr::UsdPrim& parent_prim, mjsBody* parent,
    pxr::UsdGeomXformCache& xform_cache) {
  pxr::UsdPrim prim = rigidbody_api.GetPrim();
  mjsBody* body = mjs_addBody(parent, nullptr);
  mjs_setName(body->element, prim.GetPath().GetAsString().c_str());
  SetLocalPoseFromPrim(prim, parent_prim, body, xform_cache);

  if (prim.HasAPI<pxr::UsdPhysicsMassAPI>()) {
    ParseUsdPhysicsMassAPIForBody(body, pxr::UsdPhysicsMassAPI(prim));
  }

  if (prim.HasAPI<pxr::MjcPhysicsActuatorAPI>()) {
    ParseMjcPhysicsGeneralActuatorAPI(spec, pxr::MjcPhysicsActuatorAPI(prim),
                                      mjtTrn::mjTRN_BODY,
                                      mjs_getName(body->element));
  }

  mujoco::usd::SetUsdPrimPathUserValue(body->element, prim.GetPath());

  return body;
}

// There is no common base class for UsdPhysicsRigidBodyAPI and
// UsdPhysicsCollisionAPI so we need a templated function.
template <class T>
bool IsObjectInPhysicsScene(const T& object,
                            const pxr::UsdPhysicsScene& physics_scene) {
  pxr::SdfPathVector sim_owners;
  object.GetSimulationOwnerRel().GetTargets(&sim_owners);
  for (const auto& sim_owner : sim_owners) {
    if (physics_scene.GetPath() == sim_owner) {
      return true;
    }
  }
  return false;
};

// Helper type to store all the prims that belong to a body.
using BodyPrimMap = std::map<pxr::SdfPath, std::vector<pxr::SdfPath>>;

// Recursively traverses the kinematic tree, creating bodies, joints, and geoms
// in the mjSpec.
void PopulateSpecFromTree(pxr::UsdStageRefPtr stage, mjSpec* spec,
                          mjsBody* parent_mj_body,
                          const mujoco::usd::KinematicNode* parent_node,
                          const mujoco::usd::KinematicNode& current_node,
                          pxr::UsdGeomXformCache& xform_cache,
                          const BodyPrimMap& body_to_prims) {
  mjsBody* current_mj_body;

  if (current_node.body_path.IsEmpty()) {
    // This is the world root node.
    current_mj_body = mjs_findBody(spec, "world");
  } else {
    // This is a regular body.
    pxr::UsdPrim current_body_prim =
        stage->GetPrimAtPath(current_node.body_path);
    pxr::SdfPath parent_body_path =
        parent_node ? parent_node->body_path : pxr::SdfPath();
    pxr::UsdPrim parent_prim_for_xform =
        parent_body_path.IsEmpty() ? stage->GetPseudoRoot()
                                   : stage->GetPrimAtPath(parent_body_path);

    current_mj_body = ParseUsdPhysicsRigidbody(
        spec, pxr::UsdPhysicsRigidBodyAPI(current_body_prim),
        parent_prim_for_xform, parent_mj_body, xform_cache);

    if (!current_node.joint_path.IsEmpty()) {
      pxr::UsdPrim joint_prim = stage->GetPrimAtPath(current_node.joint_path);
      ParseUsdPhysicsJoint(spec, joint_prim, current_mj_body, xform_cache);
    } else if (parent_mj_body == mjs_findBody(spec, "world")) {
      // No joint to parent, and parent is world: this is a floating body.
      mjsJoint* free_joint = mjs_addJoint(current_mj_body, nullptr);
      free_joint->type = mjJNT_FREE;
    }
  }

  // Add geoms/sites/etc. belonging to the current body.
  auto it_prims = body_to_prims.find(current_node.body_path);
  if (it_prims != body_to_prims.end()) {
    pxr::UsdPrim body_prim_for_xform =
        current_node.body_path.IsEmpty()
            ? stage->GetPseudoRoot()
            : stage->GetPrimAtPath(current_node.body_path);
    for (const auto& gprim_path : it_prims->second) {
      pxr::UsdPrim prim = stage->GetPrimAtPath(gprim_path);
      if (prim.HasAPI<pxr::UsdPhysicsCollisionAPI>()) {
        ParseUsdPhysicsCollider(spec, pxr::UsdPhysicsCollisionAPI(prim),
                                body_prim_for_xform, current_mj_body,
                                xform_cache);
      }
      if (prim.HasAPI<pxr::MjcPhysicsSiteAPI>()) {
        ParseMjcPhysicsSite(spec, pxr::MjcPhysicsSiteAPI(prim),
                            body_prim_for_xform, current_mj_body, xform_cache);
      }
    }
  }

  // Recurse through children.
  for (const auto& child_node : current_node.children) {
    PopulateSpecFromTree(stage, spec, current_mj_body, &current_node,
                         *child_node, xform_cache, body_to_prims);
  }
}
}  // namespace

mjSpec* mj_parseUSDStage(const pxr::UsdStageRefPtr stage) {
  mjSpec* spec = mj_makeSpec();

  std::vector<pxr::UsdPhysicsScene> physics_scenes;

  // Xform cache to use for all queries when parsing.
  pxr::UsdGeomXformCache xform_cache;

  // Search for UsdPhysicsScene type prim, use the first one that has
  // the MjcPhysicsSceneAPI applied or the first UsdPhysicsScene otherwise.
  std::optional<pxr::UsdPhysicsScene> physics_scene;
  for (auto prim : stage->Traverse()) {
    if (prim.IsA<pxr::UsdPhysicsScene>()) {
      bool has_mjc_physics_api = prim.HasAPI<pxr::MjcPhysicsSceneAPI>();
      if (!physics_scene.has_value() || has_mjc_physics_api) {
        physics_scene = pxr::UsdPhysicsScene(prim);
        // If we've found the first scene with MjcPhysicsSceneAPI, we can stop
        // searching.
        if (has_mjc_physics_api) {
          break;
        }
      }
    }
  }

  if (physics_scene.has_value()) {
    ParseUsdPhysicsScene(spec, *physics_scene);
  }

  pxr::SdfPath default_prim_path;
  if (stage->GetDefaultPrim().IsValid()) {
    default_prim_path = stage->GetDefaultPrim().GetPath();
  }

  // Data Structures
  std::vector<pxr::UsdPhysicsJoint> all_joints;
  std::vector<pxr::SdfPath> all_body_paths_vec;
  BodyPrimMap body_to_prims;

  // =========================================================================
  // PASS 1: Collect Bodies, Joints, and Geoms/Sites/etc.
  // =========================================================================
  // A single DFS pass to find all bodies, joints, and determine
  // which body owns each geom/site/etc. prim.
  std::vector<pxr::SdfPath> owner_stack;
  owner_stack.push_back(pxr::SdfPath());  // Start with the world as owner.

  const auto range = pxr::UsdPrimRange::PreAndPostVisit(
      stage->GetPseudoRoot(), pxr::UsdTraverseInstanceProxies());

  for (auto it = range.begin(); it != range.end(); ++it) {
    pxr::UsdPrim prim = *it;

    bool is_body = prim.HasAPI<pxr::UsdPhysicsRigidBodyAPI>();
    bool resets = xform_cache.GetResetXformStack(prim);
    // Only update (push/pop) the owner stack for bodies (becomes new owner) and
    // resetXformStack (reset owner to world).
    bool is_pushed_to_stack = is_body || resets;

    if (it.IsPostVisit()) {
      if (is_pushed_to_stack) {
        owner_stack.pop_back();
      }
      continue;
    }

    pxr::SdfPath prim_path = prim.GetPath();
    pxr::SdfPath prim_owner = owner_stack.back();

    if (is_body) {
      all_body_paths_vec.push_back(prim_path);
      prim_owner = prim_path;
    } else if (resets) {
      prim_owner = pxr::SdfPath();  // Reset owner to world.
    }

    if (is_pushed_to_stack) {
      owner_stack.push_back(prim_owner);
    }

    if (prim.HasAPI<pxr::UsdPhysicsCollisionAPI>() ||
        prim.HasAPI<pxr::MjcPhysicsSiteAPI>()) {
      body_to_prims[prim_owner].push_back(prim_path);
    }

    if (prim.IsA<pxr::UsdPhysicsJoint>()) {
      all_joints.push_back(pxr::UsdPhysicsJoint(prim));

      it.PruneChildren();
    } else if (prim.IsA<pxr::MjcPhysicsKeyframe>()) {
      ParseMjcPhysicsKeyframe(spec, pxr::MjcPhysicsKeyframe(prim));

      it.PruneChildren();
    }
  }

  // =========================================================================
  // PASS 2: Build the kinematic tree and populate the mjSpec.
  // =========================================================================
  std::unique_ptr<mujoco::usd::KinematicNode> kinematic_tree =
      mujoco::usd::BuildKinematicTree(all_joints, all_body_paths_vec,
                                      default_prim_path);

  if (kinematic_tree) {
    PopulateSpecFromTree(stage, spec, /*parent_mj_body=*/nullptr,
                         /*parent_node=*/nullptr, *kinematic_tree, xform_cache,
                         body_to_prims);
  }

  return spec;
}

mjSpec* mj_parseUSDStage(const char* usd_path) {
  pxr::UsdStageRefPtr stage = pxr::UsdStage::Open(usd_path);
  if (!stage) {
    mju_error("Could not open USD stage: %s", usd_path);
    return nullptr;
  }
  return mj_parseUSDStage(stage);
}
