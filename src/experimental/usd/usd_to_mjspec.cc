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

#include <algorithm>
#include <cmath>
#include <deque>
#include <iterator>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <mujoco/experimental/usd/mjcPhysics/actuatorAPI.h>
#include <mujoco/experimental/usd/mjcPhysics/collisionAPI.h>
#include <mujoco/experimental/usd/mjcPhysics/meshCollisionAPI.h>
#include <mujoco/experimental/usd/mjcPhysics/sceneAPI.h>
#include <mujoco/experimental/usd/mjcPhysics/siteAPI.h>
#include <mujoco/experimental/usd/mjcPhysics/tokens.h>
#include <mujoco/experimental/usd/usd.h>
#include <mujoco/experimental/usd/utils.h>
#include <mujoco/mujoco.h>
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
#include <pxr/usd/usdPhysics/sphericalJoint.h>
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
  mjs_setString(mj_act->name, prim.GetPath().GetAsString().c_str());
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

void ParseUsdPhysicsCollider(mjSpec* spec,
                             const pxr::UsdPhysicsCollisionAPI& collision_api,
                             const pxr::UsdPrim& parent_prim, mjsBody* parent,
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
  mjs_setString(geom->name, prim.GetPath().GetAsString().c_str());
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

  SetLocalPoseFromPrim(prim, parent_prim, geom, xform_cache);

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
      mjs_setString(mesh->name, mesh_name.c_str());
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
  mjs_setString(mj_joint->name, prim.GetPath().GetAsString().c_str());

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
                                      mjtTrn::mjTRN_JOINT, mj_joint->name);
  }
}

void ParseMjcPhysicsSite(mjSpec* spec, const pxr::MjcPhysicsSiteAPI& site_api,
                         const pxr::UsdPrim& parent_prim, mjsBody* parent,
                         pxr::UsdGeomXformCache& xform_cache) {
  auto prim = site_api.GetPrim();
  mjsSite* site = mjs_addSite(parent, 0);
  mjs_setString(site->name, site_api.GetPrim().GetPath().GetAsString().c_str());
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
        site->name);
  }
}

mjsBody* ParseUsdPhysicsRigidbody(
    mjSpec* spec, const pxr::UsdPhysicsRigidBodyAPI& rigidbody_api,
    const pxr::UsdPrim& parent_prim, mjsBody* parent,
    pxr::UsdGeomXformCache& xform_cache) {
  pxr::UsdPrim prim = rigidbody_api.GetPrim();
  mjsBody* body = mjs_addBody(parent, nullptr);
  mjs_setString(body->name, prim.GetPath().GetAsString().c_str());
  SetLocalPoseFromPrim(prim, parent_prim, body, xform_cache);

  if (prim.HasAPI<pxr::UsdPhysicsMassAPI>()) {
    ParseUsdPhysicsMassAPIForBody(body, pxr::UsdPhysicsMassAPI(prim));
  }

  if (prim.HasAPI<pxr::MjcPhysicsActuatorAPI>()) {
    ParseMjcPhysicsGeneralActuatorAPI(spec, pxr::MjcPhysicsActuatorAPI(prim),
                                      mjtTrn::mjTRN_BODY, body->name);
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

// A struct to hold the forest representation.
// The forest is a map from a root path to its tree.
// A tree is an adjacency list, mapping a parent path to its children paths.
using MjUsdForest =
    std::map<pxr::SdfPath, std::map<pxr::SdfPath, std::vector<pxr::SdfPath>>>;

// A directed edge.
using Edge = std::pair<pxr::SdfPath, pxr::SdfPath>;

// A map from a directed edge to the path of the joint representing that edge.
using EdgesMap = std::map<Edge, pxr::SdfPath>;

// Constructs and validates a forest (a collection of disjoint trees)
// from a list of directed edges. It also considers all rigid bodies in the
// scene, treating those not involved in any edge as isolated, free-floating
// bodies.
//
// An empty 'from' path represents the world body. Returns `std::nullopt` for
// invalid forest structures (e.g. cycles, multiple parents).
std::optional<MjUsdForest> BuildForestFromEdges(
    const EdgesMap& edges, const std::vector<pxr::SdfPath>& all_body_paths) {
  if (edges.empty() && all_body_paths.empty()) {
    return MjUsdForest{};
  }

  std::map<pxr::SdfPath, std::vector<pxr::SdfPath>> children_map;
  std::map<pxr::SdfPath, pxr::SdfPath> parent_map;
  std::set<pxr::SdfPath> all_nodes(all_body_paths.begin(),
                                   all_body_paths.end());

  // 1. Initial Pass: Build maps and perform local validation
  for (const auto& edge_pair : edges) {
    const auto& [from, to] = edge_pair.first;
    if (from == to) {
      mju_error("Self-loop detected at node %s", to.GetString().c_str());
      return std::nullopt;
    }
    if (parent_map.count(to)) {
      mju_error("Node %s has multiple parents ('%s' and '%s').",
                to.GetString().c_str(), parent_map.at(to).GetString().c_str(),
                from.GetString().c_str());
      return std::nullopt;
    }

    children_map[from].push_back(to);
    parent_map[to] = from;
    all_nodes.insert(from);
    all_nodes.insert(to);
  }

  // 2. Find all root nodes
  // A root is a node that is not a child of any other node. This includes
  // roots of kinematic trees and isolated rigid bodies.
  std::set<pxr::SdfPath> roots = all_nodes;
  for (const auto& pair : parent_map) {
    roots.erase(pair.first);  // `pair.first` is a child node.
  }

  if (roots.empty() && !all_nodes.empty()) {
    mju_error("No root nodes found, but edges exist. A cycle is present.");
    return std::nullopt;
  }

  // 3. Build the forest, claiming nodes for each tree
  MjUsdForest forest;
  std::set<pxr::SdfPath> claimed_nodes;

  for (const auto& root : roots) {
    // Perform a traversal (BFS) to find all nodes in this tree
    std::deque<pxr::SdfPath> q;
    q.push_back(root);
    std::set<pxr::SdfPath> nodes_in_this_tree;

    while (!q.empty()) {
      pxr::SdfPath current_node = q.front();
      q.pop_front();

      if (claimed_nodes.count(current_node)) {
        mju_error("Node %s is shared between multiple trees.",
                  current_node.GetString().c_str());
        return std::nullopt;
      }

      nodes_in_this_tree.insert(current_node);
      claimed_nodes.insert(current_node);

      if (children_map.count(current_node)) {
        for (const auto& child : children_map.at(current_node)) {
          q.push_back(child);
        }
      }
    }

    // Construct the adjacency list for this specific tree
    std::map<pxr::SdfPath, std::vector<pxr::SdfPath>> tree_adj_list;
    for (const auto& node : nodes_in_this_tree) {
      if (children_map.count(node)) {
        tree_adj_list[node] = children_map.at(node);
      }
    }
    forest[root] = tree_adj_list;
  }

  // 4. Final check for cycles (unclaimed nodes)
  std::set<pxr::SdfPath> unclaimed_nodes;  // all_nodes - claimed_nodes
  std::set_difference(all_nodes.begin(), all_nodes.end(), claimed_nodes.begin(),
                      claimed_nodes.end(),
                      std::inserter(unclaimed_nodes, unclaimed_nodes.begin()));

  if (!unclaimed_nodes.empty()) {
    std::string unclaimed_str;
    for (const auto& node : unclaimed_nodes) {
      unclaimed_str += "'" + node.GetString() + "' ";
    }
    mju_error(
        "Cycle detected. The following nodes are part of a cycle "
        "and not reachable from any root: %s",
        unclaimed_str.c_str());
    return std::nullopt;
  }

  return forest;
}

void TraverseAndBuildTree(
    pxr::UsdStageRefPtr stage, mjSpec* spec, mjsBody* parent_mj_body,
    const pxr::SdfPath& parent_body_path, const pxr::SdfPath& current_body_path,
    const std::map<pxr::SdfPath, std::vector<pxr::SdfPath>>& tree,
    const EdgesMap& edges, pxr::UsdGeomXformCache& xform_cache);

// Traverses the prim and all its descendants in the USD hierarchy and parses
// supported entities like colliders and sites, attaching them to the given
// mjBody. The traversal for a given branch stops when:
// - a descendant with a RigidBodyAPI is found, as that will be handled by
// TraverseAndBuildTree.
// - a descendant with a ResetXformStack is found, as that will be handled by
// the top level traversal of independent prims.
void ParseCurrentAndDescendants(mjSpec* spec, const pxr::UsdPrim& prim,
                                const pxr::UsdPrim& parent_prim, mjsBody* body,
                                pxr::UsdGeomXformCache& xform_cache) {
  if (prim.HasAPI<pxr::UsdPhysicsCollisionAPI>()) {
    ParseUsdPhysicsCollider(spec, pxr::UsdPhysicsCollisionAPI(prim),
                            parent_prim, body, xform_cache);
  }
  if (prim.HasAPI<pxr::MjcPhysicsSiteAPI>()) {
    ParseMjcPhysicsSite(spec, pxr::MjcPhysicsSiteAPI(prim), parent_prim, body,
                        xform_cache);
  }

  // Make sure we traverse into instance proxies to ensure we support
  // instanceable references.
  // See https://openusd.org/dev/api/_usd__page__scenegraph_instancing.html
  for (const auto& child :
       prim.GetFilteredChildren(pxr::UsdTraverseInstanceProxies())) {
    if (child.HasAPI<pxr::UsdPhysicsRigidBodyAPI>()) {
      continue;
    }
    if (xform_cache.GetResetXformStack(prim)) {
      continue;
    }
    ParseCurrentAndDescendants(spec, child, prim, body, xform_cache);
  }
}

// Recursively traverses a kinematic tree, creating bodies and joints in the
// mjSpec.
void TraverseAndBuildTree(
    pxr::UsdStageRefPtr stage, mjSpec* spec, mjsBody* parent_mj_body,
    const pxr::SdfPath& parent_body_path, const pxr::SdfPath& current_body_path,
    const std::map<pxr::SdfPath, std::vector<pxr::SdfPath>>& tree,
    const EdgesMap& edges, pxr::UsdGeomXformCache& xform_cache) {
  pxr::UsdPrim current_body_prim = stage->GetPrimAtPath(current_body_path);
  pxr::UsdPrim parent_prim_for_xform =
      parent_body_path.IsEmpty() ? stage->GetPseudoRoot()
                                 : stage->GetPrimAtPath(parent_body_path);

  mjsBody* current_mj_body = ParseUsdPhysicsRigidbody(
      spec, pxr::UsdPhysicsRigidBodyAPI(current_body_prim),
      parent_prim_for_xform, parent_mj_body, xform_cache);

  auto edge_key = std::make_pair(parent_body_path, current_body_path);
  auto it_edge = edges.find(edge_key);

  if (it_edge != edges.end()) {
    // An edge exists, indicating a connection to the parent. This body is
    // either world-attached or part of a larger articulation.
    const pxr::SdfPath& joint_path = it_edge->second;
    if (!joint_path.IsEmpty()) {
      // An explicit joint prim exists, so we parse it.
      pxr::UsdPrim joint_prim = stage->GetPrimAtPath(joint_path);
      ParseUsdPhysicsJoint(spec, joint_prim, current_mj_body, xform_cache);
    }
    // If joint_path is empty, it's an implicit fixed joint. No joint is created
    // in the mjSpec, effectively welding the body to its parent.
  } else {
    // No edge found. This condition is met for the root of a floating-base
    // tree, which has no defined joint connecting it to the world.
    if (parent_mj_body == mjs_findBody(spec, "world")) {
      // We explicitly create a free joint to make it a floating-base body.
      mjsJoint* free_joint = mjs_addJoint(current_mj_body, nullptr);
      free_joint->type = mjJNT_FREE;
    }
  }

  // Parse all geoms/sites that are found on this body in the USD hierarchy.
  ParseCurrentAndDescendants(spec, current_body_prim, parent_prim_for_xform,
                             current_mj_body, xform_cache);
  // Recurse through the kinematic tree.
  auto it_tree = tree.find(current_body_path);
  if (it_tree != tree.end()) {
    const auto& children_paths = it_tree->second;
    for (const auto& child_path : children_paths) {
      TraverseAndBuildTree(stage, spec, current_mj_body, current_body_path,
                           child_path, tree, edges, xform_cache);
    }
  }
}

// Adds a new edge to the edges map, dealing with duplicates:
// - explicit joints always replace implicit joints
// - more than one explicit joint is unsupported so we print a warning and
// keep the first one found
void AddEdge(EdgesMap& edges, const pxr::SdfPath& from, const pxr::SdfPath& to,
             const pxr::SdfPath& joint) {
  auto edge_key = std::make_pair(from, to);
  auto it = edges.find(edge_key);

  if (it == edges.end()) {
    // No existing edge, add the new one.
    edges[edge_key] = joint;
  } else {
    // Edge already exists.
    pxr::SdfPath& existing_joint = it->second;
    bool new_is_explicit = !joint.IsEmpty();
    bool existing_is_explicit = !existing_joint.IsEmpty();

    if (new_is_explicit) {
      if (existing_is_explicit) {
        // Both are explicit: this is an error condition.
        mju_warning(
            "Multiple explicit joints defined between body %s and body %s. "
            "Joint1: %s, Joint2: %s. Keeping the first one found: %s",
            (from.IsEmpty() ? "<worldbody>" : from.GetString()).c_str(),
            to.GetString().c_str(), existing_joint.GetString().c_str(),
            joint.GetString().c_str(), existing_joint.GetString().c_str());

      } else {
        // New is explicit, existing is implicit: replace.
        existing_joint = joint;
      }
    }
    // If new is implicit, and an edge already exists (either explicit or
    // implicit), we keep the existing one. No action needed.
  }
}

// Returns the nesting body prim (or an invalid prim if there's
// any resets_xform_stack, or we've reached the end).
pxr::UsdPrim GetNestingBodyPrim(const pxr::UsdPrim& prim,
                                pxr::UsdGeomXformCache& xform_cache) {
  if (xform_cache.GetResetXformStack(prim)) {
    return pxr::UsdPrim();
  }
  pxr::UsdPrim previous_prim = prim.GetParent();
  while (previous_prim.IsValid()) {
    if (xform_cache.GetResetXformStack(previous_prim)) {
      return pxr::UsdPrim();
    }
    if (previous_prim.HasAPI<pxr::UsdPhysicsRigidBodyAPI>()) {
      return previous_prim;
    }
    previous_prim = previous_prim.GetParent();
  }
  return pxr::UsdPrim();
}
}  // namespace

mjSpec* mj_parseUSDStage(const pxr::UsdStageRefPtr stage) {
  mjSpec* spec = mj_makeSpec();

  mjsBody* world = mjs_findBody(spec, "world");

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

  std::vector<pxr::SdfPath> body_paths;
  EdgesMap edges;
  // TODO(robinalazard): Re-introduce properly adding objects to their
  // respective physics scene, or default.

  // Traverse all prims under the pseudo-root.
  // We ensure to traverse into instance proxies to ensure we support
  // instanceable references.
  // See https://openusd.org/dev/api/_usd__page__scenegraph_instancing.html
  for (auto prim : stage->Traverse(pxr::UsdTraverseInstanceProxies())) {
    // When traversing the whole scene, if we encounter a rigidbody or a joint,
    // then we populate the edges map and the list of body paths, which will be
    // processed later to build the articulation trees.
    //
    // If we encounter _anything else_:
    // - if we find that it's an independent prim (e.g. a static collider or
    // site) not belonging to any rigidbody, then we add it directly to the
    // world.
    // - otherwise, they will be handled when building the articulation trees.

    if (prim.IsA<pxr::UsdPhysicsJoint>()) {
      pxr::SdfPath joint_path = prim.GetPath();
      pxr::UsdPhysicsJoint joint(prim);

      pxr::SdfPath body1_path;
      pxr::SdfPathVector body1_paths;
      joint.GetBody1Rel().GetTargets(&body1_paths);
      if (body1_paths.empty()) {
        mju_warning("Joint %s does not have body1 rel. Skipping.",
                    prim.GetPath().GetAsString().c_str());
        continue;
      } else if (body1_paths.size() > 1) {
        mju_warning("Joint %s has multiple body1 rels. Skipping.",
                    prim.GetPath().GetAsString().c_str());
        continue;
      }
      body1_path = body1_paths[0];

      pxr::SdfPath body0_path;
      pxr::SdfPathVector body0_paths;
      joint.GetBody0Rel().GetTargets(&body0_paths);
      if (body0_paths.size() > 1) {
        mju_warning("Joint %s has multiple body0 rels. Skipping.",
                    prim.GetPath().GetAsString().c_str());
        continue;
      }
      // Empty body0, or body0 pointing to the default prim means we'll attach
      // to the worldbody.
      if (body0_paths.empty() || body0_paths[0] == default_prim_path) {
        body0_path = pxr::SdfPath();
      } else {
        body0_path = body0_paths[0];
      }

      AddEdge(edges, body0_path, body1_path, joint_path);
    } else if (prim.HasAPI<pxr::UsdPhysicsRigidBodyAPI>()) {
      pxr::SdfPath body_path = prim.GetPath();
      body_paths.push_back(body_path);

      // Find whether we are nested under a parent body.
      // Note: if any xform in between (including the current prim) resets the
      // xform stack then we are not nested.
      pxr::UsdPrim nesting_body_prim = GetNestingBodyPrim(prim, xform_cache);
      if (nesting_body_prim.IsValid()) {
        AddEdge(edges, nesting_body_prim.GetPath(), body_path, pxr::SdfPath());
      }
    } else {
      // TODO(robinalazard): the way we handle independent prims right now means
      // their relative transforms will be ignored be always pass
      // stage->GetPseudoRoot() as the parent. It works for most scene
      // realistically. But we should fix it.

      if (prim.HasAPI<pxr::UsdPhysicsCollisionAPI>()) {
        // Find whether the collider belongs to a body. If yes, it will be we
        // handled later when building the articulation trees. Otherwise, it's a
        // static collider and we add it directly to the world.
        pxr::UsdPrim nesting_body_prim = GetNestingBodyPrim(prim, xform_cache);
        if (!nesting_body_prim.IsValid()) {
          ParseUsdPhysicsCollider(spec, pxr::UsdPhysicsCollisionAPI(prim),
                                  stage->GetPseudoRoot(), world, xform_cache);
        }
      }
      if (prim.HasAPI<pxr::MjcPhysicsSiteAPI>()) {
        // Find whether the site belongs to a body. If yes, it will be we
        // handled later when building the articulation trees. Otherwise, it's a
        // static site and we add it directly to the world.
        pxr::UsdPrim nesting_body_prim = GetNestingBodyPrim(prim, xform_cache);
        if (!nesting_body_prim.IsValid()) {
          ParseMjcPhysicsSite(spec, pxr::MjcPhysicsSiteAPI(prim),
                              stage->GetPseudoRoot(), world, xform_cache);
        }
      }
    }
  }

  std::optional<MjUsdForest> forest = BuildForestFromEdges(edges, body_paths);
  if (forest.has_value()) {
    // Now that we have the forest, we can walk through it and add the bodies
    // and joints to the spec. From each body we can also visit and parse all
    // the children to also add the corresponding colliders to the body in the
    // spec.
    for (const auto& [root_path, tree] : *forest) {
      if (root_path.IsEmpty()) {
        // This root is the world. This case handles all kinematic trees that
        // are attached to the world (fixed-base articulations).
        // We iterate through its direct children which are the root bodies of
        // each world-attached tree. We iterate through these children and begin
        // the recursive build from there.
        const auto& children_of_world = tree.at(pxr::SdfPath());
        for (const auto& child_path : children_of_world) {
          TraverseAndBuildTree(stage, spec, world, root_path, child_path, tree,
                               edges, xform_cache);
        }
      } else {
        // Conversely, this case handles all the remaining top-level root bodies
        // which are their own roots and are not attached to the world
        // (floating-base articulations). This includes isolated bodies.
        // We directly begin the recursive build from the toplevel root.
        //
        // Note: the absence of an edge in the `edges` map connecting the world
        // to this root is what signals to `TraverseAndBuildTree` that this is a
        // floating base, prompting the creation of a free joint.
        TraverseAndBuildTree(stage, spec, world, pxr::SdfPath(), root_path,
                             tree, edges, xform_cache);
      }
    }
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
