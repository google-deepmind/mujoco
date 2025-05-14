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

// GENERATED FILE.  DO NOT EDIT.
#include <boost/python/class.hpp>

#include "./tokens.h"

PXR_NAMESPACE_USING_DIRECTIVE

namespace {

// Helper to return a static token as a string.  We wrap tokens as Python
// strings and for some reason simply wrapping the token using def_readonly
// bypasses to-Python conversion, leading to the error that there's no
// Python type for the C++ TfToken type.  So we wrap this functor instead.
class _WrapStaticToken {
 public:
  _WrapStaticToken(const TfToken* token) : _token(token) {}

  std::string operator()() const { return _token->GetString(); }

 private:
  const TfToken* _token;
};

template <typename T>
void _AddToken(T& cls, const char* name, const TfToken& token) {
  cls.add_static_property(
      name,
      boost::python::make_function(
          _WrapStaticToken(&token),
          boost::python::return_value_policy<boost::python::return_by_value>(),
          boost::mpl::vector1<std::string>()));
}

}  // namespace

void wrapMjcPhysicsTokens() {
  boost::python::class_<MjcPhysicsTokensType, boost::noncopyable> cls(
      "Tokens", boost::python::no_init);
  _AddToken(cls, "auto_", MjcPhysicsTokens->auto_);
  _AddToken(cls, "cg", MjcPhysicsTokens->cg);
  _AddToken(cls, "dense", MjcPhysicsTokens->dense);
  _AddToken(cls, "elliptic", MjcPhysicsTokens->elliptic);
  _AddToken(cls, "euler", MjcPhysicsTokens->euler);
  _AddToken(cls, "implicit", MjcPhysicsTokens->implicit);
  _AddToken(cls, "implicitfast", MjcPhysicsTokens->implicitfast);
  _AddToken(cls, "mjcPhysicsActuatorgroupdisable",
            MjcPhysicsTokens->mjcPhysicsActuatorgroupdisable);
  _AddToken(cls, "mjcPhysicsApirate", MjcPhysicsTokens->mjcPhysicsApirate);
  _AddToken(cls, "mjcPhysicsCcd_iterations",
            MjcPhysicsTokens->mjcPhysicsCcd_iterations);
  _AddToken(cls, "mjcPhysicsCcd_tolerance",
            MjcPhysicsTokens->mjcPhysicsCcd_tolerance);
  _AddToken(cls, "mjcPhysicsCone", MjcPhysicsTokens->mjcPhysicsCone);
  _AddToken(cls, "mjcPhysicsDensity", MjcPhysicsTokens->mjcPhysicsDensity);
  _AddToken(cls, "mjcPhysicsFlagActuation",
            MjcPhysicsTokens->mjcPhysicsFlagActuation);
  _AddToken(cls, "mjcPhysicsFlagAutoreset",
            MjcPhysicsTokens->mjcPhysicsFlagAutoreset);
  _AddToken(cls, "mjcPhysicsFlagClampctrl",
            MjcPhysicsTokens->mjcPhysicsFlagClampctrl);
  _AddToken(cls, "mjcPhysicsFlagConstraint",
            MjcPhysicsTokens->mjcPhysicsFlagConstraint);
  _AddToken(cls, "mjcPhysicsFlagContact",
            MjcPhysicsTokens->mjcPhysicsFlagContact);
  _AddToken(cls, "mjcPhysicsFlagEnergy",
            MjcPhysicsTokens->mjcPhysicsFlagEnergy);
  _AddToken(cls, "mjcPhysicsFlagEquality",
            MjcPhysicsTokens->mjcPhysicsFlagEquality);
  _AddToken(cls, "mjcPhysicsFlagEulerdamp",
            MjcPhysicsTokens->mjcPhysicsFlagEulerdamp);
  _AddToken(cls, "mjcPhysicsFlagFilterparent",
            MjcPhysicsTokens->mjcPhysicsFlagFilterparent);
  _AddToken(cls, "mjcPhysicsFlagFrictionloss",
            MjcPhysicsTokens->mjcPhysicsFlagFrictionloss);
  _AddToken(cls, "mjcPhysicsFlagFwdinv",
            MjcPhysicsTokens->mjcPhysicsFlagFwdinv);
  _AddToken(cls, "mjcPhysicsFlagGravity",
            MjcPhysicsTokens->mjcPhysicsFlagGravity);
  _AddToken(cls, "mjcPhysicsFlagInvdiscrete",
            MjcPhysicsTokens->mjcPhysicsFlagInvdiscrete);
  _AddToken(cls, "mjcPhysicsFlagIsland",
            MjcPhysicsTokens->mjcPhysicsFlagIsland);
  _AddToken(cls, "mjcPhysicsFlagLimit", MjcPhysicsTokens->mjcPhysicsFlagLimit);
  _AddToken(cls, "mjcPhysicsFlagMidphase",
            MjcPhysicsTokens->mjcPhysicsFlagMidphase);
  _AddToken(cls, "mjcPhysicsFlagMulticcd",
            MjcPhysicsTokens->mjcPhysicsFlagMulticcd);
  _AddToken(cls, "mjcPhysicsFlagNativeccd",
            MjcPhysicsTokens->mjcPhysicsFlagNativeccd);
  _AddToken(cls, "mjcPhysicsFlagOverride",
            MjcPhysicsTokens->mjcPhysicsFlagOverride);
  _AddToken(cls, "mjcPhysicsFlagPassive",
            MjcPhysicsTokens->mjcPhysicsFlagPassive);
  _AddToken(cls, "mjcPhysicsFlagRefsafe",
            MjcPhysicsTokens->mjcPhysicsFlagRefsafe);
  _AddToken(cls, "mjcPhysicsFlagSensor",
            MjcPhysicsTokens->mjcPhysicsFlagSensor);
  _AddToken(cls, "mjcPhysicsFlagWarmstart",
            MjcPhysicsTokens->mjcPhysicsFlagWarmstart);
  _AddToken(cls, "mjcPhysicsImpratio", MjcPhysicsTokens->mjcPhysicsImpratio);
  _AddToken(cls, "mjcPhysicsIntegrator",
            MjcPhysicsTokens->mjcPhysicsIntegrator);
  _AddToken(cls, "mjcPhysicsIterations",
            MjcPhysicsTokens->mjcPhysicsIterations);
  _AddToken(cls, "mjcPhysicsJacobian", MjcPhysicsTokens->mjcPhysicsJacobian);
  _AddToken(cls, "mjcPhysicsLs_iterations",
            MjcPhysicsTokens->mjcPhysicsLs_iterations);
  _AddToken(cls, "mjcPhysicsLs_tolerance",
            MjcPhysicsTokens->mjcPhysicsLs_tolerance);
  _AddToken(cls, "mjcPhysicsMagnetic", MjcPhysicsTokens->mjcPhysicsMagnetic);
  _AddToken(cls, "mjcPhysicsNoslip_iterations",
            MjcPhysicsTokens->mjcPhysicsNoslip_iterations);
  _AddToken(cls, "mjcPhysicsNoslip_tolerance",
            MjcPhysicsTokens->mjcPhysicsNoslip_tolerance);
  _AddToken(cls, "mjcPhysicsO_friction",
            MjcPhysicsTokens->mjcPhysicsO_friction);
  _AddToken(cls, "mjcPhysicsO_margin", MjcPhysicsTokens->mjcPhysicsO_margin);
  _AddToken(cls, "mjcPhysicsO_solimp", MjcPhysicsTokens->mjcPhysicsO_solimp);
  _AddToken(cls, "mjcPhysicsO_solref", MjcPhysicsTokens->mjcPhysicsO_solref);
  _AddToken(cls, "mjcPhysicsSdf_initpoints",
            MjcPhysicsTokens->mjcPhysicsSdf_initpoints);
  _AddToken(cls, "mjcPhysicsSdf_iterations",
            MjcPhysicsTokens->mjcPhysicsSdf_iterations);
  _AddToken(cls, "mjcPhysicsSolver", MjcPhysicsTokens->mjcPhysicsSolver);
  _AddToken(cls, "mjcPhysicsTimestep", MjcPhysicsTokens->mjcPhysicsTimestep);
  _AddToken(cls, "mjcPhysicsTolerance", MjcPhysicsTokens->mjcPhysicsTolerance);
  _AddToken(cls, "mjcPhysicsViscosity", MjcPhysicsTokens->mjcPhysicsViscosity);
  _AddToken(cls, "mjcPhysicsWind", MjcPhysicsTokens->mjcPhysicsWind);
  _AddToken(cls, "newton", MjcPhysicsTokens->newton);
  _AddToken(cls, "pgs", MjcPhysicsTokens->pgs);
  _AddToken(cls, "pyramidal", MjcPhysicsTokens->pyramidal);
  _AddToken(cls, "rk4", MjcPhysicsTokens->rk4);
  _AddToken(cls, "sparse", MjcPhysicsTokens->sparse);
  _AddToken(cls, "SceneAPI", MjcPhysicsTokens->SceneAPI);
  _AddToken(cls, "SiteAPI", MjcPhysicsTokens->SiteAPI);
}
