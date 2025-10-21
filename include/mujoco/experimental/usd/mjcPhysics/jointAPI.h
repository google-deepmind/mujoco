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

#ifndef MJCPHYSICS_GENERATED_JOINTAPI_H
#define MJCPHYSICS_GENERATED_JOINTAPI_H

/// \file mjcPhysics/jointAPI.h

#include <mujoco/experimental/usd/mjcPhysics/api.h>
#include <mujoco/experimental/usd/mjcPhysics/tokens.h>
#include <pxr/base/gf/matrix4d.h>
#include <pxr/base/gf/vec3d.h>
#include <pxr/base/gf/vec3f.h>
#include <pxr/base/tf/token.h>
#include <pxr/base/tf/type.h>
#include <pxr/base/vt/value.h>
#include <pxr/pxr.h>
#include <pxr/usd/usd/apiSchemaBase.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/stage.h>

PXR_NAMESPACE_OPEN_SCOPE

class SdfAssetPath;

// -------------------------------------------------------------------------- //
// MJCJOINTAPI                                                                //
// -------------------------------------------------------------------------- //

/// \class MjcPhysicsJointAPI
///
/// API describing a MuJoCo joint.
///
/// For any described attribute \em Fallback \em Value or \em Allowed \em Values
/// below that are text/tokens, the actual token is published and defined in
/// \ref MjcPhysicsTokens. So to set an attribute to the value "rightHanded",
/// use MjcPhysicsTokens->rightHanded as the value.
///
class MjcPhysicsJointAPI : public UsdAPISchemaBase {
 public:
  /// Compile time constant representing what kind of schema this class is.
  ///
  /// \sa UsdSchemaKind
  static const UsdSchemaKind schemaKind = UsdSchemaKind::SingleApplyAPI;

  /// Construct a MjcPhysicsJointAPI on UsdPrim \p prim .
  /// Equivalent to MjcPhysicsJointAPI::Get(prim.GetStage(), prim.GetPath())
  /// for a \em valid \p prim, but will not immediately throw an error for
  /// an invalid \p prim
  explicit MjcPhysicsJointAPI(const UsdPrim& prim = UsdPrim())
      : UsdAPISchemaBase(prim) {}

  /// Construct a MjcPhysicsJointAPI on the prim held by \p schemaObj .
  /// Should be preferred over MjcPhysicsJointAPI(schemaObj.GetPrim()),
  /// as it preserves SchemaBase state.
  explicit MjcPhysicsJointAPI(const UsdSchemaBase& schemaObj)
      : UsdAPISchemaBase(schemaObj) {}

  /// Destructor.
  MJCPHYSICS_API
  virtual ~MjcPhysicsJointAPI();

  /// Return a vector of names of all pre-declared attributes for this schema
  /// class and all its ancestor classes.  Does not include attributes that
  /// may be authored by custom/extended methods of the schemas involved.
  MJCPHYSICS_API
  static const TfTokenVector& GetSchemaAttributeNames(
      bool includeInherited = true);

  /// Return a MjcPhysicsJointAPI holding the prim adhering to this
  /// schema at \p path on \p stage.  If no prim exists at \p path on
  /// \p stage, or if the prim at that path does not adhere to this schema,
  /// return an invalid schema object.  This is shorthand for the following:
  ///
  /// \code
  /// MjcPhysicsJointAPI(stage->GetPrimAtPath(path));
  /// \endcode
  ///
  MJCPHYSICS_API
  static MjcPhysicsJointAPI Get(const UsdStagePtr& stage, const SdfPath& path);

  /// Returns true if this <b>single-apply</b> API schema can be applied to
  /// the given \p prim. If this schema can not be a applied to the prim,
  /// this returns false and, if provided, populates \p whyNot with the
  /// reason it can not be applied.
  ///
  /// Note that if CanApply returns false, that does not necessarily imply
  /// that calling Apply will fail. Callers are expected to call CanApply
  /// before calling Apply if they want to ensure that it is valid to
  /// apply a schema.
  ///
  /// \sa UsdPrim::GetAppliedSchemas()
  /// \sa UsdPrim::HasAPI()
  /// \sa UsdPrim::CanApplyAPI()
  /// \sa UsdPrim::ApplyAPI()
  /// \sa UsdPrim::RemoveAPI()
  ///
  MJCPHYSICS_API
  static bool CanApply(const UsdPrim& prim, std::string* whyNot = nullptr);

  /// Applies this <b>single-apply</b> API schema to the given \p prim.
  /// This information is stored by adding "MjcJointAPI" to the
  /// token-valued, listOp metadata \em apiSchemas on the prim.
  ///
  /// \return A valid MjcPhysicsJointAPI object is returned upon success.
  /// An invalid (or empty) MjcPhysicsJointAPI object is returned upon
  /// failure. See \ref UsdPrim::ApplyAPI() for conditions
  /// resulting in failure.
  ///
  /// \sa UsdPrim::GetAppliedSchemas()
  /// \sa UsdPrim::HasAPI()
  /// \sa UsdPrim::CanApplyAPI()
  /// \sa UsdPrim::ApplyAPI()
  /// \sa UsdPrim::RemoveAPI()
  ///
  MJCPHYSICS_API
  static MjcPhysicsJointAPI Apply(const UsdPrim& prim);

 protected:
  /// Returns the kind of schema this class belongs to.
  ///
  /// \sa UsdSchemaKind
  MJCPHYSICS_API
  UsdSchemaKind _GetSchemaKind() const override;

 private:
  // needs to invoke _GetStaticTfType.
  friend class UsdSchemaRegistry;
  MJCPHYSICS_API
  static const TfType& _GetStaticTfType();

  static bool _IsTypedSchema();

  // override SchemaBase virtuals.
  MJCPHYSICS_API
  const TfType& _GetTfType() const override;

 public:
  // --------------------------------------------------------------------- //
  // GROUP
  // --------------------------------------------------------------------- //
  /// Integer MuJoCo group to which the joint belongs.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform int mjc:group = 0` |
  /// | C++ Type | int |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Int |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetGroupAttr() const;

  /// See GetGroupAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateGroupAttr(VtValue const& defaultValue = VtValue(),
                               bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // MJCSPRINGDAMPER
  // --------------------------------------------------------------------- //
  /// When both numbers are positive, the compiler will override any stiffness
  /// and damping values specified with the attributes below, and will instead
  /// set them automatically so that the resulting mass-spring-damper for this
  /// joint has the desired time constant (first value) and damping ratio
  /// (second value). This is done by taking into account the joint inertia in
  /// the model reference configuration. Note that the format is the same as the
  /// solref parameter of the constraint solver.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double[] mjc:springdamper = [0, 0]` |
  /// | C++ Type | VtArray<double> |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->DoubleArray |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetMjcSpringdamperAttr() const;

  /// See GetMjcSpringdamperAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateMjcSpringdamperAttr(
      VtValue const& defaultValue = VtValue(),
      bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // MJCSOLREFLIMIT
  // --------------------------------------------------------------------- //
  /// Constraint solver parameters for simulating joint limits.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double[] mjc:solreflimit = [0.02, 1]` |
  /// | C++ Type | VtArray<double> |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->DoubleArray |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetMjcSolreflimitAttr() const;

  /// See GetMjcSolreflimitAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateMjcSolreflimitAttr(VtValue const& defaultValue = VtValue(),
                                        bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // MJCSOLIMPLIMIT
  // --------------------------------------------------------------------- //
  /// Constraint solver parameters for simulating joint limits.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double[] mjc:solimplimit = [0.9, 0.95, 0.001,
  /// 0.5, 2]` | | C++ Type | VtArray<double> | | \ref Usd_Datatypes "Usd Type"
  /// | SdfValueTypeNames->DoubleArray | | \ref SdfVariability "Variability" |
  /// SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetMjcSolimplimitAttr() const;

  /// See GetMjcSolimplimitAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateMjcSolimplimitAttr(VtValue const& defaultValue = VtValue(),
                                        bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // MJCSOLREFFRICTION
  // --------------------------------------------------------------------- //
  /// Constraint solver parameters for simulating dry friction.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double[] mjc:solreffriction = [0.02, 1]` |
  /// | C++ Type | VtArray<double> |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->DoubleArray |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetMjcSolreffrictionAttr() const;

  /// See GetMjcSolreffrictionAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateMjcSolreffrictionAttr(
      VtValue const& defaultValue = VtValue(),
      bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // MJCSOLIMPFRICTION
  // --------------------------------------------------------------------- //
  /// Constraint solver parameters for simulating dry friction.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double[] mjc:solimpfriction = [0.9, 0.95, 0.001,
  /// 0.5, 2]` | | C++ Type | VtArray<double> | | \ref Usd_Datatypes "Usd Type"
  /// | SdfValueTypeNames->DoubleArray | | \ref SdfVariability "Variability" |
  /// SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetMjcSolimpfrictionAttr() const;

  /// See GetMjcSolimpfrictionAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateMjcSolimpfrictionAttr(
      VtValue const& defaultValue = VtValue(),
      bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // MJCSTIFFNESS
  // --------------------------------------------------------------------- //
  /// Joint stiffness. If this value is positive, a spring will be created with
  /// equilibrium position given by springref below. The spring force is
  /// computed along with the other passive forces.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double mjc:stiffness = 0` |
  /// | C++ Type | double |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Double |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetMjcStiffnessAttr() const;

  /// See GetMjcStiffnessAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateMjcStiffnessAttr(VtValue const& defaultValue = VtValue(),
                                      bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // MJCACTUATORFRCRANGEMIN
  // --------------------------------------------------------------------- //
  /// Minimum range for clamping total actuator forces acting on this joint. See
  /// Force limits for details. It is available only for scalar joints (hinge
  /// and slider) and ignored for ball and free joints. The compiler expects the
  /// first value to be smaller than the second value. Setting this attribute
  /// without specifying actuatorfrclimited is an error if compiler-autolimits
  /// is 'false'.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double mjc:actuatorfrcrange:min = 0` |
  /// | C++ Type | double |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Double |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetMjcActuatorfrcrangeMinAttr() const;

  /// See GetMjcActuatorfrcrangeMinAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateMjcActuatorfrcrangeMinAttr(
      VtValue const& defaultValue = VtValue(),
      bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // MJCACTUATORFRCRANGEMAX
  // --------------------------------------------------------------------- //
  /// Maximum range for clamping total actuator forces acting on this joint. See
  /// Force limits for details. It is available only for scalar joints (hinge
  /// and slider) and ignored for ball and free joints. The compiler expects the
  /// first value to be smaller than the second value. Setting this attribute
  /// without specifying actuatorfrclimited is an error if compiler-autolimits
  /// is 'false'.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double mjc:actuatorfrcrange:max = 0` |
  /// | C++ Type | double |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Double |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetMjcActuatorfrcrangeMaxAttr() const;

  /// See GetMjcActuatorfrcrangeMaxAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateMjcActuatorfrcrangeMaxAttr(
      VtValue const& defaultValue = VtValue(),
      bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // MJCACTUATORFRCLIMITED
  // --------------------------------------------------------------------- //
  /// This attribute specifies whether actuator forces acting on the joint
  /// should be clamped. See Force limits for details. It is available only for
  /// scalar joints (hinge and slider) and ignored for ball and free joints.
  /// This attribute interacts with the actuatorfrcrange attribute. If this
  /// attribute is 'false', actuator force clamping is disabled. If it is
  /// 'true', actuator force clamping is enabled. If this attribute is 'auto',
  /// and autolimits is set in compiler, actuator force clamping will be enabled
  /// if actuatorfrcrange is defined.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform token mjc:actuatorfrclimited = "auto"` |
  /// | C++ Type | TfToken |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Token |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  /// | \ref MjcPhysicsTokens "Allowed Values" | false, true, auto |
  MJCPHYSICS_API
  UsdAttribute GetMjcActuatorfrclimitedAttr() const;

  /// See GetMjcActuatorfrclimitedAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateMjcActuatorfrclimitedAttr(
      VtValue const& defaultValue = VtValue(),
      bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // MJCACTUATORGRAVCOMP
  // --------------------------------------------------------------------- //
  /// If this flag is enabled, gravity compensation applied to this joint is
  /// added to actuator forces (mjData.qfrc_actuator) rather than passive forces
  /// (mjData.qfrc_passive). Notionally, this means that gravity compensation is
  /// the result of a control system rather than natural buoyancy. In practice,
  /// enabling this flag is useful when joint-level actuator force clamping is
  /// used. In this case, the total actuation force applied on a joint,
  /// including gravity compensation, is guaranteed to not exceed the specified
  /// limits. See Force limits and actuatorfrcrange for more details on this
  /// type of force limit.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform bool mjc:actuatorgravcomp = 0` |
  /// | C++ Type | bool |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetMjcActuatorgravcompAttr() const;

  /// See GetMjcActuatorgravcompAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateMjcActuatorgravcompAttr(
      VtValue const& defaultValue = VtValue(),
      bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // MJCMARGIN
  // --------------------------------------------------------------------- //
  /// The distance threshold below which limits become active. Recall that the
  /// Constraint solver normally generates forces as soon as a constraint
  /// becomes active, even if the margin parameter makes that happen at a
  /// distance. This attribute together with solreflimit and solimplimit can be
  /// used to model a soft joint limit.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double mjc:margin = 0` |
  /// | C++ Type | double |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Double |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetMjcMarginAttr() const;

  /// See GetMjcMarginAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateMjcMarginAttr(VtValue const& defaultValue = VtValue(),
                                   bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // MJCREF
  // --------------------------------------------------------------------- //
  /// The reference position or angle of the joint. This attribute is only used
  /// for slide and hinge joints. It defines the joint value corresponding to
  /// the initial model configuration. The amount of spatial transformation that
  /// the joint applies at runtime equals the current joint value stored in
  /// mjData.qpos minus this reference value stored in mjModel.qpos0. The
  /// meaning of these vectors was discussed in the Stand-alone section in the
  /// Overview chapter.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double mjc:ref = 0` |
  /// | C++ Type | double |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Double |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetMjcRefAttr() const;

  /// See GetMjcRefAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateMjcRefAttr(VtValue const& defaultValue = VtValue(),
                                bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // MJCSPRINGREF
  // --------------------------------------------------------------------- //
  /// The joint position or angle in which the joint spring (if any) achieves
  /// equilibrium. Similar to the vector mjModel.qpos0 which stores all joint
  /// reference values specified with the ref attribute above, all spring
  /// reference values specified with this attribute are stored in the vector
  /// mjModel.qpos_spring. The model configuration corresponding to
  /// mjModel.qpos_spring is also used to compute the spring reference lengths
  /// of all tendons, stored in mjModel.tendon_lengthspring. This is because
  /// tendons can also have springs.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double mjc:springref = 0` |
  /// | C++ Type | double |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Double |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetMjcSpringrefAttr() const;

  /// See GetMjcSpringrefAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateMjcSpringrefAttr(VtValue const& defaultValue = VtValue(),
                                      bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // MJCARMATURE
  // --------------------------------------------------------------------- //
  /// Additional inertia associated with movement of the joint that is not due
  /// to body mass. This added inertia is usually due to a rotor (a.k.a
  /// armature) spinning faster than the joint itself due to a geared
  /// transmission. The value applies to all degrees of freedom created by this
  /// joint. Besides increasing the realism of joints with geared transmission,
  /// positive armature significantly improves simulation stability, even for
  /// small values, and is a recommended possible fix when encountering
  /// stability issues.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double mjc:armature = 0` |
  /// | C++ Type | double |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Double |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetMjcArmatureAttr() const;

  /// See GetMjcArmatureAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateMjcArmatureAttr(VtValue const& defaultValue = VtValue(),
                                     bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // MJCDAMPING
  // --------------------------------------------------------------------- //
  /// Damping applied to all degrees of freedom created by this joint. Unlike
  /// friction loss which is computed by the constraint solver, damping is
  /// simply a force linear in velocity. It is included in the passive forces.
  /// Despite this simplicity, larger damping values can make numerical
  /// integrators unstable, which is why our Euler integrator handles damping
  /// implicitly. See Integration in the Computation chapter.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double mjc:damping = 0` |
  /// | C++ Type | double |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Double |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetMjcDampingAttr() const;

  /// See GetMjcDampingAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateMjcDampingAttr(VtValue const& defaultValue = VtValue(),
                                    bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // MJCFRICTIONLOSS
  // --------------------------------------------------------------------- //
  /// Friction loss due to dry friction. This value is the same for all degrees
  /// of freedom created by this joint. Semantically friction loss does not make
  /// sense for free joints, but the compiler allows it. To enable friction
  /// loss, set this attribute to a positive value.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double mjc:frictionloss = 0` |
  /// | C++ Type | double |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Double |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetMjcFrictionlossAttr() const;

  /// See GetMjcFrictionlossAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateMjcFrictionlossAttr(
      VtValue const& defaultValue = VtValue(),
      bool writeSparsely = false) const;

 public:
  // ===================================================================== //
  // Feel free to add custom code below this line, it will be preserved by
  // the code generator.
  //
  // Just remember to:
  //  - Close the class declaration with };
  //  - Close the namespace with PXR_NAMESPACE_CLOSE_SCOPE
  //  - Close the include guard with #endif
  // ===================================================================== //
  // --(BEGIN CUSTOM CODE)--
};

PXR_NAMESPACE_CLOSE_SCOPE

#endif
