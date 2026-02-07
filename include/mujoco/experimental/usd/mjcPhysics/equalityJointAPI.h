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

#ifndef MJCPHYSICS_GENERATED_EQUALITYJOINTAPI_H
#define MJCPHYSICS_GENERATED_EQUALITYJOINTAPI_H

/// \file mjcPhysics/equalityJointAPI.h

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
// MJCEQUALITYJOINTAPI                                                        //
// -------------------------------------------------------------------------- //

/// \class MjcPhysicsEqualityJointAPI
///
/// API providing extension attributes to represent equality/joint constraints.
/// This API is applied to a joint prim which acts as the constrained joint
/// (joint1 in MuJoCo terminology). The target relationship points to another
/// joint prim which is the reference joint (joint2 in MuJoCo terminology). The
/// constrained joint's position or angle is constrained to be a quartic
/// polynomial of the reference joint's position or angle. Only scalar joint
/// types (slide and hinge) can be used.
///
class MjcPhysicsEqualityJointAPI : public UsdAPISchemaBase {
 public:
  /// Compile time constant representing what kind of schema this class is.
  ///
  /// \sa UsdSchemaKind
  static const UsdSchemaKind schemaKind = UsdSchemaKind::SingleApplyAPI;

  /// Construct a MjcPhysicsEqualityJointAPI on UsdPrim \p prim .
  /// Equivalent to MjcPhysicsEqualityJointAPI::Get(prim.GetStage(),
  /// prim.GetPath()) for a \em valid \p prim, but will not immediately throw an
  /// error for an invalid \p prim
  explicit MjcPhysicsEqualityJointAPI(const UsdPrim& prim = UsdPrim())
      : UsdAPISchemaBase(prim) {}

  /// Construct a MjcPhysicsEqualityJointAPI on the prim held by \p schemaObj .
  /// Should be preferred over MjcPhysicsEqualityJointAPI(schemaObj.GetPrim()),
  /// as it preserves SchemaBase state.
  explicit MjcPhysicsEqualityJointAPI(const UsdSchemaBase& schemaObj)
      : UsdAPISchemaBase(schemaObj) {}

  /// Destructor.
  MJCPHYSICS_API
  virtual ~MjcPhysicsEqualityJointAPI();

  /// Return a vector of names of all pre-declared attributes for this schema
  /// class and all its ancestor classes.  Does not include attributes that
  /// may be authored by custom/extended methods of the schemas involved.
  MJCPHYSICS_API
  static const TfTokenVector& GetSchemaAttributeNames(
      bool includeInherited = true);

  /// Return a MjcPhysicsEqualityJointAPI holding the prim adhering to this
  /// schema at \p path on \p stage.  If no prim exists at \p path on
  /// \p stage, or if the prim at that path does not adhere to this schema,
  /// return an invalid schema object.  This is shorthand for the following:
  ///
  /// \code
  /// MjcPhysicsEqualityJointAPI(stage->GetPrimAtPath(path));
  /// \endcode
  ///
  MJCPHYSICS_API
  static MjcPhysicsEqualityJointAPI Get(const UsdStagePtr& stage,
                                        const SdfPath& path);

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
  /// This information is stored by adding "MjcEqualityJointAPI" to the
  /// token-valued, listOp metadata \em apiSchemas on the prim.
  ///
  /// \return A valid MjcPhysicsEqualityJointAPI object is returned upon
  /// success. An invalid (or empty) MjcPhysicsEqualityJointAPI object is
  /// returned upon failure. See \ref UsdPrim::ApplyAPI() for conditions
  /// resulting in failure.
  ///
  /// \sa UsdPrim::GetAppliedSchemas()
  /// \sa UsdPrim::HasAPI()
  /// \sa UsdPrim::CanApplyAPI()
  /// \sa UsdPrim::ApplyAPI()
  /// \sa UsdPrim::RemoveAPI()
  ///
  MJCPHYSICS_API
  static MjcPhysicsEqualityJointAPI Apply(const UsdPrim& prim);

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
  // COEF0
  // --------------------------------------------------------------------- //
  /// Constant coefficient a0 of the quartic polynomial. The constraint is:
  /// y = y0 + a0 + a1*(x-x0) + a2*(x-x0)^2 + a3*(x-x0)^3 + a4*(x-x0)^4.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double mjc:coef0 = 0` |
  /// | C++ Type | double |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Double |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetCoef0Attr() const;

  /// See GetCoef0Attr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateCoef0Attr(VtValue const& defaultValue = VtValue(),
                               bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // COEF1
  // --------------------------------------------------------------------- //
  /// Linear coefficient a1 of the quartic polynomial.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double mjc:coef1 = 1` |
  /// | C++ Type | double |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Double |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetCoef1Attr() const;

  /// See GetCoef1Attr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateCoef1Attr(VtValue const& defaultValue = VtValue(),
                               bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // COEF2
  // --------------------------------------------------------------------- //
  /// Quadratic coefficient a2 of the quartic polynomial.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double mjc:coef2 = 0` |
  /// | C++ Type | double |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Double |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetCoef2Attr() const;

  /// See GetCoef2Attr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateCoef2Attr(VtValue const& defaultValue = VtValue(),
                               bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // COEF3
  // --------------------------------------------------------------------- //
  /// Cubic coefficient a3 of the quartic polynomial.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double mjc:coef3 = 0` |
  /// | C++ Type | double |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Double |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetCoef3Attr() const;

  /// See GetCoef3Attr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateCoef3Attr(VtValue const& defaultValue = VtValue(),
                               bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // COEF4
  // --------------------------------------------------------------------- //
  /// Quartic coefficient a4 of the quartic polynomial.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `uniform double mjc:coef4 = 0` |
  /// | C++ Type | double |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Double |
  /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
  MJCPHYSICS_API
  UsdAttribute GetCoef4Attr() const;

  /// See GetCoef4Attr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateCoef4Attr(VtValue const& defaultValue = VtValue(),
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
