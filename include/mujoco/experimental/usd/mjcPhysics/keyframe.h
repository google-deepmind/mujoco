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

#ifndef MJCPHYSICS_GENERATED_KEYFRAME_H
#define MJCPHYSICS_GENERATED_KEYFRAME_H

/// \file mjcPhysics/keyframe.h

#include <mujoco/experimental/usd/mjcPhysics/api.h>
#include <mujoco/experimental/usd/mjcPhysics/tokens.h>
#include <pxr/base/gf/matrix4d.h>
#include <pxr/base/gf/vec3d.h>
#include <pxr/base/gf/vec3f.h>
#include <pxr/base/tf/token.h>
#include <pxr/base/tf/type.h>
#include <pxr/base/vt/value.h>
#include <pxr/pxr.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usd/typed.h>

PXR_NAMESPACE_OPEN_SCOPE

class SdfAssetPath;

// -------------------------------------------------------------------------- //
// MJCKEYFRAME                                                                //
// -------------------------------------------------------------------------- //

/// \class MjcPhysicsKeyframe
///
/// Represents time independent keyframe values.
///
class MjcPhysicsKeyframe : public UsdTyped {
 public:
  /// Compile time constant representing what kind of schema this class is.
  ///
  /// \sa UsdSchemaKind
  static const UsdSchemaKind schemaKind = UsdSchemaKind::ConcreteTyped;

  /// Construct a MjcPhysicsKeyframe on UsdPrim \p prim .
  /// Equivalent to MjcPhysicsKeyframe::Get(prim.GetStage(), prim.GetPath())
  /// for a \em valid \p prim, but will not immediately throw an error for
  /// an invalid \p prim
  explicit MjcPhysicsKeyframe(const UsdPrim& prim = UsdPrim())
      : UsdTyped(prim) {}

  /// Construct a MjcPhysicsKeyframe on the prim held by \p schemaObj .
  /// Should be preferred over MjcPhysicsKeyframe(schemaObj.GetPrim()),
  /// as it preserves SchemaBase state.
  explicit MjcPhysicsKeyframe(const UsdSchemaBase& schemaObj)
      : UsdTyped(schemaObj) {}

  /// Destructor.
  MJCPHYSICS_API
  virtual ~MjcPhysicsKeyframe();

  /// Return a vector of names of all pre-declared attributes for this schema
  /// class and all its ancestor classes.  Does not include attributes that
  /// may be authored by custom/extended methods of the schemas involved.
  MJCPHYSICS_API
  static const TfTokenVector& GetSchemaAttributeNames(
      bool includeInherited = true);

  /// Return a MjcPhysicsKeyframe holding the prim adhering to this
  /// schema at \p path on \p stage.  If no prim exists at \p path on
  /// \p stage, or if the prim at that path does not adhere to this schema,
  /// return an invalid schema object.  This is shorthand for the following:
  ///
  /// \code
  /// MjcPhysicsKeyframe(stage->GetPrimAtPath(path));
  /// \endcode
  ///
  MJCPHYSICS_API
  static MjcPhysicsKeyframe Get(const UsdStagePtr& stage, const SdfPath& path);

  /// Attempt to ensure a \a UsdPrim adhering to this schema at \p path
  /// is defined (according to UsdPrim::IsDefined()) on this stage.
  ///
  /// If a prim adhering to this schema at \p path is already defined on this
  /// stage, return that prim.  Otherwise author an \a SdfPrimSpec with
  /// \a specifier == \a SdfSpecifierDef and this schema's prim type name for
  /// the prim at \p path at the current EditTarget.  Author \a SdfPrimSpec s
  /// with \p specifier == \a SdfSpecifierDef and empty typeName at the
  /// current EditTarget for any nonexistent, or existing but not \a Defined
  /// ancestors.
  ///
  /// The given \a path must be an absolute prim path that does not contain
  /// any variant selections.
  ///
  /// If it is impossible to author any of the necessary PrimSpecs, (for
  /// example, in case \a path cannot map to the current UsdEditTarget's
  /// namespace) issue an error and return an invalid \a UsdPrim.
  ///
  /// Note that this method may return a defined prim whose typeName does not
  /// specify this schema class, in case a stronger typeName opinion overrides
  /// the opinion at the current EditTarget.
  ///
  MJCPHYSICS_API
  static MjcPhysicsKeyframe Define(const UsdStagePtr& stage,
                                   const SdfPath& path);

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
  // MJCQPOS
  // --------------------------------------------------------------------- //
  /// Vector of joint positions, copied into mjData.qpos when the simulation
  /// state is set to this keyframe.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `double[] mjc:qpos` |
  /// | C++ Type | VtArray<double> |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->DoubleArray |
  MJCPHYSICS_API
  UsdAttribute GetMjcQposAttr() const;

  /// See GetMjcQposAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateMjcQposAttr(VtValue const& defaultValue = VtValue(),
                                 bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // MJCQVEL
  // --------------------------------------------------------------------- //
  /// Vector of joint velocities, copied into mjData.qvel when the simulation
  /// state is set to this keyframe.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `double[] mjc:qvel` |
  /// | C++ Type | VtArray<double> |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->DoubleArray |
  MJCPHYSICS_API
  UsdAttribute GetMjcQvelAttr() const;

  /// See GetMjcQvelAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateMjcQvelAttr(VtValue const& defaultValue = VtValue(),
                                 bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // MJCACT
  // --------------------------------------------------------------------- //
  /// Vector of actuator activations, copied into mjData.act when the simulation
  /// state is set to this keyframe.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `double[] mjc:act` |
  /// | C++ Type | VtArray<double> |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->DoubleArray |
  MJCPHYSICS_API
  UsdAttribute GetMjcActAttr() const;

  /// See GetMjcActAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateMjcActAttr(VtValue const& defaultValue = VtValue(),
                                bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // MJCCTRL
  // --------------------------------------------------------------------- //
  /// Vector of controls, copied into mjData.ctrl when the simulation state is
  /// set to this keyframe.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `double[] mjc:ctrl` |
  /// | C++ Type | VtArray<double> |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->DoubleArray |
  MJCPHYSICS_API
  UsdAttribute GetMjcCtrlAttr() const;

  /// See GetMjcCtrlAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateMjcCtrlAttr(VtValue const& defaultValue = VtValue(),
                                 bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // MJCMPOS
  // --------------------------------------------------------------------- //
  /// Vector of mocap body positions, copied into mjData.mocap_pos when the
  /// simulation state is set to this keyframe.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `double[] mjc:mpos` |
  /// | C++ Type | VtArray<double> |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->DoubleArray |
  MJCPHYSICS_API
  UsdAttribute GetMjcMposAttr() const;

  /// See GetMjcMposAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateMjcMposAttr(VtValue const& defaultValue = VtValue(),
                                 bool writeSparsely = false) const;

 public:
  // --------------------------------------------------------------------- //
  // MJCMQUAT
  // --------------------------------------------------------------------- //
  /// Vector of mocap body quaternions, copied into mjData.mocap_quat when the
  /// simulation state is set to this keyframe.
  ///
  /// | ||
  /// | -- | -- |
  /// | Declaration | `double[] mjc:mquat` |
  /// | C++ Type | VtArray<double> |
  /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->DoubleArray |
  MJCPHYSICS_API
  UsdAttribute GetMjcMquatAttr() const;

  /// See GetMjcMquatAttr(), and also
  /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
  /// If specified, author \p defaultValue as the attribute's default,
  /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
  /// the default for \p writeSparsely is \c false.
  MJCPHYSICS_API
  UsdAttribute CreateMjcMquatAttr(VtValue const& defaultValue = VtValue(),
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
