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

#ifndef MUJOCO_SRC_EXPERIMENTAL_USD_PLUGINS_MJCF_UTILS_H_
#define MUJOCO_SRC_EXPERIMENTAL_USD_PLUGINS_MJCF_UTILS_H_

#include <type_traits>

#include <pxr/base/tf/token.h>
#include <pxr/imaging/hd/primTypeIndex.h>
#include <pxr/usd/sdf/abstractData.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/sdf/schema.h>
#include <pxr/usd/sdf/types.h>
#include <pxr/usd/sdf/valueTypeName.h>

namespace mujoco {
namespace usd {

// Create a prim spec and append it as a child of parent_path.
pxr::SdfPath CreatePrimSpec(
    pxr::SdfAbstractDataRefPtr& data, const pxr::SdfPath& parent_path,
    const pxr::TfToken& name, const pxr::TfToken& type = pxr::TfToken(),
    pxr::SdfSpecifier specifier = pxr::SdfSpecifier::SdfSpecifierDef);

// Create an attribute spec and append it as a child of parent_path.
// By default the attribute will be varying.
pxr::SdfPath CreateAttributeSpec(
    pxr::SdfAbstractDataRefPtr& data, const pxr::SdfPath& prim_path,
    const pxr::TfToken& name, const pxr::SdfValueTypeName& type_name,
    pxr::SdfVariability variability = pxr::SdfVariabilityVarying);

// Create a relationship spec and append it as a child of prim_path.
pxr::SdfPath CreateRelationshipSpec(
    pxr::SdfAbstractDataRefPtr& data, const pxr::SdfPath& prim_path,
    const pxr::TfToken& relationship_name,
    const pxr::SdfPath& relationship_path,
    pxr::SdfVariability variability = pxr::SdfVariabilityVarying);

pxr::SdfPath CreateClassSpec(pxr::SdfAbstractDataRefPtr& data,
                             const pxr::SdfPath& prim_path,
                             const pxr::TfToken& class_name);

void AddAttributeConnection(pxr::SdfAbstractDataRefPtr& data,
                            const pxr::SdfPath& attribute_path,
                            const pxr::SdfPath& target_attribute_path);

void AddPrimReference(pxr::SdfAbstractDataRefPtr& data,
                      const pxr::SdfPath& prim_path,
                      const pxr::SdfPath& referenced_prim_path);

void AddPrimInherit(pxr::SdfAbstractDataRefPtr& data,
                    const pxr::SdfPath& prim_path,
                    const pxr::SdfPath& class_path);

void ApplyApiSchema(pxr::SdfAbstractDataRefPtr& data,
                    const pxr::SdfPath& prim_path,
                    const pxr::TfToken& schema_name);

void SetPrimKind(pxr::SdfAbstractDataRefPtr& data,
                 const pxr::SdfPath& prim_path, pxr::TfToken kind);

void SetPrimPurpose(pxr::SdfAbstractDataRefPtr& data,
                    const pxr::SdfPath& prim_path, pxr::TfToken purpose);

// Set the value specified by key on any field at field_path.
template <typename T>
void SetField(pxr::SdfAbstractDataRefPtr& data, const pxr::SdfPath& field_path,
              const pxr::TfToken key, T&& value) {
  using Deduced = typename std::remove_reference_t<T>;
  const auto typed_val = pxr::SdfAbstractDataConstTypedValue<Deduced>(&value);
  const pxr::SdfAbstractDataConstValue& untyped_val = typed_val;

  data->Set(field_path, key, untyped_val);
}

// Set the value specified by key on an attribute spec at attribute_path.
template <typename T>
void SetAttribute(pxr::SdfAbstractDataRefPtr& data,
                  const pxr::SdfPath& attribute_path, const pxr::TfToken key,
                  T&& value) {
  SetField(data, attribute_path, key, value);
}

// Set the value specified by key on a prim spec at prim_path.
template <typename T>
void SetPrimMetadata(pxr::SdfAbstractDataRefPtr& data,
                     const pxr::SdfPath& prim_path, const pxr::TfToken key,
                     T&& value) {
  SetAttribute(data, prim_path, key, value);
}

// Set the value specified by key on an attribute spec at attribute_path.
template <typename T>
void SetAttributeMetadata(pxr::SdfAbstractDataRefPtr& data,
                          const pxr::SdfPath& attribute_path,
                          const pxr::TfToken key, T&& value) {
  SetAttribute(data, attribute_path, key, value);
}

// Set the default value on an attribute spec at attribute_path.
template <typename T>
void SetAttributeDefault(pxr::SdfAbstractDataRefPtr& data,
                         const pxr::SdfPath& attribute_path,
                         T&& default_value) {
  SetAttribute(data, attribute_path, pxr::SdfFieldKeys->Default, default_value);
}

// Set the value specified by key on the root layer.
template <typename T>
void SetLayerMetadata(pxr::SdfAbstractDataRefPtr& data, const pxr::TfToken& key,
                      T&& value) {
  SetAttribute(data, pxr::SdfPath::AbsoluteRootPath(), key, value);
}

}  // namespace usd
}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_USD_PLUGINS_MJCF_UTILS_H_
