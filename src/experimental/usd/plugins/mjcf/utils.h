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

#include <pxr/base/tf/token.h>
#include <pxr/usd/sdf/attributeSpec.h>
#include <pxr/usd/sdf/declareHandles.h>
#include <pxr/usd/sdf/layer.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/sdf/schema.h>
#include <pxr/usd/sdf/types.h>
#include <pxr/usd/sdf/valueTypeName.h>

namespace mujoco {
namespace usd {

// Create a prim spec and append it as a child of parent_path.
pxr::SdfPrimSpecHandle CreatePrimSpec(
    pxr::SdfLayerRefPtr layer, const pxr::SdfPath& parent_path,
    const pxr::TfToken& name, const pxr::TfToken& type = pxr::TfToken(),
    pxr::SdfSpecifier specifier = pxr::SdfSpecifier::SdfSpecifierDef);

// Create an attribute spec and append it as a child of parent_path.
// By default the attribute will be varying.
pxr::SdfAttributeSpecHandle CreateAttributeSpec(
    pxr::SdfLayerRefPtr layer, const pxr::SdfPrimSpecHandle& prim_spec,
    const pxr::TfToken& name, const pxr::SdfValueTypeName& type_name,
    pxr::SdfVariability variability = pxr::SdfVariabilityVarying);

// Create a relationship spec and append it as a child of prim_path.
pxr::SdfRelationshipSpecHandle CreateRelationshipSpec(
    pxr::SdfLayerRefPtr layer, const pxr::SdfPrimSpecHandle& prim_spec,
    const pxr::TfToken& relationship_name,
    const pxr::SdfPath& relationship_path,
    pxr::SdfVariability variability = pxr::SdfVariabilityVarying);

pxr::SdfPrimSpecHandle CreateClassSpec(pxr::SdfLayerRefPtr layer,
                             const pxr::SdfPath& prim_path,
                             const pxr::TfToken& class_name);

void AddAttributeConnection(pxr::SdfLayerRefPtr layer,
                            const pxr::SdfAttributeSpecHandle& attribute_spec,
                            const pxr::SdfAttributeSpecHandle& target_attribute_spec);

void AddPrimReference(pxr::SdfLayerRefPtr layer,
                      const pxr::SdfPrimSpecHandle& prim_spec,
                      const pxr::SdfPath& referenced_prim_path);

void AddPrimInherit(pxr::SdfLayerRefPtr layer,
                    const pxr::SdfPrimSpecHandle& prim_spec,
                    const pxr::SdfPath& class_path);

void ApplyApiSchema(pxr::SdfLayerRefPtr layer,
                    const pxr::SdfPrimSpecHandle& prim_spec,
                    const pxr::TfToken& schema_name);

void SetPrimKind(pxr::SdfLayerRefPtr layer,
                 const pxr::SdfPrimSpecHandle& prim_spec, pxr::TfToken kind);

void SetPrimPurpose(pxr::SdfLayerRefPtr layer,
                    const pxr::SdfPrimSpecHandle& prim_spec, pxr::TfToken purpose);

// Set the value specified by key on any field at field_path.
template <typename T>
void SetField(pxr::SdfLayerRefPtr layer, const pxr::SdfPath& field_path,
              const pxr::TfToken key, T&& value) {
  layer->SetField(field_path, key, value);
}

// Set the value specified by key on any field at field_path.
template <typename T>
void SetFieldTimeSample(pxr::SdfLayerRefPtr layer,
                        const pxr::SdfPath& field_path, double time,
                        T&& value) {
  layer->SetTimeSample(field_path, time, value);
}


// Set the value specified by key on an attribute spec at attribute_path.
template <typename T>
void SetAttributeMetadata(pxr::SdfLayerRefPtr layer,
                          const pxr::SdfPath& attribute_path,
                          const pxr::TfToken key, T&& value) {
  SetAttribute(layer, attribute_path, key, value);
}

// Set the default value on an attribute spec at attribute_path.
template <typename T>
void SetAttributeDefault(pxr::SdfLayerRefPtr layer,
                         const pxr::SdfAttributeSpecHandle& attribute,
                         T&& default_value) {
  attribute->SetField(pxr::SdfFieldKeys->Default, default_value);
}

// Set the default value on an attribute spec at attribute_path.
template <typename T>
void SetAttributeTimeSample(pxr::SdfLayerRefPtr layer,
                         const pxr::SdfAttributeSpecHandle& attr_spec,
                         double time,
                         T&& default_value) {
  layer->SetTimeSample(attr_spec->GetPath(), time, default_value);
}

// Set the value specified by key on the root layer.
template <typename T>
void SetLayerMetadata(pxr::SdfLayerRefPtr layer, const pxr::TfToken& key,
                      T&& value) {
  layer->SetField(pxr::SdfPath::AbsoluteRootPath(), key, value);
}

}  // namespace usd
}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_USD_PLUGINS_MJCF_UTILS_H_
