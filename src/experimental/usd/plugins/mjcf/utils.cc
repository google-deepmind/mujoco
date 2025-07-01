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

#include "mjcf/utils.h"

#include <string>
#include <vector>

#include <pxr/base/tf/token.h>
#include <pxr/usd/sdf/abstractData.h>
#include <pxr/usd/sdf/listOp.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/sdf/reference.h>
#include <pxr/usd/sdf/schema.h>
#include <pxr/usd/sdf/types.h>
#include <pxr/usd/sdf/valueTypeName.h>
#include <pxr/usd/usd/tokens.h>
#include <pxr/usd/usdGeom/tokens.h>
namespace {

template <typename T>
void AppendChild(pxr::SdfAbstractDataRefPtr& data, const pxr::SdfPath& specPath,
                 const pxr::TfToken& childKey, const T& child) {
  // Get existing children.
  std::vector<T> children;
  pxr::SdfAbstractDataTypedValue<std::vector<T>> getter(&children);
  data->Has(specPath, childKey, &getter);

  children.push_back(child);
  data->Set(specPath, childKey,
            pxr::SdfAbstractDataConstTypedValue<std::vector<T>>(&children));
}

template <typename T>
void AppendListOp(pxr::SdfAbstractDataRefPtr& data,
                  const pxr::SdfPath& spec_path, const pxr::TfToken& field,
                  const T& item) {
  // Get existing list op.
  pxr::SdfListOp<T> list_op;
  pxr::SdfAbstractDataTypedValue<pxr::SdfListOp<T>> getter(&list_op);
  data->Has(spec_path, field, &getter);

  auto items = list_op.GetExplicitItems();
  items.push_back(item);
  list_op.SetExplicitItems(items);
  data->Set(spec_path, field,
            pxr::SdfAbstractDataConstTypedValue<pxr::SdfListOp<T>>(&list_op));
}

template <typename T>
void PrependListOp(pxr::SdfAbstractDataRefPtr& data,
                   const pxr::SdfPath& spec_path, const pxr::TfToken& field,
                   const T& item) {
  // Get existing list op.
  pxr::SdfListOp<T> listOp;
  pxr::SdfAbstractDataTypedValue<pxr::SdfListOp<T>> getter(&listOp);
  data->Has(spec_path, field, &getter);

  auto prependedItems = listOp.GetPrependedItems();
  prependedItems.insert(prependedItems.begin(), item);
  listOp.SetPrependedItems(prependedItems);
  data->Set(spec_path, field,
            pxr::SdfAbstractDataConstTypedValue<pxr::SdfListOp<T>>(&listOp));
}
}  // namespace

namespace mujoco {
namespace usd {

pxr::SdfPath CreatePrimSpec(pxr::SdfAbstractDataRefPtr& data,
                            const pxr::SdfPath& parent_path,
                            const pxr::TfToken& name, const pxr::TfToken& type,
                            pxr::SdfSpecifier specifier) {
  const pxr::SdfPath prim_path = parent_path.AppendChild(name);
  data->CreateSpec(prim_path, pxr::SdfSpecTypePrim);
  data->Set(prim_path, pxr::SdfFieldKeys->Specifier,
            pxr::SdfAbstractDataConstTypedValue<pxr::SdfSpecifier>(&specifier));
  if (!type.IsEmpty()) {
    data->Set(prim_path, pxr::SdfFieldKeys->TypeName,
              pxr::SdfAbstractDataConstTypedValue<const pxr::TfToken>(&type));
  }

  AppendChild(data, parent_path, pxr::SdfChildrenKeys->PrimChildren, name);

  return prim_path;
}

pxr::SdfPath CreateAttributeSpec(pxr::SdfAbstractDataRefPtr& data,
                                 const pxr::SdfPath& prim_path,
                                 const pxr::TfToken& name,
                                 const pxr::SdfValueTypeName& type_name,
                                 pxr::SdfVariability variability) {
  const pxr::SdfPath propertyPath = prim_path.AppendProperty(name);
  data->CreateSpec(propertyPath, pxr::SdfSpecTypeAttribute);

  pxr::TfToken typeNameToken = type_name.GetAsToken();
  data->Set(propertyPath, pxr::SdfFieldKeys->TypeName,
            pxr::SdfAbstractDataConstTypedValue<pxr::TfToken>(&typeNameToken));
  if (variability != pxr::SdfVariabilityVarying) {
    data->Set(
        propertyPath, pxr::SdfFieldKeys->Variability,
        pxr::SdfAbstractDataConstTypedValue<pxr::SdfVariability>(&variability));
  }

  AppendChild(data, prim_path, pxr::SdfChildrenKeys->PropertyChildren, name);

  return propertyPath;
}

pxr::SdfPath CreateRelationshipSpec(pxr::SdfAbstractDataRefPtr& data,
                                    const pxr::SdfPath& prim_path,
                                    const pxr::TfToken& relationship_name,
                                    const pxr::SdfPath& relationship_path,
                                    pxr::SdfVariability variability) {
  pxr::SdfPath prop_path = prim_path.AppendProperty(relationship_name);
  data->CreateSpec(prop_path, pxr::SdfSpecTypeRelationship);
  if (variability != pxr::SdfVariabilityVarying) {
    data->Set(
        prop_path, pxr::SdfFieldKeys->Variability,
        pxr::SdfAbstractDataConstTypedValue<pxr::SdfVariability>(&variability));
  }

  AppendChild(data, prim_path, pxr::SdfChildrenKeys->PropertyChildren,
              relationship_name);

  AppendChild(data, prop_path, pxr::SdfChildrenKeys->RelationshipTargetChildren,
              relationship_path);
  AppendListOp(data, prop_path, pxr::SdfFieldKeys->TargetPaths,
               relationship_path);

  pxr::SdfPath target_path = prop_path.AppendTarget(relationship_path);
  data->CreateSpec(target_path, pxr::SdfSpecTypeRelationshipTarget);

  return prop_path;
}

pxr::SdfPath CreateClassSpec(pxr::SdfAbstractDataRefPtr& data,
                             const pxr::SdfPath& prim_path,
                             const pxr::TfToken& class_name) {
  pxr::SdfPath class_path = prim_path.AppendChild(class_name);
  pxr::SdfSpecifier class_specifier = pxr::SdfSpecifier::SdfSpecifierClass;
  data->CreateSpec(class_path, pxr::SdfSpecTypePrim);
  data->Set(
      class_path, pxr::SdfFieldKeys->Specifier,
      pxr::SdfAbstractDataConstTypedValue<pxr::SdfSpecifier>(&class_specifier));

  AppendChild(data, prim_path, pxr::SdfChildrenKeys->PrimChildren, class_name);

  return class_path;
}

void AddAttributeConnection(pxr::SdfAbstractDataRefPtr& data,
                            const pxr::SdfPath& attribute_path,
                            const pxr::SdfPath& target_attribute_path) {
  AppendChild(data, attribute_path, pxr::SdfChildrenKeys->ConnectionChildren,
              target_attribute_path);
  AppendListOp(data, attribute_path, pxr::SdfFieldKeys->ConnectionPaths,
               target_attribute_path);

  data->CreateSpec(attribute_path.AppendTarget(target_attribute_path),
                   pxr::SdfSpecTypeConnection);
}

void AddPrimReference(pxr::SdfAbstractDataRefPtr& data,
                      const pxr::SdfPath& prim_path,
                      const pxr::SdfPath& referenced_prim_path) {
  PrependListOp(data, prim_path, pxr::SdfFieldKeys->References,
                pxr::SdfReference("", referenced_prim_path));
}

void AddPrimInherit(pxr::SdfAbstractDataRefPtr& data,
                    const pxr::SdfPath& prim_path,
                    const pxr::SdfPath& class_path) {
  PrependListOp(data, prim_path, pxr::SdfFieldKeys->InheritPaths, class_path);
}

void ApplyApiSchema(pxr::SdfAbstractDataRefPtr& data,
                    const pxr::SdfPath& prim_path,
                    const pxr::TfToken& schema_name) {
  PrependListOp(data, prim_path, pxr::UsdTokens->apiSchemas, schema_name);
}

void SetPrimKind(pxr::SdfAbstractDataRefPtr& data,
                 const pxr::SdfPath& prim_path, pxr::TfToken kind) {
  SetPrimMetadata(data, prim_path, pxr::TfToken("kind"), kind);
}

void SetPrimPurpose(pxr::SdfAbstractDataRefPtr& data,
                    const pxr::SdfPath& prim_path, pxr::TfToken purpose) {
  const pxr::SdfPath attr_path = CreateAttributeSpec(
      data, prim_path, pxr::UsdGeomTokens->purpose,
      pxr::SdfValueTypeNames->Token, pxr::SdfVariabilityUniform);
  SetAttributeDefault(data, attr_path, purpose);
}

}  // namespace usd
}  // namespace mujoco
