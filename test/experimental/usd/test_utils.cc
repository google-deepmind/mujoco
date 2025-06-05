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

#include "test/experimental/usd/test_utils.h"

#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <pxr/base/tf/token.h>
#include <pxr/usd/sdf/assetPath.h>
#include <pxr/usd/sdf/childrenPolicies.h>
#include <pxr/usd/sdf/declareHandles.h>
#include <pxr/usd/sdf/fileFormat.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/sdf/schema.h>
#include <pxr/usd/usd/common.h>
#include <pxr/usd/usd/modelAPI.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/usd/usdGeom/primvarsAPI.h>
namespace mujoco {
namespace usd {

using pxr::SdfPath;

pxr::SdfLayerRefPtr LoadLayer(
    const std::string& xml,
    const pxr::SdfFileFormat::FileFormatArguments& args) {
  auto layer = pxr::SdfLayer::CreateAnonymous(
      "test_layer", pxr::SdfFileFormat::FindByExtension("xml"), args);
  layer->ImportFromString(xml);
  EXPECT_THAT(layer, testing::NotNull());
  return layer;
}

pxr::UsdStageRefPtr OpenStageWithPhysics(const std::string& xml) {
  pxr::SdfFileFormat::FileFormatArguments args;
  args["usdMjcfToggleUsdPhysics"] = "true";
  pxr::SdfLayerRefPtr layer = LoadLayer(xml, args);
  auto stage = pxr::UsdStage::Open(layer);
  EXPECT_THAT(stage, testing::NotNull());
  return stage;
}

template <>
void ExpectAttributeEqual<pxr::SdfAssetPath>(pxr::UsdStageRefPtr stage,
                                             pxr::SdfPath path,
                                             const pxr::SdfAssetPath& value) {
  auto attr = stage->GetAttributeAtPath(path);
  EXPECT_TRUE(attr.IsValid());
  pxr::SdfAssetPath attr_value;
  attr.Get(&attr_value);
  EXPECT_EQ(attr_value.GetAssetPath(), value.GetAssetPath());
}

void ExpectAttributeHasConnection(pxr::UsdStageRefPtr stage, const char* path,
                                  const char* connection_path) {
  auto attr = stage->GetAttributeAtPath(SdfPath(path));
  EXPECT_TRUE(attr.IsValid());
  pxr::SdfPathVector sources;
  attr.GetConnections(&sources);
  EXPECT_EQ(sources.size(), 1);
  EXPECT_EQ(sources[0], SdfPath(connection_path));
}

void ExpectAllAuthoredAttributesMatchSchemaTypes(const pxr::UsdPrim& prim) {
  // Get all properties on the prim that have authored opinions.
  for (const pxr::UsdProperty& prop : prim.GetAuthoredProperties()) {
    // We only care about attributes, as they are the ones with a typeName.
    if (pxr::UsdAttribute attr = prop.As<pxr::UsdAttribute>()) {
      // 1. Get the official, composed schema type name for the attribute.
      const pxr::TfToken schemaTypeName = attr.GetTypeName().GetAsToken();

      // An empty schema type name means the attribute is not defined by
      // a schema, or is of a dynamically-determined type. We can't
      // check for a mismatch in this case.
      if (schemaTypeName.IsEmpty()) {
        continue;
      }

      // 2. Get the property stack to check for authored opinions.
      // The stack is ordered from strongest to weakest.
      const pxr::SdfPropertySpecHandleVector propStack =
          attr.GetPropertyStack();

      for (const pxr::SdfPropertySpecHandle& spec : propStack) {
        // We only care about attribute specs.
        if (auto attrSpec = TfDynamic_cast<pxr::SdfAttributeSpecHandle>(spec)) {
          // 3. Check if this spec has an authored `typeName`.
          if (attrSpec->HasField(pxr::SdfFieldKeys->TypeName)) {
            const pxr::TfToken authoredTypeName =
                attrSpec->GetTypeName().GetAsToken();

            EXPECT_EQ(authoredTypeName, schemaTypeName)
                << "Type mismatch for attribute <" << attr.GetPath()
                << ">: expected schema-defined type '"
                << schemaTypeName.GetString() << "', got authored type '"
                << authoredTypeName.GetString() << "' in layer @"
                << attrSpec->GetLayer()->GetIdentifier() << "@";

            // We've found the strongest authored opinion for `typeName`,
            // so we can stop checking the stack for this attribute.
            break;
          }
        }
      }
    }
  }
}
}  // namespace usd
}  // namespace mujoco
