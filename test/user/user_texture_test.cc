// Copyright 2026 DeepMind Technologies Limited
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

// Tests for textures by reference: <compiler passthrough> and <texture embed>.

#include <array>
#include <cstddef>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <ios>
#include <iterator>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjspec.h>
#include <mujoco/mjxmacro.h>
#include <mujoco/mujoco.h>
#include "src/engine/engine_io.h"
#include "test/compare_model.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::AllOf;
using ::testing::HasSubstr;
using ::testing::IsNull;
using ::testing::NotNull;

// the path stored for an asset, "" if none
std::string PathAt(const mjModel* m, int pathadr) {
  return pathadr < 0 ? "" : std::string(m->paths + pathadr);
}

// whether a path stored in mjModel opens on its own, from any directory
bool Opens(const std::string& path) {
  std::array<char, 1024> error;
  mjResource* resource =
      mju_openResource("", path.c_str(), nullptr, error.data(), error.size());
  if (!resource) return false;
  mju_closeResource(resource);
  return true;
}

// the content of a file under test/
std::string ReadTestData(const std::string& path) {
  std::ifstream file(GetTestDataFilePath(path), std::ios::binary);
  return std::string((std::istreambuf_iterator<char>(file)), {});
}

// write content to dir/name, creating dir
void WriteFile(const std::filesystem::path& dir, const std::string& name,
               std::string_view content) {
  std::filesystem::create_directories(dir);
  std::ofstream((dir / name).string(), std::ios::binary) << content;
}

// the fields which differ between an embedded and a by-reference compile of the
// same spec: the payload and the colorspace which only a decode resolves
bool IsPayloadField(std::string_view name) {
  return name == "tex_adr" || name == "tex_width" || name == "tex_height" ||
         name == "tex_data" || name == "tex_colorspace" || name == "ntexdata" ||
         name == "nbuffer";
}

// the first field outside the texture payload which differs between two
// models, "" if none
std::string FirstDifference(const mjModel* a, const mjModel* b) {
  MJMODEL_POINTERS_PREAMBLE(a);
#define X(name) \
  if (!IsPayloadField(#name) && a->name != b->name) return #name;
  MJMODEL_SIZES
#undef X
#define X(type, name, nr, nc)                                     \
  if (!IsPayloadField(#name) &&                                   \
      std::memcmp(a->name, b->name, sizeof(type) * a->nr * nc)) { \
    return #name;                                                 \
  }
  MJMODEL_POINTERS
#undef X
  return "";
}

// a texture i of m is by reference: no payload, a path which opens
void ExpectByReference(const mjModel* m, int i) {
  EXPECT_EQ(m->tex_adr[i], -1) << "texture " << i;
  EXPECT_EQ(m->tex_width[i], 0) << "texture " << i;
  EXPECT_EQ(m->tex_height[i], 0) << "texture " << i;
  EXPECT_GE(m->tex_pathadr[i], 0) << "texture " << i;
  EXPECT_TRUE(Opens(PathAt(m, m->tex_pathadr[i]))) << "texture " << i;
}

// a texture i of m is embedded: pixels in the model
void ExpectEmbedded(const mjModel* m, int i) {
  EXPECT_GE(m->tex_adr[i], 0) << "texture " << i;
  EXPECT_GT(m->tex_width[i], 0) << "texture " << i;
  EXPECT_GT(m->tex_height[i], 0) << "texture " << i;
  EXPECT_LE(m->tex_adr[i] + static_cast<mjtSize>(m->tex_nchannel[i]) *
                                m->tex_width[i] * m->tex_height[i],
            m->ntexdata)
      << "texture " << i;
}

// a scene with file textures: PNG and KTX, 2d and cube, with and without a
// content type, under a texture directory
constexpr char kSceneXml[] = R"(
<mujoco>
  <compiler texturedir="textures"/>
  <asset>
    <texture name="png" type="2d" file="hfield.png" colorspace="sRGB"/>
    <texture name="ktx" type="2d" file="ibl.ktx" content_type="image/ktx"
      colorspace="linear"/>
    <texture name="cube" type="cube" file="hfield.png" colorspace="linear"/>
    <texture name="auto" type="2d" file="hfield.png"/>
    <texture name="checker" type="2d" builtin="checker" width="4" height="4"/>
    <material name="png" texture="png"/>
    <material name="ktx" texture="ktx"/>
    <material name="cube" texture="cube"/>
    <material name="auto" texture="auto"/>
    <material name="checker" texture="checker"/>
  </asset>
  <worldbody>
    <geom type="plane" size="1 1 .1" material="png"/>
    <geom size=".1" pos="0 0 .5" material="ktx"/>
    <geom size=".1" pos="1 0 .5" material="cube"/>
    <geom size=".1" pos="0 1 .5" material="auto"/>
    <geom size=".1" pos="1 1 .5" material="checker"/>
  </worldbody>
</mujoco>
)";

class TextureByReferenceTest : public MujocoTest {
 protected:
  // write the scene's files under a fresh directory
  void SetUp() override {
    MujocoTest::SetUp();
    // one directory per test: ctest runs the tests of a file in parallel
    const testing::TestInfo* test =
        testing::UnitTest::GetInstance()->current_test_info();
    root_ = std::filesystem::path(testing::TempDir()) /
            (std::string("by_reference_") + test->test_suite_name() + "_" +
             test->name());
    std::filesystem::remove_all(root_);
    WriteFile(root_ / "textures", "hfield.png",
              ReadTestData("user/testdata/hfield.png"));
    WriteFile(root_ / "textures", "ibl.ktx",
              ReadTestData("render/testdata/ibl.ktx"));
  }

  void TearDown() override {
    std::filesystem::remove_all(root_);
    MujocoTest::TearDown();
  }

  // parse xml as a model file in the scene directory
  mjSpec* Parse(const std::string& xml) {
    std::array<char, 1024> error;
    mjSpec* spec =
        mj_parseXMLString(xml.c_str(), nullptr, error.data(), error.size());
    EXPECT_THAT(spec, NotNull()) << error.data();
    if (spec) mjs_setString(spec->modelfiledir, root_.string().c_str());
    return spec;
  }

  // compile xml with the given policy
  mjModel* Compile(const std::string& xml, mjtPassthrough passthrough,
                   const mjVFS* vfs = nullptr) {
    mjSpec* spec = Parse(xml);
    if (!spec) return nullptr;
    spec->compiler.passthrough = passthrough;
    mjModel* m = mj_compile(spec, vfs);
    error_ = m ? "" : mjs_getError(spec);
    mj_deleteSpec(spec);
    return m;
  }

  std::filesystem::path root_;
  std::string error_;
};

// the same spec compiled embedded and by reference differs only in the payload
TEST_F(TextureByReferenceTest, OnlyPayloadDiffers) {
  mjModel* embedded = Compile(kSceneXml, mjPASSTHROUGH_NONE);
  ASSERT_THAT(embedded, NotNull()) << error_;
  mjModel* byref = Compile(kSceneXml, mjPASSTHROUGH_TEXTURES);
  ASSERT_THAT(byref, NotNull()) << error_;
  ASSERT_EQ(byref->ntex, 5);

  EXPECT_EQ(FirstDifference(embedded, byref), "");
  EXPECT_GT(embedded->ntexdata, byref->ntexdata);
  EXPECT_GT(embedded->nbuffer, byref->nbuffer);

  // the file textures are by reference, the builtin one stays embedded;
  // nchannel is as authored, 1 for a KTX as in the embedded compile
  int checker = mj_name2id(byref, mjOBJ_TEXTURE, "checker");
  for (int i = 0; i < byref->ntex; i++) {
    if (i == checker) {
      ExpectEmbedded(byref, i);
    } else {
      ExpectByReference(byref, i);
      EXPECT_EQ(PathAt(byref, byref->tex_pathadr[i]),
                PathAt(embedded, embedded->tex_pathadr[i]));
    }
    EXPECT_EQ(byref->tex_nchannel[i], embedded->tex_nchannel[i]);
    EXPECT_EQ(PathAt(byref, byref->tex_contenttypeadr[i]),
              PathAt(embedded, embedded->tex_contenttypeadr[i]));
  }
  EXPECT_EQ(byref->ntexdata, 3 * 4 * 4);

  // the content type is as authored
  int ktx = mj_name2id(byref, mjOBJ_TEXTURE, "ktx");
  int png = mj_name2id(byref, mjOBJ_TEXTURE, "png");
  EXPECT_EQ(PathAt(byref, byref->tex_contenttypeadr[ktx]), "image/ktx");
  EXPECT_EQ(byref->tex_contenttypeadr[png], -1);

  // colorspace "auto" stays auto by reference, embedded resolves it
  int automatic = mj_name2id(byref, mjOBJ_TEXTURE, "auto");
  EXPECT_EQ(byref->tex_colorspace[automatic], mjCOLORSPACE_AUTO);
  EXPECT_NE(embedded->tex_colorspace[automatic], mjCOLORSPACE_AUTO);
  EXPECT_EQ(byref->tex_colorspace[png], mjCOLORSPACE_SRGB);

  // validation accepts the by-reference addresses
  EXPECT_THAT(mj_validateReferences(byref), IsNull());

  mj_deleteModel(embedded);
  mj_deleteModel(byref);
}

// embed="true" keeps the pixels under the policy, embed="false" leaves the
// file by reference without it
TEST_F(TextureByReferenceTest, EmbedOverridesPolicy) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler texturedir="textures"/>
    <asset>
      <texture name="keep" type="2d" file="hfield.png" embed="true"/>
      <texture name="drop" type="2d" file="hfield.png" embed="false"/>
      <texture name="follow" type="2d" file="hfield.png"/>
    </asset>
  </mujoco>
  )";

  for (mjtPassthrough policy : {mjPASSTHROUGH_NONE, mjPASSTHROUGH_TEXTURES}) {
    mjModel* m = Compile(xml, policy);
    ASSERT_THAT(m, NotNull()) << error_;
    ExpectEmbedded(m, mj_name2id(m, mjOBJ_TEXTURE, "keep"));
    ExpectByReference(m, mj_name2id(m, mjOBJ_TEXTURE, "drop"));
    int follow = mj_name2id(m, mjOBJ_TEXTURE, "follow");
    if (policy == mjPASSTHROUGH_TEXTURES) {
      ExpectByReference(m, follow);
    } else {
      ExpectEmbedded(m, follow);
    }
    mj_deleteModel(m);
  }
}

// the textures which cannot be by reference, with the reason given for them
struct Ineligible {
  const char* texture;
  const char* reason;
};

constexpr Ineligible kIneligible[] = {
    {R"(<texture name="t" type="2d" builtin="checker" width="4" height="4")",
     "builtin"},
    {R"(<texture name="t" type="cube" fileright="hfield.png")", "cube files"},
    {R"(<texture name="t" type="2d" file="hfield.png" hflip="true")",
     "flipped"},
    {R"(<texture name="t" type="2d" file="hfield.png" vflip="true")",
     "flipped"},
    {R"(<texture name="t" type="cube" file="hfield.png" gridsize="1 1"
          gridlayout="F")",
     nullptr},  // a 1x1 grid is no grid
    {R"(<texture name="t" type="cube" file="hfield.png" gridsize="2 2"
          gridlayout="FBLR")",
     "grid"},
};

std::string IneligibleXml(const char* texture, const char* embed_value) {
  return std::string(R"(
  <mujoco>
    <compiler texturedir="textures"/>
    <asset>
      )") +
         texture + " embed=\"" + embed_value + "\"/>\n</asset>\n</mujoco>";
}

// embed="false" on a texture which cannot be by reference is an error which
// names the texture and the reason; under the policy, "auto" embeds it
TEST_F(TextureByReferenceTest, IneligibleTextures) {
  for (const Ineligible& c : kIneligible) {
    mjModel* m = Compile(IneligibleXml(c.texture, "false"), mjPASSTHROUGH_NONE);
    if (!c.reason) {
      ASSERT_THAT(m, NotNull()) << error_;
      ExpectByReference(m, 0);
      mj_deleteModel(m);
      continue;
    }
    EXPECT_THAT(m, IsNull()) << c.texture;
    EXPECT_THAT(error_, HasSubstr("texture cannot be by reference"));
    EXPECT_THAT(error_, HasSubstr(c.reason));
    EXPECT_THAT(error_, HasSubstr("Element name 't'"));

    m = Compile(IneligibleXml(c.texture, "auto"), mjPASSTHROUGH_TEXTURES);
    ASSERT_THAT(m, NotNull()) << error_;
    ExpectEmbedded(m, 0);
    mj_deleteModel(m);
  }
}

// a texture given as a buffer stays embedded, and embed="false" on it errors
TEST_F(TextureByReferenceTest, BufferTexture) {
  for (mjtEmbed embed : {mjEMBED_AUTO, mjEMBED_FALSE}) {
    mjSpec* spec = mj_makeSpec();
    spec->compiler.passthrough = mjPASSTHROUGH_TEXTURES;
    mjsTexture* texture = mjs_addTexture(spec, nullptr);
    mjs_setName(texture->element, "buffer");
    texture->type = mjTEXTURE_2D;
    texture->width = 2;
    texture->height = 1;
    texture->embed = embed;
    std::vector<std::byte> pixels(6, std::byte{7});
    mjs_setBuffer(texture->data, pixels.data(), pixels.size());
    mjModel* m = mj_compile(spec, nullptr);
    if (embed == mjEMBED_AUTO) {
      ASSERT_THAT(m, NotNull()) << mjs_getError(spec);
      ExpectEmbedded(m, 0);
      mj_deleteModel(m);
    } else {
      EXPECT_THAT(m, IsNull());
      EXPECT_THAT(mjs_getError(spec), HasSubstr("buffer"));
    }
    mj_deleteSpec(spec);
  }
}

// a file served from memory by the VFS has no lifetime beyond compilation: it
// stays embedded under the policy and is an error under embed="false"
TEST_F(TextureByReferenceTest, VfsServedStaysEmbedded) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <texture name="memory" type="2d" file="memory.png"/>
      <texture name="disk" type="2d" file="textures/hfield.png"/>
    </asset>
  </mujoco>
  )";
  static constexpr char forced[] = R"(
  <mujoco>
    <asset>
      <texture name="memory" type="2d" file="memory.png" embed="false"/>
    </asset>
  </mujoco>
  )";

  std::string png = ReadTestData("user/testdata/hfield.png");
  auto vfs = std::make_unique<mjVFS>();
  mj_defaultVFS(vfs.get());
  mj_addBufferVFS(vfs.get(), "memory.png", png.data(), png.size());

  mjModel* m = Compile(xml, mjPASSTHROUGH_TEXTURES, vfs.get());
  ASSERT_THAT(m, NotNull()) << error_;
  ExpectEmbedded(m, mj_name2id(m, mjOBJ_TEXTURE, "memory"));
  ExpectByReference(m, mj_name2id(m, mjOBJ_TEXTURE, "disk"));
  mj_deleteModel(m);

  m = Compile(forced, mjPASSTHROUGH_NONE, vfs.get());
  EXPECT_THAT(m, IsNull());
  EXPECT_THAT(error_, HasSubstr("served from memory by the VFS"));
  EXPECT_THAT(error_, HasSubstr("Element name 'memory'"));

  // the same file on disk, with no VFS, is by reference
  WriteFile(root_, "memory.png", png);
  m = Compile(forced, mjPASSTHROUGH_NONE);
  ASSERT_THAT(m, NotNull()) << error_;
  ExpectByReference(m, 0);
  mj_deleteModel(m);
  mj_deleteVFS(vfs.get());
}

// with strippath, a by-reference texture of a resource provider keeps its name,
// so its path opens through the provider, and a VFS file with the stripped name
// does not make it look served from memory
TEST_F(TextureByReferenceTest, StrippathKeepsProviderTexture) {
  static constexpr char kName[] = "texreftest://dir/hfield.png";
  static const std::string png = ReadTestData("user/testdata/hfield.png");
  mjpResourceProvider provider = {
      .prefix = "texreftest",
      .open = [](mjResource* resource) -> int {
        return std::strcmp(resource->name, kName) == 0;
      },
      .read = [](mjResource*, const void** buffer) -> int {
        *buffer = png.data();
        return png.size();
      },
      .close = [](mjResource*) {},
  };
  ASSERT_GT(mjp_registerProvider(&provider), 0);

  static constexpr char xml[] = R"(
  <mujoco>
    <compiler strippath="true"/>
    <asset>
      <texture name="png" type="2d" file="texreftest://dir/hfield.png"/>
    </asset>
  </mujoco>
  )";
  mjModel* m = Compile(xml, mjPASSTHROUGH_TEXTURES);
  ASSERT_THAT(m, NotNull()) << error_;
  ExpectByReference(m, 0);
  EXPECT_EQ(PathAt(m, m->tex_pathadr[0]), kName);
  mj_deleteModel(m);

  auto vfs = std::make_unique<mjVFS>();
  mj_defaultVFS(vfs.get());
  mj_addBufferVFS(vfs.get(), "hfield.png", png.data(), png.size());
  m = Compile(xml, mjPASSTHROUGH_TEXTURES, vfs.get());
  ASSERT_THAT(m, NotNull()) << error_;
  ExpectByReference(m, 0);
  mj_deleteModel(m);
  mj_deleteVFS(vfs.get());
}

// embed is a defaultable attribute: a class sets it, a texture overrides it
TEST_F(TextureByReferenceTest, DefaultClass) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler texturedir="textures" passthrough="textures"/>
    <default>
      <texture embed="true"/>
      <default class="byref">
        <texture embed="false"/>
      </default>
    </default>
    <asset>
      <texture name="main" type="2d" file="hfield.png"/>
      <texture name="byref" class="byref" type="2d" file="hfield.png"/>
      <texture name="override" class="byref" type="2d" file="hfield.png"
        embed="true"/>
    </asset>
  </mujoco>
  )";

  mjSpec* spec = Parse(xml);
  ASSERT_THAT(spec, NotNull());
  EXPECT_EQ(spec->compiler.passthrough, mjPASSTHROUGH_TEXTURES);
  mjModel* m = mj_compile(spec, nullptr);
  ASSERT_THAT(m, NotNull()) << mjs_getError(spec);
  ExpectEmbedded(m, mj_name2id(m, mjOBJ_TEXTURE, "main"));
  ExpectByReference(m, mj_name2id(m, mjOBJ_TEXTURE, "byref"));
  ExpectEmbedded(m, mj_name2id(m, mjOBJ_TEXTURE, "override"));
  mj_deleteModel(m);

  // the policy comes from the XML; the defaults survive a save and reload
  EXPECT_EQ(mjs_findDefault(spec, "byref")->texture->embed, mjEMBED_FALSE);
  EXPECT_EQ(mjs_findDefault(spec, "main")->texture->embed, mjEMBED_TRUE);
  std::array<char, 1024> error;
  std::string saved(1 << 16, '\0');
  ASSERT_GE(mj_saveXMLString(spec, saved.data(), saved.size(), error.data(),
                             error.size()),
            0)
      << error.data();
  EXPECT_THAT(saved, HasSubstr("passthrough=\"textures\""));
  EXPECT_THAT(saved, HasSubstr("embed=\"true\""));
  EXPECT_THAT(saved, HasSubstr("embed=\"false\""));
  mj_deleteSpec(spec);
}

// the policy is that of the model being compiled: a parent applies it to the
// textures of an attached child, and a parent without it keeps them embedded
TEST_F(TextureByReferenceTest, AttachFollowsCompilingModel) {
  static constexpr char child_xml[] = R"(
  <mujoco>
    <compiler texturedir="textures"/>
    <asset>
      <texture name="tex" type="2d" file="hfield.png"/>
      <material name="mat" texture="tex"/>
    </asset>
    <worldbody>
      <body name="box">
        <geom size=".1" material="mat"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  static constexpr char parent_xml[] = R"(
  <mujoco>
    <asset>
      <texture name="own" type="2d" file="textures/hfield.png"/>
    </asset>
  </mujoco>
  )";

  for (int parent_policy = 0; parent_policy < 2; parent_policy++) {
    mjSpec* parent = Parse(parent_xml);
    mjSpec* child = Parse(child_xml);
    ASSERT_THAT(parent, NotNull());
    ASSERT_THAT(child, NotNull());
    parent->compiler.passthrough =
        parent_policy ? mjPASSTHROUGH_TEXTURES : mjPASSTHROUGH_NONE;
    child->compiler.passthrough =
        parent_policy ? mjPASSTHROUGH_NONE : mjPASSTHROUGH_TEXTURES;
    mjsFrame* frame = mjs_addFrame(mjs_findBody(parent, "world"), nullptr);
    mjsBody* box = mjs_findBody(child, "box");
    ASSERT_THAT(mjs_attach(frame->element, box->element, "c_", ""), NotNull())
        << mjs_getError(parent);

    mjModel* m = mj_compile(parent, nullptr);
    ASSERT_THAT(m, NotNull()) << mjs_getError(parent);
    ASSERT_EQ(m->ntex, 2);
    for (int i = 0; i < m->ntex; i++) {
      if (parent_policy) {
        ExpectByReference(m, i);
      } else {
        ExpectEmbedded(m, i);
      }
    }
    mj_deleteModel(m);
    mj_deleteSpec(parent);
    mj_deleteSpec(child);
  }
}

// an MJB keeps by-reference textures as references; copying works
TEST_F(TextureByReferenceTest, MjbRoundTripAndCopy) {
  mjModel* m = Compile(kSceneXml, mjPASSTHROUGH_TEXTURES);
  ASSERT_THAT(m, NotNull()) << error_;

  std::vector<char> mjb(mj_sizeModel(m));
  mj_saveModel(m, nullptr, mjb.data(), mjb.size());
  mjModel* loaded = mj_loadModelBuffer(mjb.data(), mjb.size());
  ASSERT_THAT(loaded, NotNull());
  EXPECT_THAT(mj_validateReferences(loaded), IsNull());
  std::string field;
  EXPECT_EQ(CompareModel(m, loaded, field), 0) << field;
  for (int i = 0; i < loaded->ntex; i++) {
    if (loaded->tex_adr[i] == -1) ExpectByReference(loaded, i);
  }

  mjModel* copy = mj_copyModel(nullptr, m);
  ASSERT_THAT(copy, NotNull());
  EXPECT_EQ(CompareModel(m, copy, field), 0) << field;

  // a genuinely bad address still fails validation
  int png = mj_name2id(copy, mjOBJ_TEXTURE, "png");
  copy->tex_adr[png] = -2;
  EXPECT_THAT(mj_validateReferences(copy), HasSubstr("tex_adr"));

  // a by-reference texture must have a path
  copy->tex_adr[png] = -1;
  copy->tex_pathadr[png] = -1;
  EXPECT_THAT(mj_validateReferences(copy), HasSubstr("has no path"));

  mj_deleteModel(copy);
  mj_deleteModel(loaded);
  mj_deleteModel(m);
}

// recompiling after toggling the policy switches the representation, saving
// as XML writes the authored file and the policy, and reloading gives the same
// model; the spec is not modified by compilation
TEST_F(TextureByReferenceTest, RecompileAndSaveXml) {
  mjSpec* spec = Parse(kSceneXml);
  ASSERT_THAT(spec, NotNull());
  spec->compiler.passthrough = mjPASSTHROUGH_TEXTURES;
  mjModel* m = mj_compile(spec, nullptr);
  ASSERT_THAT(m, NotNull()) << mjs_getError(spec);
  mjData* d = mj_makeData(m);
  int png = mj_name2id(m, mjOBJ_TEXTURE, "png");
  ExpectByReference(m, png);

  // the spec texture is as authored
  mjsTexture* texture =
      mjs_asTexture(mjs_findElement(spec, mjOBJ_TEXTURE, "png"));
  ASSERT_THAT(texture, NotNull());
  EXPECT_EQ(mjs_getString(texture->file), std::string("hfield.png"));
  EXPECT_EQ(texture->embed, mjEMBED_AUTO);
  EXPECT_EQ(texture->width, 0);
  EXPECT_EQ(texture->height, 0);

  // recompile embedded, then by reference again
  spec->compiler.passthrough = mjPASSTHROUGH_NONE;
  ASSERT_EQ(mj_recompile(spec, nullptr, m, d), 0) << mjs_getError(spec);
  ExpectEmbedded(m, png);
  EXPECT_GT(m->ntexdata, 3 * 4 * 4);
  spec->compiler.passthrough = mjPASSTHROUGH_TEXTURES;
  ASSERT_EQ(mj_recompile(spec, nullptr, m, d), 0) << mjs_getError(spec);
  ExpectByReference(m, png);
  EXPECT_EQ(m->ntexdata, 3 * 4 * 4);
  EXPECT_EQ(mjs_getString(texture->file), std::string("hfield.png"));

  // saved XML writes the authored file and the policy, and reloads to the same
  // model
  std::string saved = SaveAndReadXml(spec);
  EXPECT_THAT(saved, HasSubstr("passthrough=\"textures\""));
  EXPECT_THAT(saved, HasSubstr("file=\"hfield.png\""));
  EXPECT_THAT(saved, testing::Not(HasSubstr("embed=")));
  mjSpec* reloaded = Parse(saved);
  ASSERT_THAT(reloaded, NotNull());
  EXPECT_EQ(reloaded->compiler.passthrough, mjPASSTHROUGH_TEXTURES);
  mjModel* m2 = mj_compile(reloaded, nullptr);
  ASSERT_THAT(m2, NotNull()) << mjs_getError(reloaded);
  std::string field;
  EXPECT_EQ(CompareModel(m, m2, field), 0) << field;

  mj_deleteModel(m2);
  mj_deleteSpec(reloaded);
  mj_deleteData(d);
  mj_deleteModel(m);
  mj_deleteSpec(spec);
}

// the defaults of the new fields
TEST_F(TextureByReferenceTest, Defaults) {
  mjSpec* spec = mj_makeSpec();
  EXPECT_EQ(spec->compiler.passthrough, mjPASSTHROUGH_NONE);
  mjsTexture* texture = mjs_addTexture(spec, nullptr);
  EXPECT_EQ(texture->embed, mjEMBED_AUTO);
  mj_deleteSpec(spec);
}

// ------------------------------ mj_loadTexture -------------------------------

// the pixels of texture i, as mj_loadTexture returns them
struct Pixels {
  std::vector<mjtByte> data;
  int width = 0, height = 0, nchannel = 0, colorspace = -1;
  std::string error;
};

Pixels LoadTexture(const mjModel* m, int i) {
  Pixels p;
  std::array<char, 1024> error;
  mjtByte* data = nullptr;
  if (mj_loadTexture(m, i, &data, &p.width, &p.height, &p.colorspace,
                     error.data(), error.size())) {
    p.error = error.data();
    return p;
  }
  p.nchannel = m->tex_nchannel[i];
  p.data.assign(data,
                data + static_cast<mjtSize>(p.width) * p.height * p.nchannel);
  mju_free(data);
  return p;
}

// the pixels of embedded texture i, as stored in the model
Pixels Slice(const mjModel* m, int i) {
  Pixels p;
  p.width = m->tex_width[i];
  p.height = m->tex_height[i];
  p.nchannel = m->tex_nchannel[i];
  p.colorspace = m->tex_colorspace[i];
  const mjtByte* data = m->tex_data + m->tex_adr[i];
  p.data.assign(data,
                data + static_cast<mjtSize>(p.width) * p.height * p.nchannel);
  return p;
}

void ExpectSamePixels(const Pixels& a, const Pixels& b, int i) {
  EXPECT_EQ(a.error, "") << "texture " << i;
  EXPECT_EQ(a.width, b.width) << "texture " << i;
  EXPECT_EQ(a.height, b.height) << "texture " << i;
  EXPECT_EQ(a.nchannel, b.nchannel) << "texture " << i;
  EXPECT_EQ(a.colorspace, b.colorspace) << "texture " << i;
  EXPECT_EQ(a.data.size(), b.data.size()) << "texture " << i;
  EXPECT_TRUE(a.data == b.data) << "texture " << i;
}

// every PNG and KTX under the test models, in the ways a single file can be
// authored: 2d with 1, 3 and 4 channels, cube and skybox, colorspace given and
// inferred from the file, content type given and inferred from the extension
constexpr char kLoadXml[] = R"(
<mujoco>
  <compiler texturedir="textures"/>
  <asset>
    <texture name="rgba3" type="2d" file="hfield.png"/>
    <texture name="rgba4" type="2d" file="hfield.png" nchannel="4"/>
    <texture name="rgba1" type="2d" file="hfield.png" nchannel="1"/>
    <texture name="palette" type="2d" file="test-pattern.png"/>
    <texture name="srgb" type="2d" file="carpet.png" nchannel="4"/>
    <texture name="srgblinear" type="2d" file="carpet.png" colorspace="linear"/>
    <texture name="ktx" type="2d" file="ibl.ktx" content_type="image/ktx"/>
    <texture name="ktxext" type="2d" file="ibl.ktx" colorspace="sRGB"/>
    <texture name="cube" type="cube" file="hfield.png"/>
    <texture name="skybox" type="skybox" file="hfield.png" colorspace="sRGB"/>
    <texture name="checker" type="2d" builtin="checker" width="4" height="4"/>
  </asset>
</mujoco>
)";

class LoadTextureTest : public TextureByReferenceTest {
 protected:
  void SetUp() override {
    TextureByReferenceTest::SetUp();
    WriteFile(root_ / "textures", "test-pattern.png",
              ReadTestData("render/testdata/test-pattern.png"));
    WriteFile(root_ / "textures", "carpet.png",
              ReadTestData("user/testdata/textured_torus/carpet.png"));
  }
};

// mj_loadTexture on a by-reference texture gives exactly what the eager compile
// stores, including the resolved colorspace; on an embedded texture, a copy
TEST_F(LoadTextureTest, MatchesEagerCompile) {
  mjModel* eager = Compile(kLoadXml, mjPASSTHROUGH_NONE);
  ASSERT_THAT(eager, NotNull()) << error_;
  mjModel* byref = Compile(kLoadXml, mjPASSTHROUGH_TEXTURES);
  ASSERT_THAT(byref, NotNull()) << error_;
  ASSERT_EQ(byref->ntex, 11);
  EXPECT_EQ(byref->ntexdata, 3 * 4 * 4);

  for (int i = 0; i < byref->ntex; i++) {
    ExpectSamePixels(LoadTexture(byref, i), Slice(eager, i), i);
    ExpectSamePixels(LoadTexture(eager, i), Slice(eager, i), i);
  }

  // the decoder resolved what the model left as auto
  int palette = mj_name2id(byref, mjOBJ_TEXTURE, "palette");
  int ktx = mj_name2id(byref, mjOBJ_TEXTURE, "ktx");
  int rgba3 = mj_name2id(byref, mjOBJ_TEXTURE, "rgba3");
  EXPECT_EQ(byref->tex_colorspace[palette], mjCOLORSPACE_AUTO);
  EXPECT_EQ(LoadTexture(byref, palette).colorspace, mjCOLORSPACE_SRGB);
  EXPECT_EQ(LoadTexture(byref, rgba3).colorspace, mjCOLORSPACE_LINEAR);
  EXPECT_EQ(LoadTexture(byref, ktx).colorspace, mjCOLORSPACE_LINEAR);
  EXPECT_EQ(byref->tex_nchannel[ktx], 1);
  EXPECT_EQ(LoadTexture(byref, ktx).height, 1);

  // the out-parameters other than the pixels may be left out
  mjtByte* data = nullptr;
  int w = 0;
  EXPECT_EQ(
      mj_loadTexture(byref, palette, &data, &w, nullptr, nullptr, nullptr, 0),
      0);
  EXPECT_EQ(w, 256);
  mju_free(data);

  mj_deleteModel(byref);
  mj_deleteModel(eager);
}

// the errors: a bad id, a file that does not resolve, a content type the
// decoder does not know, a file the decoder cannot read, a file which cannot be
// a cube
TEST_F(LoadTextureTest, Errors) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler texturedir="textures"/>
    <asset>
      <texture name="png" type="2d" file="hfield.png"/>
      <texture name="jpeg" type="2d" file="hfield.png" content_type="image/jpeg"/>
      <texture name="corrupt" type="2d" file="corrupt.png"/>
      <texture name="ktxcube" type="cube" file="ibl.ktx"/>
    </asset>
  </mujoco>
  )";
  WriteFile(root_ / "textures", "corrupt.png", "not a png");
  WriteFile(root_ / "textures", "hfield.pnx",
            ReadTestData("user/testdata/hfield.png"));
  mjModel* m = Compile(xml, mjPASSTHROUGH_TEXTURES);
  ASSERT_THAT(m, NotNull()) << error_;
  int png = mj_name2id(m, mjOBJ_TEXTURE, "png");
  EXPECT_EQ(LoadTexture(m, png).error, "");

  EXPECT_THAT(LoadTexture(m, -1).error, HasSubstr("out of range"));
  EXPECT_THAT(LoadTexture(m, m->ntex).error, HasSubstr("out of range"));
  EXPECT_THAT(LoadTexture(m, mj_name2id(m, mjOBJ_TEXTURE, "jpeg")).error,
              HasSubstr("unsupported content type: 'image/jpeg'"));
  EXPECT_THAT(LoadTexture(m, mj_name2id(m, mjOBJ_TEXTURE, "corrupt")).error,
              HasSubstr("error decoding PNG"));
  EXPECT_THAT(LoadTexture(m, mj_name2id(m, mjOBJ_TEXTURE, "ktxcube")).error,
              HasSubstr("must have 3 channels"));

  // the path is a string in the model: a file that is not there, then an
  // extension the decoder does not know
  char* path = m->paths + m->tex_pathadr[png];
  char* name = path + std::strlen(path) - std::strlen("hfield.png");
  name[0] = 'x';
  EXPECT_THAT(LoadTexture(m, png).error, HasSubstr("xfield.png"));
  name[0] = 'h';
  name[std::strlen("hfield.pn")] = 'x';
  EXPECT_THAT(
      LoadTexture(m, png).error,
      AllOf(HasSubstr("unsupported content type"), HasSubstr("hfield.pnx")));

  // a null error buffer is accepted
  mjtByte* data = nullptr;
  int w, h, c;
  EXPECT_NE(mj_loadTexture(m, png, &data, &w, &h, &c, nullptr, 0), 0);
  EXPECT_THAT(data, IsNull());

  mj_deleteModel(m);
}

// ------------------------------ mj_embedAssets -------------------------------

// embedding a by-reference compile gives the eager compile, field for field;
// the result saves, loads and validates; an all-embedded model gives a plain
// copy
TEST_F(LoadTextureTest, EmbedAssetsEqualsEagerCompile) {
  mjModel* eager = Compile(kLoadXml, mjPASSTHROUGH_NONE);
  ASSERT_THAT(eager, NotNull()) << error_;
  mjModel* byref = Compile(kLoadXml, mjPASSTHROUGH_TEXTURES);
  ASSERT_THAT(byref, NotNull()) << error_;
  ASSERT_GT(eager->ntexdata, byref->ntexdata);

  std::array<char, 1024> error;
  mjModel* embedded = mj_embedAssets(byref, error.data(), error.size());
  ASSERT_THAT(embedded, NotNull()) << error.data();
  std::string field;
  EXPECT_EQ(CompareModel(eager, embedded, field), 0) << field;
  EXPECT_EQ(embedded->ntexdata, eager->ntexdata);
  EXPECT_EQ(embedded->nbuffer, eager->nbuffer);
  EXPECT_EQ(mj_sizeModel(embedded), mj_sizeModel(eager));
  EXPECT_THAT(mj_validateReferences(embedded), IsNull());
  for (int i = 0; i < embedded->ntex; i++) ExpectEmbedded(embedded, i);

  // the by-reference model is untouched
  mjModel* byref2 = Compile(kLoadXml, mjPASSTHROUGH_TEXTURES);
  EXPECT_EQ(CompareModel(byref, byref2, field), 0) << field;

  // save and load
  std::vector<char> mjb(mj_sizeModel(embedded));
  mj_saveModel(embedded, nullptr, mjb.data(), mjb.size());
  mjModel* loaded = mj_loadModelBuffer(mjb.data(), mjb.size());
  ASSERT_THAT(loaded, NotNull());
  EXPECT_THAT(mj_validateReferences(loaded), IsNull());
  EXPECT_EQ(CompareModel(eager, loaded, field), 0) << field;

  // an all-embedded model is copied
  mjModel* copy = mj_embedAssets(eager, error.data(), error.size());
  ASSERT_THAT(copy, NotNull()) << error.data();
  EXPECT_EQ(CompareModel(eager, copy, field), 0) << field;
  EXPECT_NE(copy->buffer, eager->buffer);

  mj_deleteModel(copy);
  mj_deleteModel(loaded);
  mj_deleteModel(byref2);
  mj_deleteModel(embedded);
  mj_deleteModel(byref);
  mj_deleteModel(eager);
}

// a file that does not resolve is an error naming the texture, and nothing is
// allocated
TEST_F(LoadTextureTest, EmbedAssetsMissingFile) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler texturedir="textures"/>
    <asset>
      <texture name="png" type="2d" file="hfield.png"/>
      <texture name="missing" type="2d" file="missing.png"/>
    </asset>
  </mujoco>
  )";
  mjModel* m = Compile(xml, mjPASSTHROUGH_TEXTURES);
  ASSERT_THAT(m, NotNull()) << error_;

  std::array<char, 1024> error;
  EXPECT_THAT(mj_embedAssets(m, error.data(), error.size()), IsNull());
  EXPECT_THAT(error.data(), HasSubstr("texture 1: "));
  EXPECT_THAT(error.data(), HasSubstr("missing.png"));
  EXPECT_THAT(mj_embedAssets(m, nullptr, 0), IsNull());

  // with the file in place it succeeds
  WriteFile(root_ / "textures", "missing.png",
            ReadTestData("user/testdata/hfield.png"));
  mjModel* embedded = mj_embedAssets(m, error.data(), error.size());
  ASSERT_THAT(embedded, NotNull()) << error.data();
  EXPECT_EQ(embedded->ntexdata, 2 * 3 * 32 * 32);
  mj_deleteModel(embedded);
  mj_deleteModel(m);
}

// textures which differ only in embed compile to different representations,
// so mjs_dedupAssets does not merge them
TEST_F(TextureByReferenceTest, DedupKeepsDifferentEmbed) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <texture name="a" type="2d" builtin="checker" width="8" height="8"/>
      <texture name="b" type="2d" builtin="checker" width="8" height="8"/>
      <texture name="c" type="2d" builtin="checker" width="8" height="8"
               embed="true"/>
    </asset>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjSpec* spec = mj_parseXMLString(xml, 0, error.data(), error.size());
  ASSERT_THAT(spec, NotNull()) << error.data();
  EXPECT_EQ(mjs_dedupAssets(spec), 1) << mjs_getError(spec);
  EXPECT_THAT(mjs_findElement(spec, mjOBJ_TEXTURE, "a"), NotNull());
  EXPECT_THAT(mjs_findElement(spec, mjOBJ_TEXTURE, "b"), IsNull());
  EXPECT_THAT(mjs_findElement(spec, mjOBJ_TEXTURE, "c"), NotNull());
  mj_deleteSpec(spec);
}

}  // namespace
}  // namespace mujoco
