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

#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <future>
#include <ios>
#include <iterator>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <miniz.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using MjzArchiveProviderTest = MujocoTest;

using ::testing::Gt;
using ::testing::IsNull;
using ::testing::NotNull;

TEST_F(MjzArchiveProviderTest, ProviderIsRegistered) {
  const mjpResourceProvider* prov_mjz =
      mjp_findArchiveResourceProvider("model.mjz");
  ASSERT_THAT(prov_mjz, NotNull());
  EXPECT_THAT(prov_mjz->open, NotNull());
  EXPECT_THAT(prov_mjz->read, NotNull());
  EXPECT_THAT(prov_mjz->close, NotNull());
  EXPECT_THAT(prov_mjz->mount, NotNull());
  EXPECT_THAT(prov_mjz->unmount, NotNull());

  const mjpResourceProvider* prov_zip =
      mjp_findArchiveResourceProvider("robot.zip");
  ASSERT_THAT(prov_zip, NotNull());

  const mjpResourceProvider* prov_xml =
      mjp_findArchiveResourceProvider("model.xml");
  EXPECT_THAT(prov_xml, IsNull());
}

TEST_F(MjzArchiveProviderTest, OpenAndReadDiskArchiveMembers) {
  std::string filepath = GetTestDataFilePath("testdata/model.mjz");
  std::string member_path = filepath + "/model.xml";

  char err[1000] = "";
  mjResource* res =
      mju_openResource("", member_path.c_str(), nullptr, err, sizeof(err));
  ASSERT_THAT(res, NotNull()) << err;

  const void* buffer = nullptr;
  int size = mju_readResource(res, &buffer);
  EXPECT_THAT(size, Gt(0));
  ASSERT_THAT(buffer, NotNull());

  std::string_view xml((const char*)buffer, size);
  EXPECT_TRUE(xml.find("<mujoco") != std::string_view::npos);

  mju_closeResource(res);
}

TEST_F(MjzArchiveProviderTest, OpenNonExistentMemberFails) {
  std::string filepath = GetTestDataFilePath("testdata/model.mjz");
  std::string member_path = filepath + "/nonexistent_file.xml";

  char err[1000] = "";
  mjResource* res =
      mju_openResource("", member_path.c_str(), nullptr, err, sizeof(err));
  EXPECT_THAT(res, IsNull());
}

TEST_F(MjzArchiveProviderTest, OpenAndReadNonDiskArchive) {
  std::string filepath = GetTestDataFilePath("testdata/model.mjz");
  std::ifstream file(filepath, std::ios::binary);
  ASSERT_TRUE(file.is_open());
  std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(file)),
                             std::istreambuf_iterator<char>());
  ASSERT_FALSE(bytes.empty());

  struct CustomProviderData {
    std::vector<uint8_t> archive_bytes;
  };

  static CustomProviderData provider_data;
  provider_data.archive_bytes = std::move(bytes);

  mjpResourceProvider custom_provider;
  mjp_defaultResourceProvider(&custom_provider);
  custom_provider.prefix = "mockprovider";
  custom_provider.data = &provider_data;
  custom_provider.open = [](mjResource* res) -> int {
    if (!res || !res->name) return 0;
    if (std::strstr(res->name, "archive.mjz") != nullptr) {
      res->data = res->provider->data;
      return 1;
    }
    return 0;
  };
  custom_provider.read = [](mjResource* res, const void** buffer) -> int {
    auto* data = static_cast<CustomProviderData*>(res->data);
    if (!data) return -1;
    *buffer = data->archive_bytes.data();
    return static_cast<int>(data->archive_bytes.size());
  };
  custom_provider.close = [](mjResource* res) { res->data = nullptr; };

  mjp_registerResourceProvider(&custom_provider);

  char err[1000] = "";
  std::string member_uri = "mockprovider://data/archive.mjz/model.xml";
  mjResource* res =
      mju_openResource("", member_uri.c_str(), nullptr, err, sizeof(err));
  ASSERT_THAT(res, NotNull()) << err;

  const void* buffer = nullptr;
  int size = mju_readResource(res, &buffer);
  EXPECT_THAT(size, Gt(0));
  ASSERT_THAT(buffer, NotNull());

  std::string_view xml((const char*)buffer, size);
  EXPECT_TRUE(xml.find("<mujoco") != std::string_view::npos);

  mju_closeResource(res);
}

TEST_F(MjzArchiveProviderTest, RelativePathsAndPathReduction) {
  std::string filepath = GetTestDataFilePath("testdata/model.mjz");

  // 1. Path containing redundant dot segments and slashes
  std::string path_with_dots = filepath + "/./subdir/../model.xml";
  char err[1000] = "";
  mjResource* res1 =
      mju_openResource("", path_with_dots.c_str(), nullptr, err, sizeof(err));
  ASSERT_THAT(res1, NotNull()) << err;
  const void* buf1 = nullptr;
  EXPECT_THAT(mju_readResource(res1, &buf1), Gt(0));
  mju_closeResource(res1);

  // 2. Path with redundant duplicate slashes
  std::string path_with_slashes = filepath + "///model.xml";
  mjResource* res2 = mju_openResource("", path_with_slashes.c_str(), nullptr,
                                      err, sizeof(err));
  ASSERT_THAT(res2, NotNull()) << err;
  const void* buf2 = nullptr;
  EXPECT_THAT(mju_readResource(res2, &buf2), Gt(0));
  mju_closeResource(res2);
}

TEST_F(MjzArchiveProviderTest, DotsInDirectoryName) {
  // Verifies that a directory name with dots does not mislead archive extension
  // matching
  std::string filepath = GetTestDataFilePath("testdata/model.mjz");
  std::ifstream file(filepath, std::ios::binary);
  ASSERT_TRUE(file.is_open());
  std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(file)),
                             std::istreambuf_iterator<char>());

  struct DottedProviderData {
    std::vector<uint8_t> archive_bytes;
  };

  static DottedProviderData dotted_data;
  dotted_data.archive_bytes = std::move(bytes);

  mjpResourceProvider dotted_provider;
  mjp_defaultResourceProvider(&dotted_provider);
  dotted_provider.prefix = "dottedprovider";
  dotted_provider.data = &dotted_data;
  dotted_provider.open = [](mjResource* res) -> int {
    if (!res || !res->name) return 0;
    if (std::strstr(res->name, "model.mjz") != nullptr) {
      res->data = res->provider->data;
      return 1;
    }
    return 0;
  };
  dotted_provider.read = [](mjResource* res, const void** buffer) -> int {
    auto* data = static_cast<DottedProviderData*>(res->data);
    if (!data) return -1;
    *buffer = data->archive_bytes.data();
    return static_cast<int>(data->archive_bytes.size());
  };
  dotted_provider.close = [](mjResource* res) { res->data = nullptr; };

  mjp_registerResourceProvider(&dotted_provider);

  char err[1000] = "";
  // Directory with dots "v1.2.3.folder/assets.dir"
  std::string uri =
      "dottedprovider://v1.2.3.folder/assets.dir/model.mjz/model.xml";
  mjResource* res =
      mju_openResource("", uri.c_str(), nullptr, err, sizeof(err));
  ASSERT_THAT(res, NotNull()) << err;

  const void* buffer = nullptr;
  int size = mju_readResource(res, &buffer);
  EXPECT_THAT(size, Gt(0));
  mju_closeResource(res);
}

TEST_F(MjzArchiveProviderTest, CaseInsensitiveExtension) {
  const mjpResourceProvider* prov_upper =
      mjp_findArchiveResourceProvider("model.MJZ");
  EXPECT_THAT(prov_upper, NotNull());

  const mjpResourceProvider* prov_mixed =
      mjp_findArchiveResourceProvider("model.Zip");
  EXPECT_THAT(prov_mixed, NotNull());
}

TEST_F(MjzArchiveProviderTest, BackslashSeparators) {
  std::string filepath = GetTestDataFilePath("testdata/model.mjz");
  std::string member_path = filepath + "\\model.xml";

  char err[1000] = "";
  mjResource* res =
      mju_openResource("", member_path.c_str(), nullptr, err, sizeof(err));
  ASSERT_THAT(res, NotNull()) << err;

  const void* buffer = nullptr;
  int size = mju_readResource(res, &buffer);
  EXPECT_THAT(size, Gt(0));
  ASSERT_THAT(buffer, NotNull());

  std::string_view xml((const char*)buffer, size);
  EXPECT_TRUE(xml.find("<mujoco") != std::string_view::npos);

  mju_closeResource(res);
}

TEST_F(MjzArchiveProviderTest, NestedArchiveMembers) {
  // Build an in-memory inner zip archive
  mz_zip_archive inner_zip;
  std::memset(&inner_zip, 0, sizeof(inner_zip));
  ASSERT_TRUE(mz_zip_writer_init_heap(&inner_zip, 0, 4096));

  const char kInnerXml[] =
      "<mujoco model=\"inner\"><body name=\"b_inner\"/></mujoco>";
  const char kInnerMesh[] = "v 7.0 8.0 9.0\n";
  ASSERT_TRUE(mz_zip_writer_add_mem(&inner_zip, "inner.xml", kInnerXml,
                                    std::strlen(kInnerXml), MZ_BEST_SPEED));
  ASSERT_TRUE(mz_zip_writer_add_mem(&inner_zip, "sub/inner_mesh.obj",
                                    kInnerMesh, std::strlen(kInnerMesh),
                                    MZ_BEST_SPEED));

  void* inner_buf = nullptr;
  size_t inner_sz = 0;
  ASSERT_TRUE(
      mz_zip_writer_finalize_heap_archive(&inner_zip, &inner_buf, &inner_sz));
  ASSERT_THAT(inner_buf, NotNull());
  ASSERT_THAT(inner_sz, Gt(0));

  // Build outer mjz archive containing files and the inner zip archive
  mz_zip_archive zip;
  std::memset(&zip, 0, sizeof(zip));
  ASSERT_TRUE(mz_zip_writer_init_heap(&zip, 0, 4096));

  const char kRootXml[] =
      "<mujoco model=\"nested\"><include file=\"sub/child.xml\"/></mujoco>";
  const char kChildXml[] = "<mujoco><body name=\"b1\"/></mujoco>";
  const char kMeshObj[] = "v 1.0 2.0 3.0\nv 4.0 5.0 6.0\nf 1 2 1\n";

  ASSERT_TRUE(mz_zip_writer_add_mem(&zip, "model.xml", kRootXml,
                                    std::strlen(kRootXml), MZ_BEST_SPEED));
  ASSERT_TRUE(mz_zip_writer_add_mem(&zip, "sub/child.xml", kChildXml,
                                    std::strlen(kChildXml), MZ_BEST_SPEED));
  ASSERT_TRUE(mz_zip_writer_add_mem(&zip, "sub/nested/deep/mesh.obj", kMeshObj,
                                    std::strlen(kMeshObj), MZ_BEST_SPEED));
  ASSERT_TRUE(mz_zip_writer_add_mem(&zip, "inner.zip", inner_buf, inner_sz,
                                    MZ_BEST_SPEED));
  ASSERT_TRUE(mz_zip_writer_add_mem(&zip, "sub/inner.zip", inner_buf, inner_sz,
                                    MZ_BEST_SPEED));

  void* zip_buf = nullptr;
  size_t zip_sz = 0;
  ASSERT_TRUE(mz_zip_writer_finalize_heap_archive(&zip, &zip_buf, &zip_sz));
  ASSERT_THAT(zip_buf, NotNull());
  ASSERT_THAT(zip_sz, Gt(0));

  std::vector<uint8_t> zip_bytes((uint8_t*)zip_buf, (uint8_t*)zip_buf + zip_sz);
  mz_free(zip_buf);
  mz_zip_writer_end(&zip);

  mz_free(inner_buf);
  mz_zip_writer_end(&inner_zip);

  struct NestedProviderData {
    std::vector<uint8_t> bytes;
  };

  static NestedProviderData nested_data;
  nested_data.bytes = std::move(zip_bytes);

  mjpResourceProvider nested_provider;
  mjp_defaultResourceProvider(&nested_provider);
  nested_provider.prefix = "nestedprovider";
  nested_provider.data = &nested_data;
  nested_provider.open = [](mjResource* res) -> int {
    if (!res || !res->name) return 0;
    if (std::strstr(res->name, "nested.mjz") != nullptr) {
      res->data = res->provider->data;
      return 1;
    }
    return 0;
  };
  nested_provider.read = [](mjResource* res, const void** buffer) -> int {
    auto* data = static_cast<NestedProviderData*>(res->data);
    if (!data) return -1;
    *buffer = data->bytes.data();
    return static_cast<int>(data->bytes.size());
  };
  nested_provider.close = [](mjResource* res) { res->data = nullptr; };

  mjp_registerResourceProvider(&nested_provider);

  // Read nested child
  char err[1000] = "";
  std::string child_uri = "nestedprovider://models/nested.mjz/sub/child.xml";
  mjResource* child_res =
      mju_openResource("", child_uri.c_str(), nullptr, err, sizeof(err));
  ASSERT_THAT(child_res, NotNull()) << err;
  const void* child_buf = nullptr;
  int child_sz = mju_readResource(child_res, &child_buf);
  EXPECT_THAT(child_sz, Gt(0));
  EXPECT_TRUE(std::string_view((const char*)child_buf, child_sz)
                  .find("body name=\"b1\"") != std::string_view::npos);
  mju_closeResource(child_res);

  // Read deeply nested mesh
  std::string mesh_uri =
      "nestedprovider://models/nested.mjz/sub/nested/deep/mesh.obj";
  mjResource* mesh_res =
      mju_openResource("", mesh_uri.c_str(), nullptr, err, sizeof(err));
  ASSERT_THAT(mesh_res, NotNull()) << err;
  const void* mesh_buf = nullptr;
  int mesh_sz = mju_readResource(mesh_res, &mesh_buf);
  EXPECT_THAT(mesh_sz, Gt(0));
  EXPECT_TRUE(
      std::string_view((const char*)mesh_buf, mesh_sz).find("v 1.0 2.0 3.0") !=
      std::string_view::npos);
  mju_closeResource(mesh_res);

  // Read member from nested archive (zip within an mjz)
  std::string nested_archive_xml =
      "nestedprovider://models/nested.mjz/inner.zip/inner.xml";
  mjResource* inner_xml_res = mju_openResource("", nested_archive_xml.c_str(),
                                               nullptr, err, sizeof(err));
  ASSERT_THAT(inner_xml_res, NotNull()) << err;
  const void* inner_xml_buf = nullptr;
  int inner_xml_sz = mju_readResource(inner_xml_res, &inner_xml_buf);
  EXPECT_THAT(inner_xml_sz, Gt(0));
  EXPECT_TRUE(std::string_view((const char*)inner_xml_buf, inner_xml_sz)
                  .find("b_inner") != std::string_view::npos);
  mju_closeResource(inner_xml_res);

  // Read deeply nested member from nested archive located in a subdirectory
  std::string nested_archive_mesh =
      "nestedprovider://models/nested.mjz/sub/inner.zip/sub/inner_mesh.obj";
  mjResource* inner_mesh_res = mju_openResource("", nested_archive_mesh.c_str(),
                                                nullptr, err, sizeof(err));
  ASSERT_THAT(inner_mesh_res, NotNull()) << err;
  const void* inner_mesh_buf = nullptr;
  int inner_mesh_sz = mju_readResource(inner_mesh_res, &inner_mesh_buf);
  EXPECT_THAT(inner_mesh_sz, Gt(0));
  EXPECT_TRUE(std::string_view((const char*)inner_mesh_buf, inner_mesh_sz)
                  .find("v 7.0 8.0 9.0") != std::string_view::npos);
  mju_closeResource(inner_mesh_res);

  // Verify nested archive extraction when outer archive is on disk
  std::string temp_mjz =
      (std::filesystem::path(testing::TempDir()) / "nested_disk.mjz").string();
  {
    std::ofstream ofs(temp_mjz, std::ios::binary);
    ofs.write(reinterpret_cast<const char*>(nested_data.bytes.data()),
              nested_data.bytes.size());
  }

  std::string disk_nested_xml = temp_mjz + "/inner.zip/inner.xml";
  mjResource* disk_xml_res =
      mju_openResource("", disk_nested_xml.c_str(), nullptr, err, sizeof(err));
  ASSERT_THAT(disk_xml_res, NotNull()) << err;
  const void* disk_xml_buf = nullptr;
  int disk_xml_sz = mju_readResource(disk_xml_res, &disk_xml_buf);
  EXPECT_THAT(disk_xml_sz, Gt(0));
  EXPECT_TRUE(std::string_view((const char*)disk_xml_buf, disk_xml_sz)
                  .find("b_inner") != std::string_view::npos);
  mju_closeResource(disk_xml_res);

  std::filesystem::remove(temp_mjz);
}

TEST_F(MjzArchiveProviderTest, ConcurrentMemberExtraction) {
  std::string filepath = GetTestDataFilePath("testdata/model.mjz");

  mjVFS vfs;
  mj_defaultVFS(&vfs);

  constexpr int kNumThreads = 8;
  std::vector<std::future<bool>> futures;

  for (int i = 0; i < kNumThreads; ++i) {
    futures.push_back(std::async(std::launch::async, [&filepath, &vfs]() {
      std::string member = filepath + "/model.xml";
      char err[1000] = "";
      mjResource* res =
          mju_openResource("", member.c_str(), &vfs, err, sizeof(err));
      if (!res) return false;
      const void* buf = nullptr;
      int sz = mju_readResource(res, &buf);
      mju_closeResource(res);
      return sz > 0 && buf != nullptr;
    }));
  }

  for (auto& f : futures) {
    EXPECT_TRUE(f.get());
  }

  mj_deleteVFS(&vfs);
}

}  // namespace
}  // namespace mujoco
