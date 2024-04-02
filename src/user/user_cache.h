// Copyright 2024 DeepMind Technologies Limited
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

#ifndef MUJOCO_SRC_USER_CACHE_H_
#define MUJOCO_SRC_USER_CACHE_H_

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <set>
#include <string>
#include <memory>
#include <mutex>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

// data associated with an asset
struct mjCAssetData {
  std::shared_ptr<uint8_t> bytes;    // raw serialized bytes of cached data
  std::size_t nbytes;                // number of bytes stored
};

// A class container for a thread-safe asset cache
//
// Each mjCAsset is used to store raw and/or processed data loaded from a
// resource and is defined by a unique ID (usually the full filename of the
// asset). The asset's data can be segregated into blocks for ease of use of
// mix and matching different types of data. Each block is given a unique name
// within the asset for readability. For example, mjCAsset for a mesh may
// include all vertex positions, edges, and the computed volume.
class mjCAsset {
  friend class mjCCache;
 public:
  mjCAsset(std::string filename, std::string id, std::string timestamp)
      :  id_(std::move(id)), timestamp_(std::move(timestamp)) {
    AddReference(filename);
  }

  // move and copy constructors
  mjCAsset(mjCAsset&& other) = default;
  mjCAsset& operator=(mjCAsset&& other) = default;
  mjCAsset(const mjCAsset& other) = default;
  mjCAsset& operator=(const mjCAsset& other) = default;

  // copies a block of data into the asset and returns number of bytes stored
  // loading data into an asset should happen in a single thread
  template<typename T> std::size_t Add(const std::string& name,
                                       const T* data, std::size_t n);


  // copies a vector into the asset and returns number of bytes stored
  // loading data into an asset should happen in a single thread
  template<typename T> std::size_t AddVector(const std::string& name,
                                             const std::vector<T>& v);

  // returns a pointer to a block of data, sets n to size of data
  template<typename T>
  const T* Get(const std::string& name, std::size_t* n) const;

  // copies a block of data into a vector
  template<typename T>
  std::optional<std::vector<T>> GetVector(const std::string& name) const;

  // returns true if a block of data by the given name is stored in the asset
  bool HasData(const std::string& name) const {
    return blocks_.find(name) != blocks_.end();
  }

  const std::string& Timestamp() const { return timestamp_; }
  const std::string& Id() const { return id_; }
  std::size_t InsertNum() const { return insert_num_; }
  std::size_t AccessCount() const { return access_count_; }

 private:
  mjCAsset() = default;

  // helpers for managing models referencing this asset
  void AddReference(std::string xml_file) { references_.insert(xml_file); }
  void RemoveReference(const std::string& xml_file) {
    references_.erase(xml_file);
  }
  bool HasReferences() const { return !references_.empty(); }

  // replaces data blocks in asset
  void ReplaceBlocks(
      const std::unordered_map<std::string, mjCAssetData>& blocks,
      std::size_t nbytes);

  void IncrementAccess() { access_count_++; }

  // makes a copy for user (strip unnecessary references)
  static mjCAsset Copy(const mjCAsset& other);

  // setters
  void SetInsertNum(std::size_t num) { insert_num_ = num; }
  void SetTimestamp(std::string timestamp) { timestamp_ = timestamp; }

  // accessors
  std::size_t BytesCount() const { return nbytes_; }
  const std::unordered_map<std::string, mjCAssetData>& Blocks() const {
    return blocks_;
  }
  const std::set<std::string>& References() const { return references_; }

  std::string id_;                    // unique id associated with asset
  std::string timestamp_;             // opaque timestamp of asset
  std::size_t insert_num_;            // number when asset was inserted
  std::size_t access_count_ = 0;      // incremented when getting 0th block
  std::size_t nbytes_ = 0;            // how many bytes taken up by the asset

  // the actually data of the asset
  std::unordered_map<std::string, mjCAssetData> blocks_;

  // list of models referencing this asset
  std::set<std::string> references_;
};

struct mjCAssetCompare {
  bool operator()(const mjCAsset* e1, const mjCAsset* e2) const {
    if (e1->AccessCount() != e2->AccessCount()) {
      return e1->AccessCount() < e2->AccessCount();
    }
    return e1->InsertNum() < e2->InsertNum();
  }
};

// the class container for a thread-safe asset cache
class mjCCache {
 public:
  explicit mjCCache(std::size_t size)  :  max_size_(size) {}

  // move only
  mjCCache(mjCCache&& other) = delete;
  mjCCache& operator=(mjCCache&& other) = delete;
  mjCCache(const mjCCache& other) = delete;
  mjCCache& operator=(const mjCCache& other) = delete;

  // sets the total maximum size of the cache in bytes
  // low-priority cached assets will be dropped to make the new memory
  // requirement
  void SetMaxSize(std::size_t size);

  // returns the corresponding timestamp, if the given asset is stored in
  // the cache
  const std::string* HasAsset(const std::string& id);

  // inserts an asset into the cache, if asset is already in the cache, its data
  // is updated only if the timestamps disagree
  bool Insert(const mjCAsset& asset);
  bool Insert(mjCAsset&& asset);

  // returns the asset with the given id, if it exists in the cache
  std::optional<mjCAsset> Get(const std::string& id);

  // deletes the asset from the cache with the given id
  void DeleteAsset(const std::string& id);

  // removes model from the cache, assets only referenced by the model will be
  // deleted
  void RemoveModel(const std::string& filename);

  // Wipes out all assets from the cache for the given model
  void Reset(const std::string& filename);

  // Wipes out all internal data
  void Reset();

  // accessors
  std::size_t MaxSize() const;
  std::size_t Size() const;

 private:
  void Delete(mjCAsset* asset);
  void Delete(mjCAsset* asset, const std::string& skip);
  void Trim();

  // TODO(kylebayes): We should consider a shared mutex like in
  // engine/engine_plugin.cc as some of these methods don't need to be fully
  // locked.
  mutable std::mutex mutex_;
  std::size_t insert_num_ = 0;  // a running counter of assets being inserted
  std::size_t size_ = 0;        // current size of the cache in bytes
  std::size_t max_size_ = 0;    // max size of the cache in bytes

  // internal constant look up table for assets
  std::unordered_map<std::string, mjCAsset> lookup_;

  // internal priority queue for the cache
  std::set<mjCAsset*, mjCAssetCompare> entries_;

  // models using the cache along with the assets they reference
  std::unordered_map<std::string, std::unordered_set<mjCAsset*>> models_;
};

#endif  // MUJOCO_SRC_USER_CACHE_H_
