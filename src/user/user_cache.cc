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

#include "user/user_cache.h"

#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>

#include <mujoco/mjplugin.h>
#include "user/user_resource.h"


// makes a copy for user (strip unnecessary items)
mjCAsset mjCAsset::Copy(const mjCAsset& other) {
  mjCAsset asset;
  asset.id_ = other.Id();
  asset.timestamp_ = other.Timestamp();
  asset.data_ = other.data_;
  asset.size_ = other.size_;
  return asset;
}



// sets the total maximum size of the cache in bytes
// low-priority cached assets will be dropped to make the new memory
// requirement
void mjCCache::SetMaxSize(std::size_t size) {
  std::lock_guard<std::mutex> lock(mutex_);
  max_size_ = size;
  Trim();
}



// returns the corresponding timestamp, if the given asset is stored in the cache
const std::string* mjCCache::HasAsset(const std::string& id) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = lookup_.find(id);
  if (it == lookup_.end()) {
    return nullptr;
  }

  return &(it->second.Timestamp());
}



// inserts an asset into the cache, if asset is already in the cache, its data
// is updated only if the timestamps disagree
bool mjCCache::Insert(const std::string& modelname, const mjResource *resource,
                      std::shared_ptr<const void> data, std::size_t size) {
  std::lock_guard<std::mutex> lock(mutex_);

  // check if asset is too large to fit in the cache
  if ((size_ + size > max_size_) &&
      lookup_.find(resource->name) == lookup_.end()) {
    return false;
  }
  mjCAsset asset(modelname, resource, data, size);
  auto [it, inserted] = lookup_.insert({resource->name, asset});
  mjCAsset* asset_ptr = &(it->second);

  if (!inserted) {
    if (size_ - asset_ptr->BytesCount() + size > max_size_) {
      return false;
    }
    models_[modelname].insert(asset_ptr);  // add it for the model
    asset_ptr->AddReference(modelname);
    if (it->second.Timestamp() == asset.Timestamp()) {
      return true;
    }
    asset_ptr->SetTimestamp(asset.Timestamp());
    size_ = size_ - asset_ptr->BytesCount() + size;
    asset_ptr->ReplaceData(asset);
    return true;
  }

  // new asset
  asset_ptr->SetInsertNum(insert_num_++);
  entries_.insert(asset_ptr);
  models_[modelname].insert(asset_ptr);
  size_ += size;
  return true;
}



// populate data from the cache into the given function, return true if data was
// copied
bool mjCCache::PopulateData(const mjResource* resource, mjCDataFunc fn) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = lookup_.find(resource->name);
  if (it == lookup_.end()) {
    return false;
  }

  if (mju_isModifiedResource(resource, it->second.Timestamp().c_str())) {
    return false;
  }

  mjCAsset* asset = &(it->second);
  asset->IncrementAccess();

  // update priority queue
  entries_.erase(asset);
  entries_.insert(asset);

  return asset->PopulateData(fn);
}



// removes model from the cache along with assets referencing only this model
void mjCCache::RemoveModel(const std::string& filename) {
  std::lock_guard<std::mutex> lock(mutex_);
  for (mjCAsset* asset : models_[filename]) {
    asset->RemoveReference(filename);
    if (!asset->HasReferences()) {
      Delete(asset, filename);
    }
  }
  models_.erase(filename);
}



// Wipes out all internal data for the given model
void mjCCache::Reset(const std::string& filename) {
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto asset : models_[filename]) {
    Delete(asset, filename);
  }
  models_.erase(filename);
}



// Wipes out all internal data
void mjCCache::Reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  entries_.clear();
  lookup_.clear();
  models_.clear();
  size_ = 0;
  insert_num_ = 0;
}



std::size_t mjCCache::MaxSize() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return max_size_;
}



std::size_t mjCCache::Size() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return size_;
}



// Deletes a single asset
void mjCCache::DeleteAsset(const std::string& id) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = lookup_.find(id);
  if (it != lookup_.end()) {
    Delete(&(it->second));
  }
}



// Deletes a single asset (internal)
void mjCCache::Delete(mjCAsset* asset) {
  size_ -= asset->BytesCount();
  entries_.erase(asset);
  for (auto& reference : asset->References()) {
    models_[reference].erase(asset);
  }
  lookup_.erase(asset->Id());
}



// Deletes a single asset (internal)
void mjCCache::Delete(mjCAsset* asset, const std::string& skip) {
  size_ -= asset->BytesCount();
  entries_.erase(asset);

  for (auto& reference : asset->References()) {
    if (reference != skip) {
      models_[reference].erase(asset);
    }
  }
  lookup_.erase(asset->Id());
}



// trims out data to meet memory requirements
void mjCCache::Trim() {
  while (size_ > max_size_) {
    Delete(*entries_.begin());
  }
}
