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

#ifndef MUJOCO_SRC_ENGINE_ENGINE_GLOBAL_TABLE_H_
#define MUJOCO_SRC_ENGINE_ENGINE_GLOBAL_TABLE_H_

#include <atomic>
#include <cctype>
#include <cstdio>
#include <mutex>
#include <new>
#include <string>
#include <string_view>
#include <type_traits>

#include "engine/engine_util_errmem.h"

namespace mujoco {
static constexpr int kCacheLineBytes = 256;

static inline bool CaseInsensitiveEqual(std::string_view s1, std::string_view s2) {
  if (s1.length() != s2.length()) {
    return false;
  }
  auto len = s1.length();
  for (decltype(len) i = 0; i < len; ++i) {
    if (std::tolower(s1[i]) != std::tolower(s2[i])) {
      return false;
    }
  }
  return true;
}

// A table intended for use as global storage for extension objects such as plugins, implemented as
// a linked list of array "blocks". This is a compromise that maintains a good degree of memory
// locality while not invalidating existing pointers when growing the table. It is expected that for
// most users, the number of objects loaded will be small enough to fit in the initial block, and
// so the global table will behave like an array. Since pointers are never invalidated, we do not
// need to apply a read lock on the global table when resolving an element.
template<typename T>
struct alignas(kCacheLineBytes) TableBlock {
  static constexpr int kBlockSize = 15;

  TableBlock() : objects{}, next(nullptr) {
    static_assert(
        sizeof(TableBlock<T>) / kCacheLineBytes ==
        sizeof(TableBlock<T>::objects) / kCacheLineBytes
        + (sizeof(TableBlock<T>::objects) % kCacheLineBytes > 0),
        "TableBlock::next doesn't fit in the same cache line as the end of TableBlock::objects");
  }

  T objects[kBlockSize];
  TableBlock<T>* next;
};


using Mutex = std::mutex;

class ReentrantWriteLock {
 public:
  ReentrantWriteLock(Mutex& mutex) : mutex_(mutex) {
    if (LockCountOnCurrentThread() == 0) {
      mutex_.lock();
    }
    ++LockCountOnCurrentThread();
  }

  ~ReentrantWriteLock() {
    if (--LockCountOnCurrentThread() == 0) {
      mutex_.unlock();
    }
  }

 private:
  Mutex& mutex_;

  static int& LockCountOnCurrentThread() noexcept {
    thread_local int counter = 0;
    return counter;
  }
};

template<typename T>
class GlobalTable {
 public:
  static GlobalTable<T>& GetSingleton() {
    static_assert(std::is_trivially_destructible_v<GlobalTable<T>>);
    static GlobalTable<T> global;
    return global;
  }

  // Each extension object type T must implement these.
  using ErrorMessage = char[512];
  static const char* HumanReadableTypeName();
  static std::string_view ObjectKey(const T&);
  static bool ObjectEqual(const T&, const T&);
  static bool CopyObject(T& dst, const T& src, ErrorMessage& err);

  int count() {
    return count_.load(std::memory_order_acquire);
  }

  ReentrantWriteLock LockExclusively() {
    return ReentrantWriteLock(mutex());
  }

  int AppendIfUnique(const T& obj) {
    ErrorMessage err = "\0";

    // ========= ATTENTION! ========================================================================
    // Do not handle objects with nontrivial destructors outside of this lambda.
    // Do not call mju_error inside this lambda.
    int slot = [&]() {
      auto lock = LockExclusively();

      int count = count_.load(std::memory_order_acquire);
      int local_idx = 0;
      TableBlock<T>* block = &first_block_;

      // check if a non-identical object has already been registered
      for (int i = 0; i < count; ++i, ++local_idx) {
        if (local_idx == TableBlock<T>::kBlockSize) {
          local_idx = 0;
          block = block->next;
        }
        const T& existing = block->objects[local_idx];
        if (CaseInsensitiveEqual(ObjectKey(obj), ObjectKey(existing))) {
          if (!ObjectEqual(obj, existing)) {
            std::snprintf(err, sizeof(err), "%s '%s' is already registered",
                          HumanReadableTypeName(), std::string(ObjectKey(obj)).c_str());
            return -1;
          } else {
            return i;
          }
        }
      }

      // allocate a new block if the last allocated block is full
      if (local_idx == TableBlock<T>::kBlockSize) {
        local_idx = 0;
        block->next = new(std::nothrow) TableBlock<T>;
        if (!block->next) {
          std::snprintf(err, sizeof(err), "failed to allocate memory for a new %s table block",
                        HumanReadableTypeName());
          return -1;
        }
        block = block->next;
      }

      // copy the new object into the table
      if (!CopyObject(block->objects[local_idx], obj, err)) {
        return -1;
      }

      // increment the global count with a release memory barrier
      count_.store(count + 1, std::memory_order_release);

      return count;
    }();
    // ========= ATTENTION! ========================================================================
    // End of safe lambda, do not handle objects with non-trivial destructors beyond this point.

    // registration failed, throw an mju_error
    if (slot < 0) {
      err[sizeof(err) - 1] = '\0';
      mju_error("%s", err);
    }

    return slot;
  }

  // look up by slot number, assuming that count() has already been called
  const T* GetAtSlotUnsafe(int slot, int nslot) {
    if (slot < 0 || slot >= nslot) {
      return nullptr;
    }

    TableBlock<T>* block = &first_block_;

    // iterate over blocks in the global table until the local index is less than the block size
    int local_idx = slot;
    while (local_idx >= TableBlock<T>::kBlockSize) {
      local_idx -= TableBlock<T>::kBlockSize;
      block = block->next;
      if (!block) {
        return nullptr;
      }
    }

    // local_idx is now a valid index into the current block
    T* obj = &(block->objects[local_idx]);

    // check if obj has been initialized
    if (obj && ObjectKey(*obj).empty()) {
      return nullptr;
    }

    return obj;
  }

  // look up by key, assuming that count() has already been called
  const T* GetByKeyUnsafe(std::string_view key, int* slot, int nslot) {
    if (slot) *slot = -1;

    if (key.empty()) {
      return nullptr;
    }

    TableBlock<T>* block = &first_block_;
    int found_slot = 0;
    while (block) {
      for (int i = 0;
          i < TableBlock<T>::kBlockSize && found_slot < nslot;
          ++i, ++found_slot) {
        const T& obj = block->objects[i];

        // reached an uninitialized object, which means that iterated beyond the object count
        // this should never happen if `count` was actually returned by count()
        std::string_view candidate_key = ObjectKey(obj);
        if (candidate_key.empty()) {
          return nullptr;
        }

        // check if key matches the query
        if (CaseInsensitiveEqual(candidate_key, key)) {
          if (slot) *slot = found_slot;
          return &obj;
        }
      }

      block = block->next;
    }

    return nullptr;
  }

  const T* GetAtSlot(int slot) {
    // count() uses memory_order_acquire which acts as a barrier that guarantees that all
    // objects up to `count` have been completely inserted
    return GetAtSlotUnsafe(slot, count());
  }

  const T* GetByKey(std::string_view key, int* slot) {
    // count() uses memory_order_acquire which acts as a barrier that guarantees that all
    // objects up to `count` have been completely inserted
    return GetByKeyUnsafe(key, slot, count());
  }

 private:
  GlobalTable() {
    new(mutex_) Mutex;
  }

  Mutex& mutex() {
    return *std::launder(reinterpret_cast<Mutex*>(&mutex_));
  }

  TableBlock<T> first_block_;
  std::atomic_int count_;

  // A mutex whose destructor is never run.
  // When a C++ program terminates, the destructors for function static objects and globals will be
  // executed by whichever thread started that termination but there is no guarantee that other
  // threads have terminated. In other words, a static object may be accessed by another thread
  // after it is deleted. We avoid destruction issues by never running the destructor.
  alignas(Mutex) unsigned char mutex_[sizeof(Mutex)];
};

}  // namespace mujoco

#endif  // MUJOCO_SRC_ENGINE_ENGINE_GLOBAL_TABLE_H_
