// Copyright 2022 DeepMind Technologies Limited
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

// Plugin registration is implemented in C++, unlike the rest of the engine code which is in C.
// This is because C++ provides a standard cross-platform mutex, which we use to guard the global
// plugin table in order to make the API thread-safe. We only expose a C API externally, and this
// entire file can in principle be re-implemented in C if necessary, without breaking any external
// or internal MuJoCo code elsewhere.

#include "engine/engine_plugin.h"

#include <atomic>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <new>
#include <shared_mutex>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

extern "C" {
#if defined(_WIN32) || defined(__CYGWIN__)
  #include <windows.h>
#else
  #include <dirent.h>
  #include <dlfcn.h>
#endif
}

#ifdef __APPLE__
#include <Availability.h>
#if !defined(MAC_OS_X_VERSION_MIN_REQUIRED) && defined(__MAC_OS_X_VERSION_MIN_REQUIRED)
#define MAC_OS_X_VERSION_MIN_REQUIRED __MAC_OS_X_VERSION_MIN_REQUIRED
#endif
#endif

#include <mujoco/mjplugin.h>
#include "engine/engine_util_errmem.h"

// set default plugin definition
void mjp_defaultPlugin(mjpPlugin* plugin) {
  std::memset(plugin, 0, sizeof(*plugin));
}

namespace {
constexpr int kMaxNameLength = 1024;
constexpr int kMaxAttributes = 255;

constexpr int kCacheLine = 64;

// vfs prefix
constexpr const char* kVfsPrefix = mjVFS_PREFIX;

// A table of registered plugins, implemented as a linked list of array "blocks".
// This is a compromise that maintains a good degree of memory locality while not invalidating
// existing pointers when growing the table. It is expected that for most users, the number of
// plugins loaded into a program will be small enough to fit in the initial block, and so the global
// table will behave like an array. Since pointers are never invalidated, we do not need to apply a
// read lock on the global table when resolving a plugin.
template<typename T>
struct alignas(kCacheLine) PluginTable {
  static constexpr int kBlockSize = 15;

  PluginTable() {
    for (int i = 0; i < kBlockSize; ++i) {
      std::memset(&plugins[i], 0, sizeof(plugins[i]));
    }
  }

  T plugins[kBlockSize];
  PluginTable<T>* next = nullptr;
};

static_assert(
    sizeof(PluginTable<mjpPlugin>) / kCacheLine ==
    sizeof(PluginTable<mjpPlugin>::plugins) / kCacheLine
    + (sizeof(PluginTable<mjpPlugin>::plugins) % kCacheLine > 0),
    "PluginTable::next doesn't fit in the same cache line as the end of PluginTable::plugins");

using Mutex = std::shared_mutex;

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
class Global {
 public:
  Global() {
    new(mutex_) Mutex;
  }
  PluginTable<T>& table() {
    return table_;
  }
  std::atomic_int& count() {
    return count_;
  }
  Mutex& mutex() {
    return *std::launder(reinterpret_cast<Mutex*>(&mutex_));
  }

  ReentrantWriteLock lock_mutex_exclusively() {
    return ReentrantWriteLock(mutex());
  }

 private:
  PluginTable<T> table_;
  std::atomic_int count_;

  // A mutex whose destructor is never run.
  // When a C++ program terminates, the destructors for function static objects and globals will be
  // executed by whichever thread started that termination but there is no guarantee that other
  // threads have terminated. In other words, a static object may be accessed by another thread
  // after it is deleted. We avoid destruction issues by never running the destructor.
  alignas(Mutex) unsigned char mutex_[sizeof(Mutex)];
};

template<typename T>
Global<T>& GetGlobal() {
  static Global<T> global;
  static_assert(std::is_trivially_destructible_v<decltype(global)>);
  return global;
}

// allocate new block for the global table
template<typename T>
PluginTable<T>* AddNewTableBlock(PluginTable<T>* table) {
  char err[512];
  err[0] = '\0';

#if defined(MAC_OS_X_VERSION_MIN_REQUIRED) && MAC_OS_X_VERSION_MIN_REQUIRED < MAC_OS_X_VERSION_10_14
  // aligned nothrow new is not available until macOS 10.14
  posix_memalign(reinterpret_cast<void**>(&table->next),
                 alignof(PluginTable<T>), sizeof(PluginTable<T>));
  if (table->next) new(table->next) PluginTable<T>;
#else
  table->next = new(std::nothrow) PluginTable<T>;
#endif
  if (!table->next) {
    std::snprintf(err, sizeof(err), "failed to allocate memory for the global plugin table");
    return nullptr;
  }
  return table->next;
}

// look up a plugin by slot number
template<typename T>
const T* GetAtSlot(int slot, int nslot) {
  if (slot < 0 || slot >= nslot) {
    return nullptr;
  }

  Global<T>& global = GetGlobal<T>();
  PluginTable<T>* table = &global.table();

  // iterate over blocks in the global table until the local index is less than the block size
  int local_idx = slot;
  while (local_idx >= PluginTable<T>::kBlockSize) {
    local_idx -= PluginTable<T>::kBlockSize;
    table = table->next;
    if (!table) {
      return nullptr;
    }
  }

  // local_idx is now a valid index into the current block
  return &(table->plugins[local_idx]);
}

// return the length of a null-terminated string, or -1 if it is not terminated after kMaxNameLength
int strklen(const char* s) {
  for (int i = 0; i < kMaxNameLength; ++i) {
    if (!s[i]) {
      return i;
    }
  }
  return -1;
}

// copy a null-terminated string into a new heap-allocated char array managed by a unique_ptr
std::unique_ptr<char[]> CopyName(const char* s) {
  int len = strklen(s);
  if (len == -1) {
    return nullptr;
  }
  std::unique_ptr<char[]> out(new(std::nothrow) char[len + 1]);
  if (!out) {
    return nullptr;
  }
  std::strncpy(out.get(), s, len);
  out.get()[len] = '\0';
  return out;
}

// check if two plugins are identical
bool PluginsAreIdentical(const mjpPlugin& plugin1, const mjpPlugin& plugin2) {
  if (plugin1.name && !plugin2.name) {
    return false;
  }
  if (plugin2.name && !plugin1.name) {
    return false;
  }
  if (plugin1.name && plugin2.name &&
      std::strncmp(plugin1.name, plugin2.name, kMaxNameLength)) {
    return false;
  }

  if (plugin1.nattribute != plugin2.nattribute) {
    return false;
  }
  for (int i = 0; i < plugin1.nattribute; ++i) {
    if (plugin1.attributes[i] && !plugin2.attributes[i]) {
      return false;
    }
    if (plugin2.attributes[i] && !plugin1.attributes[i]) {
      return false;
    }
    if (plugin1.attributes[i] && plugin2.attributes[i] &&
        std::strncmp(plugin1.attributes[i], plugin2.attributes[i],
                     kMaxNameLength)) {
      return false;
    }
  }

  const char* ptr1 = reinterpret_cast<const char*>(&plugin1.attributes) +
                     sizeof(plugin1.attributes);
  const char* ptr2 = reinterpret_cast<const char*>(&plugin2.attributes) +
                     sizeof(plugin2.attributes);
  std::size_t remaining_size =
      sizeof(mjpPlugin) - (ptr1 - reinterpret_cast<const char*>(&plugin1));
  return !std::memcmp(ptr1, ptr2, remaining_size);
}

// check if two resource providers are identical
bool ResourceProvidersAreIdentical(const mjpResourceProvider* p1, const mjpResourceProvider* p2) {
  return (!std::strcmp(p1->prefix, p2->prefix) &&
          p1->open == p2->open &&
          p1->read == p2->read &&
          p1->close == p2->close &&
          p1->data == p2->data);
}
}  // namespace

// globally register a plugin (thread-safe), return new slot id
int mjp_registerPlugin(const mjpPlugin* plugin) {
  if (!plugin->name) {
    mju_error("plugin->name is a null pointer");
  } else if (plugin->name[0] == '\0') {
    mju_error("plugin->name is an empty string");
  } else if (plugin->nattribute < 0) {
    mju_error("plugin->nattribute is negative");
  } else if (plugin->nattribute > kMaxAttributes) {
    mju_error("plugin->nattribute exceeds the maximum limit of %i",
              kMaxAttributes);
  }

  char err[512];
  err[0] = '\0';

  // ========= ATTENTION! ==========================================================================
  // Do not handle objects with nontrivial destructors outside of this lambda.
  // Do not call mju_error inside this lambda.
  int slot = [&]() -> int {
    // check and copy the plugin name
    std::unique_ptr<char[]> name = CopyName(plugin->name);
    if (!name) {
      if (strklen(plugin->name) == -1) {
        std::snprintf(err, sizeof(err),
                      "plugin->name length exceeds the maximum limit of %d", kMaxNameLength);
      } else {
        std::snprintf(err, sizeof(err), "failed to allocate memory for plugin name");
      }
      return -1;
    }

    // check and copy plugin attributes
    std::vector<std::unique_ptr<char[]>> attributes_vec;
    if (plugin->nattribute) {
      attributes_vec.reserve(plugin->nattribute);
      for (int i = 0; i < plugin->nattribute; ++i) {
        std::unique_ptr<char[]> attr = CopyName(plugin->attributes[i]);
        if (!attr) {
          if (strklen(plugin->attributes[i]) == -1) {
            std::snprintf(
                err, sizeof(err),
                "plugin->attributes[%d] exceeds the maximum limit of %d", i, kMaxAttributes);
          } else {
            std::snprintf(err, sizeof(err), "failed to allocate memory for plugin attribute");
          }
          return -1;
        }
        attributes_vec.emplace_back(std::move(attr));
      }
    }

    Global<mjpPlugin>& global = GetGlobal<mjpPlugin>();
    auto lock = global.lock_mutex_exclusively();

    int count = global.count().load(std::memory_order_acquire);
    int local_idx = 0;
    PluginTable<mjpPlugin>* table = &global.table();

    // check if a non-identical plugin with the same name has already been registered
    for (int i = 0; i < count; ++i, ++local_idx) {
      if (local_idx == PluginTable<mjpPlugin>::kBlockSize) {
        local_idx = 0;
        table = table->next;
      }
      mjpPlugin& existing = table->plugins[local_idx];
      if (std::strcmp(plugin->name, existing.name) == 0) {
        if (PluginsAreIdentical(*plugin, existing)) {
          return i;
        } else {
          std::snprintf(err, sizeof(err), "plugin '%s' is already registered", plugin->name);
          return -1;
        }
      }
    }

    // allocate a new block of PluginTable if the last allocated block is full
    if (local_idx == PluginTable<mjpPlugin>::kBlockSize) {
      local_idx = 0;
      table = AddNewTableBlock<mjpPlugin>(table);
      if (!table) {
        return -1;
      }
    }

    // release the attribute names from unique_ptr into a plain array
    const char** attributes = nullptr;
    if (plugin->nattribute) {
      attributes = new(std::nothrow) const char*[plugin->nattribute];
      if (!attributes) {
        std::snprintf(err, sizeof(err), "failed to allocate memory for plugin attribute array");
        return -1;
      }
      for (int i = 0; i < plugin->nattribute; ++i) {
        attributes[i] = attributes_vec[i].release();
      }
    }

    // all checked passed, actually register the plugin into the global table
    mjpPlugin& registered_plugin = table->plugins[local_idx];
    registered_plugin = *plugin;
    registered_plugin.name = name.release();
    registered_plugin.attributes = attributes;

    // increment the global plugin count with a release memory barrier
    global.count().store(count + 1, std::memory_order_release);

    return count;
  }();

  // ========= ATTENTION! ==========================================================================
  // End of safe lambda, do not handle objects with non-trivial destructors beyond this point.

  // plugin registration failed, throw an mju_error
  if (slot < 0) {
    err[sizeof(err) - 1] = '\0';
    mju_error("%s", err);
  }

  return slot;
}

// look up plugin by slot number, assuming that mjp_pluginCount has already been called
const mjpPlugin* mjp_getPluginAtSlotUnsafe(int slot, int nslot) {
  const mjpPlugin* plugin = GetAtSlot<mjpPlugin>(slot, nslot);
  if (!plugin || !plugin->name) {
    return nullptr;
  }
  return plugin;
}

// look up plugin by name, assuming that mjp_pluginCount has already been called
const mjpPlugin* mjp_getPluginUnsafe(const char* name, int* slot, int nslot) {
  if (slot) *slot = -1;

  if (!name || !name[0]) {
    return nullptr;
  }

  Global<mjpPlugin>& plugins = GetGlobal<mjpPlugin>();
  PluginTable<mjpPlugin>* table = &plugins.table();
  int found_slot = 0;
  while (table) {
    for (int i = 0;
         i < PluginTable<mjpPlugin>::kBlockSize && found_slot < nslot;
         ++i, ++found_slot) {
      const mjpPlugin& plugin = table->plugins[i];

      // reached an uninitialized plugin, which means that iterated beyond the plugin count
      // this should never happen if `count` was actually returned by mjp_pluginCount
      if (!plugin.name) {
        return nullptr;
      }
      if (std::strcmp(plugin.name, name) == 0) {
        if (slot) *slot = found_slot;
        return &plugin;
      }
    }
    table = table->next;
  }

  return nullptr;
}

// return the number of globally registered plugins
int mjp_pluginCount() {
  return GetGlobal<mjpPlugin>().count().load(std::memory_order_acquire);
}

// look up a plugin by slot number
const mjpPlugin* mjp_getPluginAtSlot(int slot) {
  const int count = mjp_pluginCount();

  // mjp_pluginCount uses memory_order_acquire which acts as a barrier that guarantees that all
  // plugins up to `count` have been completely inserted
  return mjp_getPluginAtSlotUnsafe(slot, count);
}

// look up a plugin by name, optionally also get its registered slot number
const mjpPlugin* mjp_getPlugin(const char* name, int* slot) {
  const int count = mjp_pluginCount();
  int found_slot = -1;
  const mjpPlugin* plugin = mjp_getPluginUnsafe(name, &found_slot, count);

  if (slot) *slot = found_slot;
  return plugin;
}

namespace {
// seek the nth config attrib of a plugin instance by counting null terminators
const char* PluginAttrSeek(const mjModel* m, int plugin_id, int attrib_id) {
  const char* ptr = m->plugin_attr + m->plugin_attradr[plugin_id];
  for (int i = 0; i < attrib_id; ++i) {
    while (*ptr) {
      ++ptr;
    }
    ++ptr;
  }
  return ptr;
}
}  // namespace


// return a config attribute of a plugin instance
// NULL: invalid plugin instance ID or attribute name
const char* mj_getPluginConfig(const mjModel* m, int plugin_id, const char* attrib) {
  if (plugin_id < 0 || plugin_id >= m->nplugin || attrib == nullptr) {
    return nullptr;
  }

  const mjpPlugin* plugin = mjp_getPluginAtSlot(m->plugin[plugin_id]);
  if (!plugin) {
    return nullptr;
  }

  for (int i = 0; i < plugin->nattribute; ++i) {
    if (std::strcmp(plugin->attributes[i], attrib) == 0) {
      return PluginAttrSeek(m, plugin_id, i);
    }
  }

  return nullptr;
}

// set default resource provider definition
void mjp_defaultResourceProvider(mjpResourceProvider* provider) {
  std::memset(provider, 0, sizeof(*provider));
}

// globally register a resource provider (thread-safe), return new slot id
int mjp_registerResourceProvider(const mjpResourceProvider* provider) {
  // check against reserved prefixes
  int n = std::strlen(provider->prefix),
      m = std::strlen(kVfsPrefix);

  // one of the prefixes is a subprefix of the other
  if (!std::strncmp(kVfsPrefix, provider->prefix, n) ||
      !std::strncmp(kVfsPrefix, provider->prefix, m)) {
    mju_warning("provider->prefix is '%s' which is reserved", provider->prefix);
    return -1;
  }

  return mjp_registerResourceProviderInternal(provider);
}

// internal version of mjp_registerResourceProvider without prechecks on reserved prefixes
int mjp_registerResourceProviderInternal(const mjpResourceProvider* provider) {
  if (!provider->prefix || provider->prefix[0] == '\0') {
    mju_warning("provider->prefix is an empty string");
    return -1;
  }

  if (!provider->open || !provider->read || !provider->close) {
    mju_warning("provider must have the open, read, and close callbacks defined");
    return -1;
  }

  char err[512];
  err[0] = '\0';

  // ========= ATTENTION! ==========================================================================
  // Do not handle objects with nontrivial destructors outside of this lambda.
  // Do not call mju_error inside this lambda.
  int slot = [&]() -> int {
    bool vfs_provider = false;
    std::unique_ptr<char[]> prefix;

    // check if this is a VFS provider
    if (!std::strcmp(mjVFS_PREFIX, provider->prefix)) {
      vfs_provider = true;
    }

    // copy prefix
    if (!vfs_provider) {
      prefix = CopyName(provider->prefix);
      if (!prefix) {
        if (strklen(provider->prefix) == -1) {
          std::snprintf(err, sizeof(err),
                        "provider->prefix length exceeds the maximum limit of %d", kMaxNameLength);
        } else {
          std::snprintf(err, sizeof(err), "failed to allocate memory for resource provider prefix");
        }
        return -1;
      }
    }

    Global<mjpResourceProvider>& global = GetGlobal<mjpResourceProvider>();
    auto lock = global.lock_mutex_exclusively();
    int count = global.count().load(std::memory_order_acquire);
    int local_idx = 0;
    int free_idx = -1, free_local_idx = -1;
    PluginTable<mjpResourceProvider>* table = &global.table();
    PluginTable<mjpResourceProvider>* free_table = nullptr;

    // check if a non-identical provider with the same name has already been registered
    for (int i = 0; i < count; ++i, ++local_idx) {
      if (local_idx == PluginTable<mjpResourceProvider>::kBlockSize) {
        local_idx = 0;
        table = table->next;
      }
      mjpResourceProvider& existing = table->plugins[local_idx];

      // VFS providers can safely go in open slots
      if (vfs_provider && existing.prefix == nullptr && free_idx == -1) {
        free_table = table;
        free_idx = i;
        free_local_idx = local_idx;

        // can skip the rest
        break;
      }

      if (!vfs_provider && existing.prefix != nullptr) {
        int n = std::strlen(provider->prefix);
        int m = std::strlen(existing.prefix);

        // one of the prefixes is a subprefix of the other
        if (!std::strncmp(existing.prefix, provider->prefix, n) ||
            !std::strncmp(existing.prefix, provider->prefix, m)) {
          // if identical then return slot number
          if (ResourceProvidersAreIdentical(provider, &existing)) {
            return i;
          } else {
            std::snprintf(err, sizeof(err),
                          "a resource provider with prefix '%s' cannot be registered",
                          provider->prefix);
            return -1;
          }
        }
      }
    }

    // allocate a new block of PluginTable if the last allocated block is full
    if (free_local_idx == -1 && local_idx == PluginTable<mjpResourceProvider>::kBlockSize) {
      local_idx = 0;
      table = AddNewTableBlock<mjpResourceProvider>(table);
      if (!table) {
        return -1;
      }
    }

    // all checks passed, register the plugin into the global table
    mjpResourceProvider& registered_provider = table->plugins[local_idx];
    if (free_local_idx != -1) {
      registered_provider = free_table->plugins[free_local_idx];
    }

    registered_provider = *provider;
    registered_provider.prefix = (!vfs_provider) ? prefix.release() : kVfsPrefix;

    // increment the global plugin count with a release memory barrier
    if (free_idx == -1) {
      free_idx = count;
      global.count().store(count + 1, std::memory_order_release);
    }
    return free_idx;
  }();

  // ========= ATTENTION! ==========================================================================
  // End of safe lambda, do not handle objects with non-trivial destructors beyond this point.

  // plugin registration failed, throw a warning
  if (slot < 0) {
    err[sizeof(err) - 1] = '\0';
    mju_warning("%s", err);
  }

  return slot+1;
}

// globally unregister resource provider (thread-safe)
// only used for VFS resource providers
void mjp_unregisterResourceProvider(int slot) {
  // shift slot to zero-index
  slot--;

  if (slot < 0) {
    return;
  }

  // get global table, acquire lock
  Global<mjpResourceProvider>& global = GetGlobal<mjpResourceProvider>();
  auto lock = global.lock_mutex_exclusively();
  int count = global.count().load(std::memory_order_acquire);

  if (slot >= count) {
    return;
  }

  PluginTable<mjpResourceProvider>* table = &global.table();

  // iterate over blocks in the global table until the local index is less than the block size
  int local_idx = slot;
  while (local_idx >= PluginTable<mjpResourceProvider>::kBlockSize) {
    local_idx -= PluginTable<mjpResourceProvider>::kBlockSize;
    table = table->next;
    if (!table) {
      return;
    }
  }

  // local_idx is now a valid index into the current block
  mjpResourceProvider& provider = table->plugins[local_idx];

  // no-op for anything other than VFS resource providers
  if (provider.prefix == kVfsPrefix) {
    provider.prefix = nullptr;
  }
}

// return the number of globally registered resource providers
int mjp_resourceProviderCount() {
  return GetGlobal<mjpResourceProvider>().count().load(std::memory_order_acquire);
}

// look up a resource provider that matches its prefix against the given resource name
const mjpResourceProvider* mjp_getResourceProvider(const char* resource_name) {
  const int count = mjp_resourceProviderCount();
  if (!resource_name || !resource_name[0]) {
    return nullptr;
  }

  // since multiple VFS resource providers can be registered with the same
  // prefix, it doesn't make sense to try to match against them
  if (!std::strncmp(kVfsPrefix, resource_name, std::strlen(kVfsPrefix))) {
    return nullptr;
  }

  Global<mjpResourceProvider>& global = GetGlobal<mjpResourceProvider>();
  PluginTable<mjpResourceProvider>* table = &global.table();
  int found_slot = 0;

  while (table) {
    for (int i = 0;
         i < PluginTable<mjpPlugin>::kBlockSize && found_slot < count;
         ++i, ++found_slot) {

      const mjpResourceProvider& provider = table->plugins[i];
      const char *prefix = provider.prefix;

      if (prefix != nullptr &&
          !std::strncmp(prefix, resource_name, std::strlen(prefix))) {
        return &provider;
      }
    }
    table = table->next;
  }

  return nullptr;
}

// look up a resource provider by slot number
const mjpResourceProvider* mjp_getResourceProviderAtSlot(int slot) {
  // mjp_resourceProviderCount uses memory_order_acquire which acts as a barrier
  // that guarantees that all providers up to `count` have been completely inserted
  const int count = mjp_resourceProviderCount();

  // shift slot to be zero-indexed
  const mjpResourceProvider* provider = GetAtSlot<mjpResourceProvider>(slot - 1, count);
  if (!provider || provider->prefix[0] == '\0') {
    return nullptr;
  }
  return provider;
}

// load plugins from a dynamic library
void mj_loadPluginLibrary(const char* path) {
#if defined(_WIN32) || defined(__CYGWIN__)
  LoadLibraryA(path);
#else
  dlopen(path, RTLD_NOW | RTLD_LOCAL);
#endif
}

// scan a directory and load all dynamic libraries
void mj_loadAllPluginLibraries(const char* directory,
                               mjfPluginLibraryLoadCallback callback) {
  auto load_dso_and_call_callback = [&](const std::string& filename,
                                        const std::string& dso_path) {
    int nplugin_before;
    int nplugin_after;

    Global<mjpPlugin>& global = GetGlobal<mjpPlugin>();
    {
      auto lock = global.lock_mutex_exclusively();
      nplugin_before = mjp_pluginCount();
      mj_loadPluginLibrary(dso_path.c_str());
      nplugin_after = mjp_pluginCount();
    }

    if (callback) {
      int count = nplugin_after - nplugin_before;
      int first = count ? nplugin_before : -1;
      callback(filename.c_str(), first, count);
    }
  };

  // define platform-specific strings
#if defined(_WIN32) || defined(__CYGWIN__)
  const std::string sep = "\\";
  WIN32_FIND_DATAA find_data;
  HANDLE hfile = FindFirstFileA(
      (directory + sep + "*.dll").c_str(), &find_data);
  if (!hfile) {
    return;
  }

  // go through each file in the directory
  bool keep_going = true;
  while (keep_going) {
    const std::string name(find_data.cFileName);
    // load the library and check for plugins
    const std::string dso_path = directory + sep + name;
    load_dso_and_call_callback(name.c_str(), dso_path.c_str());
    keep_going = FindNextFileA(hfile, &find_data);
  }
  FindClose(hfile);
#else
  const std::string sep = "/";
  #if defined(__APPLE__)
    const std::string dso_suffix = ".dylib";
  #else
    const std::string dso_suffix = ".so";
  #endif

  DIR* dirp = opendir(directory);
  if (!dirp) {
    return;
  }

  // go through each entry in the directory
  for (struct dirent* dp; (dp = readdir(dirp));) {
    // only look at regular files (skip symlinks, pipes, directories, etc.)
    if (dp->d_type == DT_REG) {
      const std::string name(dp->d_name);
      if (name.size() > dso_suffix.size() &&
          name.substr(name.size() - dso_suffix.size()) == dso_suffix) {
        // load the library
        const std::string dso_path = directory + sep + name;
        load_dso_and_call_callback(name.c_str(), dso_path.c_str());
      }
    }
  }
  closedir(dirp);
#endif
}
