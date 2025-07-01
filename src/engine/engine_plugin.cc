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

#include <cctype>
#include <cstdio>
#include <cstring>
#include <memory>
#include <new>
#include <string>
#include <string_view>

extern "C" {
#if defined(_WIN32) || defined(__CYGWIN__)
  #include <windows.h>
#else
  #include <dirent.h>
  #include <dlfcn.h>
#endif
}

#include <mujoco/mjplugin.h>
#include "engine/engine_global_table.h"
#include "engine/engine_util_errmem.h"

// set default plugin definition
void mjp_defaultPlugin(mjpPlugin* plugin) {
  std::memset(plugin, 0, sizeof(*plugin));
}

namespace {
using mujoco::GlobalTable;

constexpr int kMaxNameLength = 1024;
constexpr int kMaxAttributes = 255;

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

// check if prefix is a valid URI scheme format
bool IsValidURISchemeFormat(const char* prefix) {
  int len;

  // prefix is NULL or empty
  if (prefix == nullptr || !(len = std::strlen(prefix))) {
    return false;
  }

  // first character must be a letter
  if (!std::isalpha(prefix[0])) {
    return false;
  }

  for (int i = 1; i < len; i++) {
    // each following character must be a letter, digit, '+', '.', or '-'
    if (!std::isalnum(prefix[i]) &&
        (prefix[i] != '+') &&
        (prefix[i] != '.') &&
        (prefix[i] != '-')) {
      return false;
    }
  }
  return true;
}

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

template <>
const char* GlobalTable<mjpPlugin>::HumanReadableTypeName() {
  return "plugin";
}

template <>
std::string_view GlobalTable<mjpPlugin>::ObjectKey(const mjpPlugin& plugin) {
  return std::string_view(plugin.name, strklen(plugin.name));
}

// check if two plugins are identical
template <>
bool GlobalTable<mjpPlugin>::ObjectEqual(const mjpPlugin& plugin1, const mjpPlugin& plugin2) {
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
    if (plugin1.attributes[i] && !plugin2.attributes[i]) {
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

template <>
bool GlobalTable<mjpPlugin>::CopyObject(mjpPlugin& dst, const mjpPlugin& src, ErrorMessage& err) {
  // check and copy the plugin name
  std::unique_ptr<char[]> name = CopyName(src.name);
  if (!name) {
    if (strklen(src.name) == -1) {
      std::snprintf(err, sizeof(err),
                    "plugin->name length exceeds the maximum limit of %d", kMaxNameLength);
    } else {
      std::snprintf(err, sizeof(err), "failed to allocate memory for plugin name");
    }
    return false;
  }

  // check and copy plugin attributes
  std::unique_ptr<std::unique_ptr<char[]>[]> attributes_list;
  if (src.nattribute) {
    attributes_list.reset(new(std::nothrow) std::unique_ptr<char[]>[src.nattribute]);
    if (!attributes_list) {
      std::snprintf(err, sizeof(err), "failed to allocate memory for plugin attribute list");
      return false;
    }
    for (int i = 0; i < src.nattribute; ++i) {
      std::unique_ptr<char[]> attr = CopyName(src.attributes[i]);
      if (!attr) {
        if (strklen(src.attributes[i]) == -1) {
          std::snprintf(
              err, sizeof(err),
              "length of plugin attribute %d exceeds the maximum limit of %d", i, kMaxAttributes);
        } else {
          std::snprintf(err, sizeof(err), "failed to allocate memory for plugin attribute %d", i);
        }
        return false;
      }
      attributes_list[i].swap(attr);
    }
  }

  // release the attribute names from unique_ptr into a plain array
  const char** attributes = nullptr;
  if (src.nattribute) {
    attributes = new(std::nothrow) const char*[src.nattribute];
    if (!attributes) {
      std::snprintf(err, sizeof(err), "failed to allocate memory for plugin attribute array");
      return -1;
    }
    for (int i = 0; i < src.nattribute; ++i) {
      attributes[i] = attributes_list[i].release();
    }
  }

  dst = src;
  dst.name = name.release();
  dst.attributes = attributes;

  return true;
}

template <>
const char* GlobalTable<mjpResourceProvider>::HumanReadableTypeName() {
  return "resource provider";
}

template <>
std::string_view GlobalTable<mjpResourceProvider>::ObjectKey(const mjpResourceProvider& plugin) {
  return std::string_view(plugin.prefix, strklen(plugin.prefix));
}

// check if two resource providers are identical
template <>
bool GlobalTable<mjpResourceProvider>::ObjectEqual(const mjpResourceProvider& p1, const mjpResourceProvider& p2) {
  return (CaseInsensitiveEqual(p1.prefix, p2.prefix) &&
          p1.open == p2.open &&
          p1.read == p2.read &&
          p1.close == p2.close &&
          p1.getdir == p2.getdir &&
          p1.modified == p2.modified &&
          p1.data == p2.data);
}

template <>
bool GlobalTable<mjpResourceProvider>::CopyObject(mjpResourceProvider& dst, const mjpResourceProvider& src, ErrorMessage& err) {
  // copy prefix
  std::unique_ptr<char[]> prefix = CopyName(src.prefix);
  if (!prefix) {
    if (strklen(src.prefix) == -1) {
      std::snprintf(err, sizeof(err),
                    "provider->prefix length exceeds the maximum limit of %d", kMaxNameLength);
    } else {
      std::snprintf(err, sizeof(err), "failed to allocate memory for resource provider prefix");
    }
    return false;
  }

  dst = src;
  dst.prefix = prefix.release();
  return true;
}

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

  return GlobalTable<mjpPlugin>::GetSingleton().AppendIfUnique(*plugin);
}

// look up plugin by slot number, assuming that mjp_pluginCount has already been called
const mjpPlugin* mjp_getPluginAtSlotUnsafe(int slot, int nslot) {
  return GlobalTable<mjpPlugin>::GetSingleton().GetAtSlotUnsafe(slot, nslot);
}

// look up plugin by name, assuming that mjp_pluginCount has already been called
const mjpPlugin* mjp_getPluginUnsafe(const char* name, int* slot, int nslot) {
  return GlobalTable<mjpPlugin>::GetSingleton().GetByKeyUnsafe(name, slot, nslot);
}

// return the number of globally registered plugins
int mjp_pluginCount() {
  return GlobalTable<mjpPlugin>::GetSingleton().count();
}

// look up a plugin by slot number
const mjpPlugin* mjp_getPluginAtSlot(int slot) {
  return GlobalTable<mjpPlugin>::GetSingleton().GetAtSlot(slot);
}

// look up a plugin by name, optionally also get its registered slot number
const mjpPlugin* mjp_getPlugin(const char* name, int* slot) {
  return GlobalTable<mjpPlugin>::GetSingleton().GetByKey(name, slot);
}

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
  // check if prefix is valid URI scheme format
  if (!IsValidURISchemeFormat(provider->prefix)) {
    mju_warning("provider->prefix is '%s' which is not a valid URI scheme format",
                provider->prefix);
    return -1;
  }

  if (!provider->open || !provider->read || !provider->close) {
    mju_warning("provider must have the open, read, and close callbacks defined");
    return -1;
  }

  return GlobalTable<mjpResourceProvider>::GetSingleton().AppendIfUnique(*provider) + 1;
}

// return the number of globally registered resource providers
int mjp_resourceProviderCount() {
  return GlobalTable<mjpResourceProvider>::GetSingleton().count();
}

// look up a resource provider that matches its prefix against the given resource scheme
const mjpResourceProvider* mjp_getResourceProvider(const char* resource_name) {
  if (!resource_name || !resource_name[0]) {
    return nullptr;
  }

  const char* ch = std::strchr(resource_name, ':');
  if (ch == nullptr) {
    return nullptr;
  }

  int n = ch - resource_name;
  std::string file_prefix = std::string(resource_name, n);

  // return NULL if file_prefix doesn't have a valid URI scheme syntax
  if (!IsValidURISchemeFormat(file_prefix.c_str())) {
    return nullptr;
  }

  return GlobalTable<mjpResourceProvider>::GetSingleton().GetByKey(file_prefix.c_str(), nullptr);
}

// look up a resource provider by slot number
const mjpResourceProvider* mjp_getResourceProviderAtSlot(int slot) {
  // shift slot to be zero-indexed
  return GlobalTable<mjpResourceProvider>::GetSingleton().GetAtSlot(slot - 1);
}

// load plugins from a dynamic library
void mj_loadPluginLibrary(const char* path) {
#if defined(_WIN32) || defined(__CYGWIN__)
  LoadLibraryA(path);
#else
  void* handle = dlopen(path, RTLD_NOW | RTLD_LOCAL);
  if (!handle) {
    const char* error = dlerror();
    if (error) {
      mju_error("Error loading plugin library '%s': %s\n", path, error);
    } else {
      mju_error("Unknown error loading plugin library '%s'\n", path);
    }
  }
#endif
}

// scan a directory and load all dynamic libraries
void mj_loadAllPluginLibraries(const char* directory,
                               mjfPluginLibraryLoadCallback callback) {
  auto load_dso_and_call_callback = [&](const std::string& filename,
                                        const std::string& dso_path) {
    int nplugin_before;
    int nplugin_after;

    auto& plugin_table = GlobalTable<mjpPlugin>::GetSingleton();
    {
      auto lock = plugin_table.LockExclusively();
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
