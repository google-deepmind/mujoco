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

#include <algorithm>
#include <cctype>
#include <charconv>
#include <climits>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <ios>
#include <sstream>
#include <string>
#include <system_error>
#include <unordered_map>
#include <vector>

#include <mujoco/mjplugin.h>
#include <mujoco/mjspec.h>
#include <mujoco/mujoco.h>

namespace {

// store an error message, return false
bool Fail(std::string* err, const char* msg) {
  if (err) {
    *err = std::string("GMSH decoder: ") + msg;
  }
  return false;
}

// Read data of type T from a potentially unaligned buffer pointer.
template <typename T>
void ReadFromBuffer(T* dst, const char* src) {
  std::memcpy(dst, src, sizeof(T));
}

void ReadStrFromBuffer(char* dest, const char* src, int maxlen) {
  std::strncpy(dest, src, maxlen);
}

bool IsValidElementOrNodeHeader22(const std::string& line) {
  for (char c : line) {
    if (!std::isdigit(static_cast<unsigned char>(c))) {
      return false;
    }
  }
  return true;
}

// find string in buffer, return position or -1 if not found
int findstring(const char* buffer, int buffer_sz, const char* str) {
  int len = static_cast<int>(std::strlen(str));
  for (int i = 0; i <= buffer_sz - len; i++) {
    bool found = true;
    for (int k = 0; k < len; k++) {
      if (buffer[i + k] != str[k]) {
        found = false;
        break;
      }
    }
    if (found) {
      return i;
    }
  }
  return -1;
}

// load GMSH format 4.1
bool LoadGMSH41(char* buffer, int binary, int nodeend, int nodebegin,
                int elemend, int elembegin, int& out_dim,
                std::vector<double>& point, std::vector<int>& element,
                std::string* err) {
  constexpr int kGmsh41HeaderSize = 52;
  size_t minNodeTag, numEntityBlocks, numNodes, maxNodeTag, numNodesInBlock,
      tag;
  int entityDim, entityTag, parametric;

  // ascii nodes
  if (binary == 0) {
    std::stringstream ss(std::string(buffer + nodebegin, nodeend - nodebegin));
    ss >> numEntityBlocks >> numNodes >> minNodeTag >> maxNodeTag;
    ss >> entityDim >> entityTag >> parametric >> numNodesInBlock;
    if (!ss.good()) {
      return Fail(err, "Error reading Nodes header");
    }
    if (numEntityBlocks != 1 || numNodes != numNodesInBlock) {
      return Fail(err, "All nodes must be in single block");
    }
    if (maxNodeTag != numNodesInBlock) {
      return Fail(
          err,
          "Maximum number of nodes must be equal to number of nodes in a "
          "block");
    }
    if (entityDim < 1 || entityDim > 3) {
      return Fail(err, "Entity must be 1D, 2D or 3D");
    }
    out_dim = entityDim;

    for (size_t i = 0; i < numNodes; i++) {
      ss >> tag;
      if (!ss.good()) {
        return Fail(err, "Error reading node tags");
      }
      if (tag != i + minNodeTag) {
        return Fail(err, "Node tags must be sequential");
      }
    }

    if (numNodes < 0 || numNodes >= INT_MAX / 3) {
      return Fail(err, "Invalid number of nodes.");
    }
    point.reserve(3 * numNodes);
    for (size_t i = 0; i < 3 * numNodes; i++) {
      double x;
      ss >> x;
      if (!ss.good()) {
        return Fail(err, "Error reading node coordinates");
      }
      point.push_back(x);
    }
  } else {
    // binary nodes
    if (nodeend - nodebegin < kGmsh41HeaderSize) {
      return Fail(err, "Invalid nodes header");
    }

    ReadFromBuffer(&numEntityBlocks, buffer + nodebegin);
    ReadFromBuffer(&numNodes, buffer + nodebegin + 8);
    ReadFromBuffer(&minNodeTag, buffer + nodebegin + 16);
    ReadFromBuffer(&maxNodeTag, buffer + nodebegin + 24);
    ReadFromBuffer(&entityDim, buffer + nodebegin + 32);
    ReadFromBuffer(&entityTag, buffer + nodebegin + 36);
    ReadFromBuffer(&parametric, buffer + nodebegin + 40);
    ReadFromBuffer(&numNodesInBlock, buffer + nodebegin + 44);

    if (numEntityBlocks != 1 || numNodes != numNodesInBlock) {
      return Fail(err, "All nodes must be in single block");
    }
    if (numNodes < 0) {
      return Fail(err, "Invalid number of nodes");
    }
    if (entityDim < 1 || entityDim > 3) {
      return Fail(err, "Entity must be 1D, 2D or 3D");
    }
    out_dim = entityDim;

    constexpr int numNodeComponents = 4;
    constexpr int componentSize = 8;
    int nodeDataSize = numNodeComponents * componentSize;

    if (nodeend - nodebegin < kGmsh41HeaderSize + numNodes * nodeDataSize) {
      return Fail(err, "Insufficient byte size of Nodes");
    }

    const char* tagbuffer = buffer + nodebegin + kGmsh41HeaderSize;
    for (size_t i = 0; i < numNodes; i++) {
      ReadFromBuffer(&tag, tagbuffer + i * componentSize);
      if (tag != i + minNodeTag) {
        return Fail(err, "Node tags must be sequential");
      }
    }

    if (numNodes < 0 || numNodes >= INT_MAX / 3) {
      return Fail(err, "Invalid number of nodes.");
    }
    point.reserve(3 * numNodes);
    const char* pointbuffer =
        buffer + nodebegin + kGmsh41HeaderSize + componentSize * numNodes;
    for (size_t i = 0; i < 3 * numNodes; i++) {
      double x;
      ReadFromBuffer(&x, pointbuffer + i * componentSize);
      point.push_back(x);
    }
  }

  size_t numElements, minElementTag, maxElementTag, numElementsInBlock;
  int elementType;

  // ascii elements
  if (binary == 0) {
    buffer[elemend] = 0;
    std::stringstream ss(std::string(buffer + elembegin, elemend - elembegin));
    ss >> numEntityBlocks >> numElements >> minElementTag >> maxElementTag;
    ss >> entityDim >> entityTag >> elementType >> numElementsInBlock;
    if (!ss.good()) {
      return Fail(err, "Error reading Elements header");
    }
    if (numEntityBlocks != 1 || numElements != numElementsInBlock) {
      return Fail(err, "All elements must be in single block");
    }
    if (numElements < 0) {
      return Fail(err, "Invalid number of elements");
    }
    if (entityDim != out_dim) {
      return Fail(err, "Inconsistent dimensionality in Elements");
    }
    if (numElements < 0 || numElements >= INT_MAX / 4) {
      return Fail(err, "Invalid numElements.");
    }
    if ((entityDim == 1 && elementType != 1) ||
        (entityDim == 2 && elementType != 2) ||
        (entityDim == 3 && elementType != 4)) {
      return Fail(err, "Element type inconsistent with dimensionality");
    }

    element.reserve((entityDim + 1) * numElements);
    for (size_t i = 0; i < numElements; i++) {
      size_t tag_val, nodeid;
      ss >> tag_val;
      for (int k = 0; k <= entityDim; k++) {
        ss >> nodeid;
        if (!ss.good()) {
          return Fail(err, "Error reading Elements");
        }
        if (nodeid < minNodeTag || nodeid - minNodeTag >= numNodes) {
          return Fail(err, "Invalid node index in element");
        }
        element.push_back(static_cast<int>(nodeid - minNodeTag));
      }
    }
  } else {
    // binary elements
    if (elemend - elembegin < kGmsh41HeaderSize) {
      return Fail(err, "Invalid elements header");
    }

    ReadFromBuffer(&numEntityBlocks, buffer + elembegin);
    ReadFromBuffer(&numElements, buffer + elembegin + 8);
    ReadFromBuffer(&minElementTag, buffer + elembegin + 16);
    ReadFromBuffer(&maxElementTag, buffer + elembegin + 24);
    ReadFromBuffer(&entityDim, buffer + elembegin + 32);
    ReadFromBuffer(&entityTag, buffer + elembegin + 36);
    ReadFromBuffer(&elementType, buffer + elembegin + 40);
    ReadFromBuffer(&numElementsInBlock, buffer + elembegin + 44);

    if (numEntityBlocks != 1 || numElements != numElementsInBlock) {
      return Fail(err, "All elements must be in single block");
    }
    if (numElements < 0) {
      return Fail(err, "Invalid number of elements");
    }
    if (entityDim != out_dim) {
      return Fail(err, "Inconsistent dimensionality in Elements");
    }
    if ((entityDim == 1 && elementType != 1) ||
        (entityDim == 2 && elementType != 2) ||
        (entityDim == 3 && elementType != 4)) {
      return Fail(err, "Element type inconsistent with dimensionality");
    }
    if (numElements < 0 || numElements >= INT_MAX / 4) {
      return Fail(err, "Invalid numElements.");
    }

    int numElementComponents = (entityDim + 2);
    constexpr int componentSize = 8;
    int elementDataSize = numElementComponents * componentSize;

    if (elemend - elembegin <
        kGmsh41HeaderSize + numElements * elementDataSize) {
      return Fail(err, "Insufficient byte size of Elements");
    }

    element.reserve((entityDim + 1) * numElements);
    const char* elembuffer = buffer + elembegin + kGmsh41HeaderSize;
    for (size_t i = 0; i < numElements; i++) {
      elembuffer += componentSize;
      size_t elemid;
      for (int k = 0; k <= entityDim; k++) {
        ReadFromBuffer(&elemid, elembuffer);
        if (elemid < minNodeTag || elemid - minNodeTag >= numNodes) {
          return Fail(err, "Invalid node index in element");
        }
        int elementid = static_cast<int>(elemid - minNodeTag);
        element.push_back(elementid);
        elembuffer += componentSize;
      }
    }
  }
  return true;
}

// load GMSH format 2.2
bool LoadGMSH22(char* buffer, int binary, int nodeend, int nodebegin,
                int elemend, int elembegin, int& out_dim,
                std::vector<double>& point, std::vector<int>& element,
                std::string* err) {
  size_t numNodes = 0;

  // ascii nodes
  if (binary == 0) {
    std::stringstream ss(std::string(buffer + nodebegin, nodeend - nodebegin));
    std::string line;
    std::getline(ss, line);
    if (!IsValidElementOrNodeHeader22(line)) {
      return Fail(err, "Invalid node header");
    }
    ss.seekg(-(line.size() + 1), std::ios::cur);

    size_t maxNodeTag = 0;
    ss >> maxNodeTag;
    if (!ss.good()) {
      return Fail(err, "Error reading Nodes header");
    }
    numNodes = maxNodeTag;

    if (numNodes < 0 || numNodes >= INT_MAX / 3) {
      return Fail(err, "Invalid number of nodes.");
    }

    point.reserve(3 * numNodes);
    for (size_t i = 0; i < numNodes; i++) {
      size_t tag_val;
      double x;
      ss >> tag_val;
      if (!ss.good()) {
        return Fail(err, "Error reading node tags");
      }
      for (int k = 0; k < 3; k++) {
        ss >> x;
        if (!ss.good()) {
          return Fail(err, "Error reading node coordinates");
        }
        point.push_back(x);
      }
    }
  } else {
    // binary nodes
    constexpr int nodeHeaderSizeGmshApp = 5;
    constexpr int nodeHeaderSize = nodeHeaderSizeGmshApp - 1;
    if (nodeend - nodebegin < nodeHeaderSize) {
      return Fail(err, "Invalid nodes header");
    }

    char maxNodeTagChar[11] = {0};
    ReadStrFromBuffer(maxNodeTagChar, buffer + nodebegin,
                      std::min(10, nodeend - nodebegin));
    size_t measuredHeaderSize = std::strlen(maxNodeTagChar) - 1;
    size_t maxNodeTag = 0;
    auto [node_ptr, node_ec] = std::from_chars(
        maxNodeTagChar, maxNodeTagChar + measuredHeaderSize + 1, maxNodeTag);
    if (node_ec != std::errc() || node_ptr == maxNodeTagChar) {
      return Fail(err, "Invalid number of nodes");
    }
    numNodes = maxNodeTag;

    if (numNodes < 0) {
      return Fail(err, "Invalid number of nodes");
    }

    int nodeSize = sizeof(double);
    int indexSize = sizeof(int);
    int nodeDataSize = indexSize + 3 * nodeSize;

    if (nodeend - nodebegin < nodeHeaderSize + numNodes * nodeDataSize) {
      return Fail(err, "Insufficient byte size of Nodes");
    }

    if (numNodes < 0 || numNodes >= INT_MAX / 3) {
      return Fail(err, "Invalid number of nodes.");
    }
    point.reserve(3 * numNodes);
    const char* tagBuffer = buffer + nodebegin + measuredHeaderSize;
    for (size_t i = 0; i < numNodes; i++) {
      int tag_val;
      int offset = i * (sizeof(int) + sizeof(double) * 3);
      ReadFromBuffer(&tag_val, tagBuffer + offset);
      for (int k = 0; k < 3; k++) {
        double x;
        const char* nodeBuffer = tagBuffer + sizeof(int) + sizeof(double) * k;
        ReadFromBuffer(&x, nodeBuffer + offset);
        point.push_back(x);
      }
    }
  }

  // ascii elements
  if (binary == 0) {
    buffer[elemend] = 0;
    std::stringstream ss(std::string(buffer + elembegin, elemend - elembegin));
    std::string line;
    std::getline(ss, line);
    if (!IsValidElementOrNodeHeader22(line)) {
      return Fail(err, "Invalid elements header");
    }
    ss.seekg(-(line.size() + 1), std::ios::cur);
    size_t maxElementTag = 0;
    ss >> maxElementTag;
    if (!ss.good()) {
      return Fail(err, "Error reading Elements header");
    }
    size_t numElements = maxElementTag;

    if (numElements < 0 || numElements >= INT_MAX / 4) {
      return Fail(err, "Invalid number of elements.");
    }
    if (numElements < 0) {
      return Fail(err, "Invalid number of elements");
    }

    int tag_val = 0, elementType = 0, numTags = 0;
    ss >> tag_val >> elementType >> numTags;
    if (!ss.good()) {
      return Fail(err, "Error reading Elements");
    }

    size_t entityDim = 0;
    int numNodeTags = 0;
    if (elementType == 2) {
      entityDim = 2;
      numNodeTags = 3;
    } else if (elementType == 4) {
      entityDim = 3;
      numNodeTags = 4;
    }

    if (numNodeTags < 1 || numNodeTags > 4) {
      return Fail(err, "Invalid number of node tags");
    }

    out_dim = entityDim;

    element.reserve(numNodeTags * numElements);
    for (size_t i = 0; i < numElements; i++) {
      int nodeTag = 0, physicalEntityTag = 0, elementModelEntityTag = 0;
      if (i != 0) {
        ss >> tag_val >> elementType >> numTags;
        if (!ss.good()) {
          return Fail(err, "Error reading Elements");
        }
      }
      if (numTags > 0) {
        ss >> physicalEntityTag >> elementModelEntityTag;
        if (!ss.good()) {
          return Fail(err, "Error reading Elements");
        }
      }
      for (int k = 0; k < numNodeTags; k++) {
        ss >> nodeTag;
        if (!ss.good()) {
          return Fail(err, "Error reading Elements");
        }
        if (nodeTag > numNodes || nodeTag < 1) {
          return Fail(err, "Invalid node tag");
        }
        element.push_back(nodeTag - 1);
      }
    }
  } else {
    // binary elements
    constexpr int elementHeaderSizeGmshApp = 4;
    constexpr int elementHeaderSizeFtetwild = 17;
    if (elemend - elembegin < elementHeaderSizeGmshApp) {
      return Fail(err, "Invalid elements header");
    }

    char maxElementTagChar[11] = {0};
    ReadStrFromBuffer(maxElementTagChar, buffer + elembegin,
                      std::min(10, elemend - elembegin));
    int measuredHeaderSize =
        static_cast<int>(std::strlen(maxElementTagChar)) - 1;
    int maxElementTag = 0;
    auto [elem_ptr, elem_ec] = std::from_chars(
        maxElementTagChar, maxElementTagChar + measuredHeaderSize + 1,
        maxElementTag);
    if (elem_ec != std::errc() || elem_ptr == maxElementTagChar ||
        maxElementTag < 0) {
      return Fail(err, "Invalid number of elements");
    }
    int numElements = maxElementTag;
    int tag_val, numTags;
    int nodeTag;
    int elementType;

    if (numElements < 0) {
      return Fail(err, "Invalid number of elements");
    }

    int componentSize = sizeof(int);
    const char* elementsBuffer = buffer + elembegin + measuredHeaderSize;
    ReadFromBuffer(&elementType, elementsBuffer);
    ReadFromBuffer(&numTags, elementsBuffer + componentSize * 2);
    ReadFromBuffer(&tag_val, elementsBuffer + componentSize * 3);

    int numNodeTags = 0;
    size_t entityDim = 0;
    if (elementType == 2) {
      entityDim = 2;
      numNodeTags = 3;
    } else if (elementType == 4) {
      entityDim = 3;
      numNodeTags = 4;
    }

    if (numNodeTags < 1 || numNodeTags > 4) {
      return Fail(err, "Invalid number of node tags");
    }

    out_dim = entityDim;

    constexpr int numComponentsFtetwild = 5;
    constexpr int numInfoComponents = 4;
    constexpr int numEntityTagComponents = 2;
    int numComponentsGmshApp =
        numInfoComponents + numEntityTagComponents + numNodeTags;

    int elementDataSizeFtetwild = numComponentsFtetwild * componentSize;
    int elementDataSizeGmshApp = numComponentsGmshApp * componentSize;

    int elementsBufferSizeFtetwild =
        elementHeaderSizeFtetwild + numElements * elementDataSizeFtetwild;
    int elementsBufferSizeGmshApp =
        elementHeaderSizeGmshApp + numElements * elementDataSizeGmshApp;

    if (elemend - elembegin < elementsBufferSizeFtetwild) {
      return Fail(err, "Insufficient byte size of Elements");
    }

    if (numTags > 0) {
      if (elemend - elembegin < elementsBufferSizeGmshApp) {
        return Fail(err, "Insufficient byte size of Elements");
      }

      for (int k = 0; k < numNodeTags; k++) {
        ReadFromBuffer(&nodeTag, elementsBuffer + componentSize * (6 + k));
        if (nodeTag > numNodes || nodeTag < 1) {
          return Fail(err, "Invalid node tag");
        }
        element.push_back(nodeTag - 1);
      }

      for (int i = 1; i < numElements; i++) {
        const char* numTagsBuffer = elementsBuffer + componentSize * 2;
        const char* tagBuffer = elementsBuffer + componentSize * 3;
        int offset = i * elementDataSizeGmshApp;
        ReadFromBuffer(&numTags, numTagsBuffer + offset);
        ReadFromBuffer(&tag_val, tagBuffer + offset);
        for (int k = 0; k < numNodeTags; k++) {
          const char* nodeTagBuffer = elementsBuffer + componentSize * (6 + k);
          ReadFromBuffer(&nodeTag, nodeTagBuffer + offset);
          if (nodeTag > numNodes || nodeTag < 1) {
            return Fail(err, "Invalid node tag");
          }
          element.push_back(nodeTag - 1);
        }
      }
    } else {
      for (int k = 0; k < numNodeTags; k++) {
        const char* nodeTagBuffer = elementsBuffer + componentSize * (4 + k);
        ReadFromBuffer(&nodeTag, nodeTagBuffer);
        if (nodeTag > numNodes || nodeTag < 1) {
          return Fail(err, "Invalid node tag");
        }
        element.push_back(nodeTag - 1);
      }

      for (int i = 0; i < numElements - 1; i++) {
        int offset = componentSize * (4 + 2) + i * elementDataSizeFtetwild;
        const char* tagBuffer = elementsBuffer + componentSize * 2;
        ReadFromBuffer(&tag_val, tagBuffer + offset);
        for (int k = 0; k < numNodeTags; k++) {
          const char* nodeTagBuffer = elementsBuffer + componentSize * (3 + k);
          ReadFromBuffer(&nodeTag, nodeTagBuffer + offset);
          if (nodeTag > numNodes || nodeTag < 1) {
            return Fail(err, "Invalid node tag");
          }
          element.push_back(nodeTag - 1);
        }
      }
    }
  }
  return true;
}

// Extract boundary triangle faces and compact boundary vertices from 4-node
// tetrahedra
void ExtractBoundaryFaces(const std::vector<double>& point,
                          const std::vector<int>& element,
                          std::vector<float>& boundary_verts,
                          std::vector<int>& boundary_faces) {
  struct FaceKey {
    int v[3];
    bool operator==(const FaceKey& other) const {
      return v[0] == other.v[0] && v[1] == other.v[1] && v[2] == other.v[2];
    }
  };

  struct FaceKeyHash {
    size_t operator()(const FaceKey& k) const {
      size_t h = 0;
      h ^= std::hash<int>{}(k.v[0]) + 0x9e3779b9 + (h << 6) + (h >> 2);
      h ^= std::hash<int>{}(k.v[1]) + 0x9e3779b9 + (h << 6) + (h >> 2);
      h ^= std::hash<int>{}(k.v[2]) + 0x9e3779b9 + (h << 6) + (h >> 2);
      return h;
    }
  };

  struct FaceCandidate {
    int tri[3];
    bool is_boundary;
  };

  std::vector<FaceCandidate> candidates;
  std::unordered_map<FaceKey, int, FaceKeyHash> face_map;
  int num_tets = static_cast<int>(element.size() / 4);
  candidates.reserve(num_tets * 4);
  face_map.reserve(num_tets * 4);

  for (int i = 0; i < num_tets; ++i) {
    int v0 = element[4 * i + 0];
    int v1 = element[4 * i + 1];
    int v2 = element[4 * i + 2];
    int v3 = element[4 * i + 3];

    // Compute signed volume: 1/6 * ((p1 - p0) x (p2 - p0)) . (p3 - p0)
    const double* p0 = &point[3 * v0];
    const double* p1 = &point[3 * v1];
    const double* p2 = &point[3 * v2];
    const double* p3 = &point[3 * v3];

    double d1[3] = {p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]};
    double d2[3] = {p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]};
    double d3[3] = {p3[0] - p0[0], p3[1] - p0[1], p3[2] - p0[2]};

    double cross[3] = {
        d1[1] * d2[2] - d1[2] * d2[1],
        d1[2] * d2[0] - d1[0] * d2[2],
        d1[0] * d2[1] - d1[1] * d2[0],
    };
    double vol = cross[0] * d3[0] + cross[1] * d3[1] + cross[2] * d3[2];

    // If volume is negative, swap v0 and v1 to ensure positive orientation
    if (vol < 0) {
      std::swap(v0, v1);
    }

    // Outward faces of positively oriented tetrahedron (v0, v1, v2, v3):
    int faces[4][3] = {
        {v1, v2, v3},
        {v0, v3, v2},
        {v0, v1, v3},
        {v0, v2, v1},
    };

    for (int f = 0; f < 4; ++f) {
      int a = faces[f][0];
      int b = faces[f][1];
      int c = faces[f][2];
      int s[3] = {a, b, c};
      std::sort(s, s + 3);
      FaceKey key = {s[0], s[1], s[2]};
      auto it = face_map.find(key);
      if (it == face_map.end()) {
        int idx = static_cast<int>(candidates.size());
        candidates.push_back({{a, b, c}, true});
        face_map.emplace(key, idx);
      } else {
        candidates[it->second].is_boundary = false;
      }
    }
  }

  int num_nodes = static_cast<int>(point.size() / 3);
  std::vector<bool> is_boundary_node(num_nodes, false);
  for (const auto& cand : candidates) {
    if (cand.is_boundary) {
      is_boundary_node[cand.tri[0]] = true;
      is_boundary_node[cand.tri[1]] = true;
      is_boundary_node[cand.tri[2]] = true;
    }
  }

  std::vector<int> old_to_new(num_nodes, -1);
  boundary_verts.clear();
  for (int i = 0; i < num_nodes; ++i) {
    if (is_boundary_node[i]) {
      old_to_new[i] = static_cast<int>(boundary_verts.size() / 3);
      boundary_verts.push_back(static_cast<float>(point[3 * i + 0]));
      boundary_verts.push_back(static_cast<float>(point[3 * i + 1]));
      boundary_verts.push_back(static_cast<float>(point[3 * i + 2]));
    }
  }

  boundary_faces.clear();
  for (const auto& cand : candidates) {
    if (cand.is_boundary) {
      boundary_faces.push_back(old_to_new[cand.tri[0]]);
      boundary_faces.push_back(old_to_new[cand.tri[1]]);
      boundary_faces.push_back(old_to_new[cand.tri[2]]);
    }
  }
}

mjSpec* DecodeImpl(mjResource* resource, std::string* err) {
  const void* bytes = nullptr;
  int buffer_sz = mju_readResource(resource, &bytes);

  if (buffer_sz < 0) {
    Fail(err, "Could not read GMSH file");
    return nullptr;
  }
  if (buffer_sz == 0) {
    Fail(err, "Empty GMSH file");
    return nullptr;
  }
  if (buffer_sz < 11 ||
      std::strncmp(static_cast<const char*>(bytes), "$MeshFormat", 11) != 0) {
    Fail(err, "GMSH file must begin with $MeshFormat");
    return nullptr;
  }

  // Work on a mutable copy to safely null-terminate during tokenizing.
  std::vector<char> buffer(buffer_sz + 1, 0);
  std::memcpy(buffer.data(), bytes, buffer_sz);
  char* buf = buffer.data();

  constexpr int kGmshVersionLineMax = 64;
  std::stringstream header(
      std::string(buf + 11, std::min(buffer_sz - 11, kGmshVersionLineMax)));
  double version;
  int binary;
  if (!(header >> version >> binary)) {
    Fail(err, "Could not read GMSH file header");
    return nullptr;
  }
  if (mju_round(100 * version) != 220 && mju_round(100 * version) != 410) {
    Fail(err, "Only GMSH file format versions 4.1 and 2.2 are supported");
    return nullptr;
  }

  int nodebegin = findstring(buf, buffer_sz, "$Nodes");
  int nodeend = findstring(buf, buffer_sz, "$EndNodes");
  int elembegin = findstring(buf, buffer_sz, "$Elements");
  int elemend = findstring(buf, buffer_sz, "$EndElements");

  if (nodebegin < 0) {
    Fail(err, "GMSH file missing $Nodes");
    return nullptr;
  }
  if (elembegin < 0) {
    Fail(err, "GMSH file missing $Elements");
    return nullptr;
  }

  nodebegin += static_cast<int>(std::strlen("$Nodes")) + 1;
  elembegin += static_cast<int>(std::strlen("$Elements")) + 1;

  if (nodeend < nodebegin) {
    Fail(err, "GMSH file missing $EndNodes after $Nodes");
    return nullptr;
  }
  if (elemend < elembegin) {
    Fail(err, "GMSH file missing $EndElements after $Elements");
    return nullptr;
  }

  int entityDim = 0;
  std::vector<double> point;
  std::vector<int> element;

  if (mju_round(100 * version) == 410) {
    if (!LoadGMSH41(buf, binary, nodeend, nodebegin, elemend, elembegin,
                    entityDim, point, element, err)) {
      return nullptr;
    }
  } else if (mju_round(100 * version) == 220) {
    if (!LoadGMSH22(buf, binary, nodeend, nodebegin, elemend, elembegin,
                    entityDim, point, element, err)) {
      return nullptr;
    }
  } else {
    Fail(err, "Unsupported GMSH file format version");
    return nullptr;
  }

  // 1D meshes (line elements) are not supported
  if (entityDim == 1) {
    Fail(err, "1D meshes are not supported");
    return nullptr;
  }

  mjSpec* spec = mj_makeSpec();

  // Mesh specification
  mjsMesh* mesh = mjs_addMesh(spec, nullptr);
  mjs_setString(mesh->file, resource->name);

  // Node positions in double precision
  mjs_setDouble(mesh->usernode, point.data(), point.size());

  if (entityDim == 3) {
    // 3D tetrahedral mesh: store tets in usertet, extract boundary surface
    // into uservert and userface.
    mjs_setInt(mesh->usertet, element.data(), element.size());
    std::vector<float> boundary_verts;
    std::vector<int> boundary_faces;
    ExtractBoundaryFaces(point, element, boundary_verts, boundary_faces);
    mjs_setFloat(mesh->uservert, boundary_verts.data(), boundary_verts.size());
    mjs_setInt(mesh->userface, boundary_faces.data(), boundary_faces.size());
  } else {
    // 2D surface mesh
    std::vector<float> point_f(point.begin(), point.end());
    mjs_setFloat(mesh->uservert, point_f.data(), point_f.size());
    mjs_setInt(mesh->userface, element.data(), element.size());
  }

  return spec;
}

mjSpec* Decode(mjResource* resource, const mjVFS* vfs, char* error,
               int error_sz) {
  std::string err;
  mjSpec* spec = DecodeImpl(resource, &err);
  if (!spec && error && error_sz > 0) {
    std::snprintf(error, error_sz, "%s", err.c_str());
  }
  return spec;
}

int CanDecode(const mjResource* resource) {
  if (!resource || !resource->name) {
    return 0;
  }
  const void* bytes = nullptr;
  int buffer_sz = mju_readResource(const_cast<mjResource*>(resource), &bytes);
  if (buffer_sz >= 11 &&
      std::strncmp(static_cast<const char*>(bytes), "$MeshFormat", 11) == 0) {
    return 1;
  }
  return 0;
}

}  // namespace

mjPLUGIN_LIB_INIT(gmsh_decoder) {
  mjpDecoder decoder;
  mjp_defaultDecoder(&decoder);
  decoder.content_type = "model/vnd.gmsh";
  decoder.extension = ".msh|.gmsh";
  decoder.decode = Decode;
  decoder.can_decode = CanDecode;
  mjp_registerDecoder(&decoder);
}
