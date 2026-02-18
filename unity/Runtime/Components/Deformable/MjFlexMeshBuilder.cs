using System;
using System.Collections;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Text.RegularExpressions;
using Unity.Collections;
using UnityEngine;

namespace Mujoco {

/// <summary>
/// This is a very early stage mesh generator for flex objects.
/// Texture mapping in particular is very rudimentary.
/// Use the context menu button to generate mesh from the inspector.
/// </summary>
[RequireComponent(typeof(SkinnedMeshRenderer))]
[ExecuteInEditMode]
public class MjFlexMeshBuilder : MonoBehaviour {

  private SkinnedMeshRenderer _meshRenderer;

  [SerializeField]
  MjFlexDeformable flex;

  [SerializeField]
  Vector3 uvProjectionU;

  [SerializeField]
  Vector3 uvProjectionV;

  [SerializeField]
  bool doubleSided;

  protected void Awake() {
    _meshRenderer = GetComponent<SkinnedMeshRenderer>();
  }

  [ContextMenu("Generate Mesh")]
  public void GenerateSkinnedMesh() {
    DisposeCurrentMesh();
    List<Vector3> vertices = flex.Body.Select(b => b.transform.localPosition).ToList();
    if (doubleSided) vertices.AddRange(vertices.ToArray().Reverse());
    List<Transform> transforms = flex.Body.Select(b => b.transform).ToList();
    if (doubleSided) transforms.AddRange(transforms.ToArray().Reverse());
    var mesh = new Mesh();
    mesh.name = $"Mujoco mesh for composite {name}";
    mesh.vertices = vertices.ToArray();
    var triangles = flex.Element.ToList();
    if (doubleSided) triangles.AddRange(triangles.ToArray().Reverse().Select(t => vertices.Count / 2 + t));
    mesh.triangles = triangles.ToArray();


    // Generate UVs based on vertex positions
    Vector2[] uv = new Vector2[vertices.Count];


    // Flexcoord could be used for UV
    if (flex.Texcoord != null && flex.Texcoord.Length == vertices.Count * 2) {
      Vector2[] texcoords = new Vector2[vertices.Count];
      for (int i = 0; i < vertices.Count; i++) {
        texcoords[i] = new Vector2(flex.Texcoord[i * 2], flex.Texcoord[i * 2 + 1]);
      }
      mesh.uv = texcoords;
    } else {
      for (int i = 0; i < vertices.Count; i++) {
        // Assuming horizontal layout
        uv[i] = new Vector2(
          Mathf.InverseLerp(vertices.Min(u => Vector3.Dot(u, uvProjectionU)),
            vertices.Max(u => Vector3.Dot(u, uvProjectionU)),
            Vector3.Dot(vertices[i], uvProjectionU)),
          Mathf.InverseLerp(vertices.Min(u => Vector3.Dot(u, uvProjectionV)),
            vertices.Max(u => Vector3.Dot(u, uvProjectionV)),
            Vector3.Dot(vertices[i], uvProjectionV))
        );
      }
      mesh.uv = uv;
    }

    // Calculate tangents
    mesh.RecalculateNormals();
    Vector4[] tangents = new Vector4[vertices.Count];
    Vector3[] normals = mesh.normals;

    for (int i = 0; i < vertices.Count; i++) {
      Vector3 normal = normals[i];
      Vector3 tangent = Vector3.Cross(normal, Vector3.up).normalized;
      if (tangent.magnitude < 0.01f) {
        tangent = Vector3.Cross(normal, Vector3.right).normalized;
      }
      tangents[i] = new Vector4(tangent.x, tangent.y, tangent.z, -1f);
    }
    mesh.tangents = tangents;

    // Bone weights setup
    byte[] bonesPerVertex = Enumerable.Repeat(1, vertices.Count)
        .Select(i => (byte)i).ToArray();
    BoneWeight1[] boneWeights = transforms
        .Select((t, i) => new BoneWeight1 {boneIndex = i, weight = 1}).ToArray();
    mesh.SetBoneWeights(new NativeArray<byte>(bonesPerVertex, Allocator.Temp),
      new NativeArray<BoneWeight1>(boneWeights, Allocator.Temp));
    mesh.bindposes = transforms
        .Select(t => Matrix4x4.TRS(t.localPosition, t.localRotation, Vector3.one).inverse)
        .ToArray();
    _meshRenderer.sharedMesh = mesh;
    _meshRenderer.bones = transforms.ToArray();
  }

  protected void OnDestroy() {
    DisposeCurrentMesh();
  }

  // Dynamically created meshes with no references are only disposed automatically on scene changes.
  // This prevents resource leaks in case the host environment doesn't reload scenes.
  private void DisposeCurrentMesh() {
    if (_meshRenderer.sharedMesh != null) {
#if UNITY_EDITOR
      DestroyImmediate(_meshRenderer.sharedMesh);
#else
      Destroy(_meshFilter.sharedMesh);
#endif
    }
  }
}
}
