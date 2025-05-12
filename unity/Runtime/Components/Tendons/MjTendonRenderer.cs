// Copyright 2019 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Mujoco {

public class MjTendonRenderer : MonoBehaviour {

  // We serialize these to give the option to remove tendons from the list of rendered tendons
  [SerializeField]
  public MjSpatialTendon[] SpatialTendons;

  // If a tendon does not have an attached LineRenderer, we don't update it, so this may be a
  // subset of _spatialTendons
  private (MjSpatialTendon, LineRenderer)[] _renderedTendons;

  [SerializeField]
  public TendonRenderingDefaults LineRendererDefaults;

  public (MjSpatialTendon, LineRenderer)[] RenderedTendons {
    get {
      return SpatialTendons
          .Where(st => st.GetComponent<LineRenderer>())
          .Select(st => st.GetComponent<LineRenderer>())
          .Select(lr => (lr.GetComponent<MjSpatialTendon>(), lr))
          .ToArray();
    }
  }

  void Reset() {
    SpatialTendons = GetComponentsInChildren<MjSpatialTendon>();
  }

  void Start() {
    _renderedTendons = RenderedTendons;
    MjScene.Instance.postUpdateEvent +=
        (sender, args) => UpdateTendons(sender, args, _renderedTendons);
  }

  public unsafe void UpdateTendons(object sender, MjStepArgs args,
      (MjSpatialTendon, LineRenderer)[] renderedTendons) {
    var d = args.data;
    var m = args.model;
    foreach (var (tendon, renderer) in renderedTendons) {
      int i = tendon.MujocoId;
      List<Vector3> positions = new List<Vector3>();
      double sz;

      for (int j = d->ten_wrapadr[i]; j < d->ten_wrapadr[i] + d->ten_wrapnum[i]; j++) {
        // Skip drawing of pulley (-2):
        if (d->wrap_obj[j] != -2 && d->wrap_obj[j + 1] != -2) {
          // TODO: Consider handling of wrapping segments
          /*if (d->wrap_obj[j] >= 0 && d->wrap_obj[j + 1] >= 0) {
            sz = 0.5 * m->tendon_width[i];
          } else {
            sz = m->tendon_width[i];
          }*/

          positions.Add(MjEngineTool.UnityVector3(d->wrap_xpos + 3 * j));
        }
      }
      renderer.positionCount = positions.Count;
      renderer.SetPositions(positions.ToArray());
    }
  }

  [Serializable]
  public class TendonRenderingDefaults {

    [SerializeField]
    public Color DefaultColor;

    [SerializeField]
    public Material DefaultMaterial;

    [SerializeField]
    public bool OverwriteExisting;

    [SerializeField]
    public int NumVertices = 12;

    [SerializeField]
    public bool ScaleByWidth;

    [SerializeField]
    public float WidthMultiplier = 1;

    [SerializeField]
    public bool GenerateLightingData;

  }
}
}