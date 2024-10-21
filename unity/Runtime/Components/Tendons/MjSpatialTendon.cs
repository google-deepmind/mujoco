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
using System.Diagnostics;
using System.Linq;
using System.Xml;
#if UNITY_EDITOR
using UnityEditor;
#endif
using UnityEngine;

namespace Mujoco {
[Serializable]
public class SpatialTendonEntry {
  [Tooltip("Only one of site/geom/pulley can be specified at each entry.")]
  public MjSite Site;
  [Tooltip("Only one of site/geom/pulley can be specified at each entry.")]
  public MjGeom WrapGeom;
  [Tooltip("Only used if wrap geom is specified.")]
  public MjSite WrapSideSite;
  [Tooltip("Only one of site/geom/pulley can be specified at each entry.")]
  public float PulleyDivisor;
}

public class MjSpatialTendon : MjBaseTendon {

  [Tooltip("In meters.")]
  public float RangeLower;
  [Tooltip("In meters.")]
  public float RangeUpper;

  [Tooltip("Scales with UI points. 0.003 is mapped to 1 UI point.")]
  public float Width = 0.003f;

  public List<SpatialTendonEntry> ViapointsList = new List<SpatialTendonEntry>() {};

  protected override void FromMjcf(XmlElement mjcf) {
    foreach (var child in mjcf.Cast<XmlNode>().OfType<XmlElement>()) {
      var viapoint = new SpatialTendonEntry();
      viapoint.Site = child.GetObjectReferenceAttribute<MjSite>("site");
      viapoint.WrapGeom = child.GetObjectReferenceAttribute<MjGeom>("geom");
      viapoint.WrapSideSite = child.GetObjectReferenceAttribute<MjSite>("sidesite");
      viapoint.PulleyDivisor = child.GetFloatAttribute("pulley");
      ViapointsList.Add(viapoint);
    }
    var rangeValues = mjcf.GetFloatArrayAttribute("range", defaultValue: new float[] { 0, 0 });
    Width = mjcf.GetFloatAttribute("width", defaultValue: 0.003f);
    RangeLower = rangeValues[0];
    RangeUpper = rangeValues[1];
  }

  protected override XmlElement ToMjcf(XmlDocument doc) {
    if (ViapointsList.Count() < 2) {
      throw new ArgumentOutOfRangeException($"Spatial tendon {name} needs at least 2 sites.");
    }
    if (ViapointsList[0].Site == null) {
      throw new NullReferenceException(
      $"The first entry of a spatial tendon {name} must be a site.");
    }
    if (ViapointsList[ViapointsList.Count - 1].Site == null) {
      throw new NullReferenceException(
      $"The last entry of a spatial tendon {name} must be a site.");
    }
    var mjcf = doc.CreateElement("spatial");
    foreach (SpatialTendonEntry spatialTendonEntry in ViapointsList) {
      XmlElement elementMjcf;
      if (spatialTendonEntry.Site) {
        if (spatialTendonEntry.WrapGeom || spatialTendonEntry.PulleyDivisor != 0) {
          throw new ArgumentException(
          $"Only one of site/wrap geom/pulley {name} is allowed at each entry.");
        }
        elementMjcf = doc.CreateElement("site");
        elementMjcf.SetAttribute("site", spatialTendonEntry.Site.MujocoName);
      } else if (spatialTendonEntry.WrapGeom) {
        if (spatialTendonEntry.PulleyDivisor != 0) {
          throw new ArgumentException(
          $"Only one of site/wrap geom/pulley {name} is allowed at each entry.");
        }
        elementMjcf = doc.CreateElement("geom");
        elementMjcf.SetAttribute("geom", spatialTendonEntry.WrapGeom.MujocoName);
        if (spatialTendonEntry.WrapSideSite) {
          elementMjcf.SetAttribute("sidesite", spatialTendonEntry.WrapSideSite.MujocoName);
        }
      } else {
        if (spatialTendonEntry.PulleyDivisor == 0) {
          throw new ArgumentException(
          $"Exactly one of site/wrap geom/pulley {name} is required at each entry.");
        }
        elementMjcf = doc.CreateElement("pulley");
        elementMjcf.SetAttribute("divisor", MjEngineTool.MakeLocaleInvariant($"{spatialTendonEntry.PulleyDivisor}"));
      }
      mjcf.AppendChild(elementMjcf);
    }
    if (RangeLower > RangeUpper) {
      throw new ArgumentException("Lower range value can't be bigger than Higher");
    }
    mjcf.SetAttribute("range", MjEngineTool.MakeLocaleInvariant($"{RangeLower} {RangeUpper}"));
    mjcf.SetAttribute("width", MjEngineTool.MakeLocaleInvariant($"{Width}"));

    return mjcf;
  }

#if UNITY_EDITOR
  public unsafe void OnDrawGizmosSelected() {
    if (Application.isPlaying && MjScene.InstanceExists) {
      // Lifted from src/engine/engine_vis_visualize.c
      var d = MjScene.Instance.Data;
      var m = MjScene.Instance.Model;
      int i = MujocoId;
      double sz;
      for (int j = d->ten_wrapadr[i]; j < d->ten_wrapadr[i] + d->ten_wrapnum[i] - 1; j++) {
        // Skip drawing of pulley (-2):
        if (d->wrap_obj[j] != -2 && d->wrap_obj[j + 1] != -2) {
          // determine width: smaller for segments inside wrapping objects
          if (d->wrap_obj[j] >= 0 && d->wrap_obj[j + 1] >= 0) {
            sz = 0.5 * m->tendon_width[i];
          } else {
            sz = m->tendon_width[i];
          }

          var startPos = MjEngineTool.UnityVector3(d->wrap_xpos + 3 * j);
          var endPos = MjEngineTool.UnityVector3(d->wrap_xpos + 3 * j + 3);

          // Construct line
          // At the moment we use handles instead of Gizmos (to be able to set width).
          // This means rendering won't be performed outside editor; We could implement a "TendonRenderer" component
          // (added using a context tool) that uses a LineRenderer. Then we could have nice features such as updating
          // tendon colors or width based on length or tension. That would also allow in-build rendering.
          Handles.color = Color.magenta;  // Currently we ignore tendon material.
          #if UNITY_2020_2_OR_NEWER
          Handles.DrawLine(startPos, endPos, (float)sz/0.003f);  // Handle width in UI points. We map the default width to 1 pixel.
          #else
          Handles.DrawLine(startPos, endPos);
          #endif
        }
      }
    }
  }
#endif
}
}
