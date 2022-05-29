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
using System.Xml;
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

    return mjcf;
  }

  public unsafe void OnDrawGizmos() {
    if (Application.isPlaying) {
      var scene = MjScene.Instance;
      var segmentCount = scene.Model->nwrap - 1;
      var data = scene.Data;
      int readHead = 0;
      for (int i = 0; i < segmentCount; ++i) {
        // Skip drawing of pulley:
        if (data->wrap_obj[readHead] == -2 || data->wrap_obj[readHead + 1] == -2) {
          readHead += 1;
        } else {
          Vector3 start = MjEngineTool.UnityVector3(
              MjEngineTool.MjVector3AtEntry(data->wrap_xpos, readHead));
          Vector3 end = MjEngineTool.UnityVector3(
              MjEngineTool.MjVector3AtEntry(data->wrap_xpos, readHead + 1));
          Gizmos.DrawLine(start, end);
          if (data->wrap_obj[readHead + 1] >= 0) { // a wrap geom
            readHead += 2;
          } else { // site or pulley
            readHead += 1;
          }
        }
      }
    }
  }
}
}
