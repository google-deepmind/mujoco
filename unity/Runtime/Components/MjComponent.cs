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
using System.Xml;
using UnityEngine;

namespace Mujoco {

// The base class for all components that represent Mujoco scene nodes.
[DisallowMultipleComponent]
public abstract class MjComponent : MonoBehaviour {

  // Unique name assigned to each Mujoco component.
  public string MujocoName { get; private set; }

  // Id of the geom in Mujoco's internal data structures.
  public int MujocoId { get; protected set; }

  public abstract MujocoLib.mjtObj ObjectType { get; }

  // Some components (Inertial frames for example) cannot have the name attribute
  // added to the generated Mjcf.
  protected virtual bool _suppressNameAttribute => false;

  // Binds this component to the compiled Mujoco model.
  public unsafe void BindToRuntime(MujocoLib.mjModel_* model, MujocoLib.mjData_* data) {
    MujocoId = MujocoLib.mj_name2id(model, (int)ObjectType, MujocoName);
    if (MujocoId == -1 && !_suppressNameAttribute) {
      throw new NullReferenceException($"element name {MujocoName} not found");
    }
    OnBindToRuntime(model, data);
  }

  // Generates the XML element that corresponds to this scene node.
  public XmlElement GenerateMjcf(string name, XmlDocument doc) {
    MujocoName = name;

    var mjcf = OnGenerateMjcf(doc);
    if (!_suppressNameAttribute) {
      mjcf.SetAttribute("name", name);
    }

    return mjcf;
  }

  // Parse the component settings from an external Mjcf.
  public void ParseMjcf(XmlElement mjcf) {
    // I would like to preserve the naming dualism - external mechanisms
    // calling a method, and the internal implementation implementing
    // the method with an "On" prefix.
    OnParseMjcf(mjcf);
  }

  // Parse the component settings from an external Mjcf.
  protected abstract void OnParseMjcf(XmlElement mjcf);

  // Generate implementation specific XML element.
  protected abstract XmlElement OnGenerateMjcf(XmlDocument doc);

  // Perform bind time initialization of the component.
  protected virtual unsafe void OnBindToRuntime(MujocoLib.mjModel_* model, MujocoLib.mjData_* data) {}

  // Synchronize the state of the component.
  public virtual unsafe void OnSyncState(MujocoLib.mjData_* data) {}

  private bool _sceneExcludesMe = false;

  protected unsafe virtual void OnEnable() {
    if (MjScene.Instance == null) {
      throw new Exception("MuJoCo Scene not found");
    }
    if (MjScene.Instance.Model != null) {
      _sceneExcludesMe = true;
    }
  }

  protected void Update() {
    if (_sceneExcludesMe) {
      MjScene.Instance.SceneRecreationAtLateUpdateRequested = true;
      _sceneExcludesMe = false;
    }
  }

  private bool _exiting = false;
  public void OnApplicationQuit() {
    _exiting = true;
  }

  public void OnDisable() {
    if (!_exiting && MjScene.InstanceExists) {
      MjScene.Instance.SceneRecreationAtLateUpdateRequested = true;
    }
  }

}
}
