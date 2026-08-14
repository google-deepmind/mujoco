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

  public abstract class MjBaseJoint : MjComponent {
    public override MujocoLib.mjtObj ObjectType => MujocoLib.mjtObj.mjOBJ_JOINT;

    public int QposAddress { get; private set; } = -1;
    public int DofAddress { get; private set; } = -1;
    // Joint will add Degrees of Freedom to this MjBaseBody.
    // It's the MjBaseBody located immediately above this joint in the scene tree.
    // NOTE: This property won't be initialized until runtime.
    public MjBaseBody ParentBody { get; private set; }

    // Joint will constrain the ParentBody to this MjBaseBody.
    // It's the MjBaseBody located two levels above this joint in the scene tree.
    // For some joints, such as MjFreeJoint, this can be null.
    // NOTE: This property won't be initialized until runtime.
    public MjBaseBody GrandParentBody { get; private set; }

    // Return the bodies connected by this joint.
    public void GetConnectedBodies(out MjBaseBody grandParent, out MjBaseBody parent) {
      parent = MjHierarchyTool.FindParentComponent<MjBaseBody>(this);
      if (parent != null) {
        grandParent = MjHierarchyTool.FindParentComponent<MjBaseBody>(parent);
      } else {
        grandParent = null;
      }
    }

    protected override unsafe void OnBindToRuntime(MujocoLib.mjModel_* model, MujocoLib.mjData_* data) {
      QposAddress = model->jnt_qposadr[MujocoId];
      DofAddress = model->jnt_dofadr[MujocoId];
    }

    protected void Start() {
      MjBaseBody grandparentBody, parentBody;
      GetConnectedBodies(out grandparentBody, out parentBody);
      GrandParentBody = grandparentBody;
      ParentBody = parentBody;
      if (ParentBody == null) {
        throw new Exception("The joint doesn't have a ParentBody.");
      }
    }
  }
}
