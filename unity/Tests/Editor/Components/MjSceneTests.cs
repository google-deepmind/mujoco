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
using System.Collections;
using System.Collections.Generic;
using System.Xml;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools.Utils;

namespace Mujoco {

[TestFixture]
public class MjSceneGenerationTests {

  [Test]
  public void AssigningComponentsUniqueName() {
    _fakeBodyA.enabled = true;
    _fakeBodyB.enabled = true;
    _scene.CreateScene(skipCompile:true);
    Assert.That(_fakeBodyA.MujocoName, Is.Not.EqualTo(_fakeBodyB.MujocoName));
  }

  [Test]
  public void AttachingComponentsToMjNodesThroughId() {
    _fakeBodyA.enabled = true;
    _fakeBodyB.enabled = true;
    _scene.CreateScene();
    Assert.That(_fakeBodyA.MujocoId, Is.GreaterThanOrEqualTo(0));
    Assert.That(_fakeBodyB.MujocoId, Is.GreaterThanOrEqualTo(0));
    Assert.That(_fakeBodyA.MujocoId, Is.Not.EqualTo(_fakeBodyB.MujocoId));
  }

  [Test]
  public unsafe void ModelStructureIsInitialized() {
    _fakeBodyA.enabled = true;
    _fakeBodyB.enabled = true;
    _scene.CreateScene();
    Assert.That(_scene.Model->nbody, Is.GreaterThanOrEqualTo(2));
  }

  [Test]
  public void SyncingComponentStateOnFixedUpdate() {
    _fakeBodyA.enabled = true;
    _fakeBodyB.enabled = true;
    _scene.CreateScene();
    Assert.That(_fakeBodyA.StateSynced, Is.False);
    Assert.That(_fakeBodyB.StateSynced, Is.False);
    _scene.StepScene();
    Assert.That(_fakeBodyA.StateSynced, Is.True);
    Assert.That(_fakeBodyB.StateSynced, Is.True);
  }

  [Test]
  public unsafe void PhysicsRuntimeError() {
    _scene.CreateScene();
    _scene.Data->qpos[0] = float.PositiveInfinity;
    Assert.That(
        () => { _scene.StepScene(); },
        Throws.TypeOf<PhysicsRuntimeException>()
        .With.Message.EqualTo("BADQPOS: NaN/inf in qpos."));
  }

  [Test]
  public void MjcfHierarchyMirrorsTheHierarchyOfUnityObjects() {
    _fakeBodyA.transform.parent = _fakeBodyB.transform;
    _fakeBodyA.enabled = true;
    _fakeBodyB.enabled = true;
    var mjcf = _scene.CreateScene(skipCompile:true);
    // Three components are already in this scene: body, joint and inertia (see SetUp function below)
    var mjcfFakeBodyB = mjcf.SelectNodes("/mujoco/worldbody/body")[1] as XmlElement;
    var mjcfFakeBodyA = mjcf.SelectNodes("/mujoco/worldbody/body/body")[0] as XmlElement;
    Assert.That(mjcfFakeBodyA, Is.Not.Null);
    Assert.That(mjcfFakeBodyB, Is.Not.Null);
    // B is the parent, so it appears first in the XML
    Assert.That(mjcfFakeBodyB.GetAttribute("name"), Is.EqualTo("component_3"));
    Assert.That(mjcfFakeBodyA.GetAttribute("name"), Is.EqualTo("component_4"));
  }

  [Test]
  public void ActuatorsAreAddedToDedicatedTag() {
    _actuator.enabled = true;
    var mjcf = _scene.CreateScene(skipCompile:true);
    var mjcfForActuator = mjcf.SelectNodes("/mujoco/actuator/general")[0] as XmlElement;
    Assert.That(mjcfForActuator, Is.Not.Null);
  }

  [Test]
  public void SensorsAreAddedToDedicatedTag() {
    _sensor1.enabled = true;
    var mjcf = _scene.CreateScene(skipCompile:true);
    var mjcfForSensor = mjcf.SelectNodes("/mujoco/sensor/jointpos")[0] as XmlElement;
    Assert.That(mjcfForSensor, Is.Not.Null);
  }

  [Test]
  public void SensorsAreOrdered() {
    _sensor0.enabled = true;
    _sensor1.enabled = true;
    var mjcf = _scene.CreateScene(skipCompile:true);
    var sensor0Element =
        mjcf.SelectNodes("/mujoco/sensor")[0].ChildNodes[_sensor0.transform.GetSiblingIndex()] as XmlElement;
    Assert.That(sensor0Element.Name, Is.EqualTo("user"));
    var sensor1Element =
        mjcf.SelectNodes("/mujoco/sensor")[0].ChildNodes[_sensor1.transform.GetSiblingIndex()] as XmlElement;
    Assert.That(sensor1Element.Name, Is.EqualTo("jointpos"));
  }

  [Test]
  public unsafe void SceneRecreatedWithAddition() {
    _scene.CreateScene();
    _scene.Data->qvel[0] = 1;
    var nq = _scene.Model->nq;
    var tickedRotation = _body.transform.rotation;
    var body = new GameObject("body").AddComponent<MjBody>();
    var inertia = new GameObject("inertia").AddComponent<MjInertial>();
    inertia.transform.parent = body.transform;
    var joint = new GameObject("joint").AddComponent<MjHingeJoint>();
    joint.transform.parent = body.transform;
    _scene.RecreateScene();
    Assert.That(_scene.Model->nq, Is.EqualTo(nq+1));
    Assert.That(_scene.Data->qvel[joint.DofAddress], Is.EqualTo(0));
    Assert.That(_scene.Data->qvel[_joint.DofAddress], Is.EqualTo(1));
    UnityEngine.Object.DestroyImmediate(inertia.gameObject);
    UnityEngine.Object.DestroyImmediate(joint.gameObject);
    UnityEngine.Object.DestroyImmediate(body.gameObject);
  }

  [Test]
  public unsafe void SceneRecreatedAfterDeletion() {
    var body = new GameObject("body").AddComponent<MjBody>();
    var inertia = new GameObject("inertia").AddComponent<MjInertial>();
    inertia.transform.parent = body.transform;
    var joint = new GameObject("joint").AddComponent<MjHingeJoint>();
    joint.transform.parent = body.transform;
    _scene.CreateScene();
    _scene.Data->qvel[0] = 1;
    _scene.Data->qvel[1] = 2;
    var nq = _scene.Model->nq;
    UnityEngine.Object.DestroyImmediate(inertia.gameObject);
    UnityEngine.Object.DestroyImmediate(joint.gameObject);
    UnityEngine.Object.DestroyImmediate(body.gameObject);
    _scene.RecreateScene();
    Assert.That(_scene.Model->nq, Is.EqualTo(nq-1));
    Assert.That(_scene.Data->qvel[0], Is.EqualTo(2));
  }

#region Test setup.

  public class FakeMjBody : MjBaseBody {
    public bool StateSynced = false;

    public override MujocoLib.mjtObj ObjectType => MujocoLib.mjtObj.mjOBJ_BODY;

    protected override void OnParseMjcf(XmlElement mjcf) {}

    protected override XmlElement OnGenerateMjcf(XmlDocument doc) {
      // We'll create an actual element because the code will then pass it on to Mujoco scene
      // compiler, and we care that the compiler doesn't fail.
      var mjcf = doc.CreateElement("body");
      return mjcf;
    }

    public override unsafe void OnSyncState(MujocoLib.mjData_* data) {
      StateSynced = true;
    }
  }

  private MjScene _scene;
  private FakeMjBody _fakeBodyA;
  private FakeMjBody _fakeBodyB;
  private Transform _sensorGroup;
  private MjUserSensor _sensor0;
  private MjJointScalarSensor _sensor1;
  private MjBody _body;
  private MjInertial _inertia;
  private MjHingeJoint _joint;
  private MjActuator _actuator;
  private Vector4EqualityComparer _quaternionComparer;

  [SetUp]
  public void SetUp() {
    _quaternionComparer = new Vector4EqualityComparer(1e-5f);
    _scene = MjScene.Instance;
    _fakeBodyA = new GameObject("component").AddComponent<FakeMjBody>();
    _fakeBodyB = new GameObject("component").AddComponent<FakeMjBody>();
    _sensor0 = new GameObject("sensor").AddComponent<MjUserSensor>();
    _sensor0.Dimension = 1;
    _sensor1 = new GameObject("sensor").AddComponent<MjJointScalarSensor>();
    _sensorGroup = new GameObject("sensors").transform;
    _sensor0.transform.parent = _sensorGroup;
    _sensor1.transform.parent = _sensorGroup;
    _actuator = new GameObject("actuator").AddComponent<MjActuator>();
    // body, joint and inertia are always present so that the actuator and sensor are valid
    _body = new GameObject("body").AddComponent<MjBody>();
    _inertia = new GameObject("inertia").AddComponent<MjInertial>();
    _inertia.transform.parent = _body.transform;
    _joint = new GameObject("joint").AddComponent<MjHingeJoint>();
    _joint.transform.parent = _body.transform;
    _sensor1.Joint = _joint;
    _sensor1.SensorType = MjJointScalarSensor.AvailableSensors.JointPos;
    _actuator.Joint = _joint;
    _fakeBodyA.enabled = false;
    _fakeBodyB.enabled = false;
    _sensor1.enabled = false;
    _actuator.enabled = false;
  }

  [TearDown]
  public void TearDown() {
    UnityEngine.Object.DestroyImmediate(_fakeBodyA.gameObject);
    UnityEngine.Object.DestroyImmediate(_fakeBodyB.gameObject);
    UnityEngine.Object.DestroyImmediate(_sensor0.gameObject);
    UnityEngine.Object.DestroyImmediate(_sensor1.gameObject);
    UnityEngine.Object.DestroyImmediate(_actuator.gameObject);
    UnityEngine.Object.DestroyImmediate(_joint.gameObject);
    UnityEngine.Object.DestroyImmediate(_inertia.gameObject);
    UnityEngine.Object.DestroyImmediate(_body.gameObject);
    UnityEngine.Object.DestroyImmediate(_scene.gameObject);
  }

#endregion
}
}
