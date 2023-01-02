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

#if UNITY_EDITOR

using System;
using System.Collections;
using System.Collections.Generic;
using System.Xml;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;
using UnityEngine.TestTools.Utils;

namespace Mujoco {

[TestFixture]
public class MjSensorPlayTests {
  private MjBody _body;
  private MjSite _site;
  private MjGeom _geom;
  private MjHingeJoint _joint;
  private MjSiteQuaternionSensor _siteQuatSensor0;
  private MjSiteQuaternionSensor _siteQuatSensor;
  private MjSiteVectorSensor _siteVectorSensor;
  private MjGeomQuaternionSensor _geomQuatSensor;
  private MjGeomVectorSensor _geomVectorSensor;
  private MjBodyQuaternionSensor _bodyQuatSensor;
  private MjBodyVectorSensor _bodyVectorSensor;
  private MjScene _scene;

  [SetUp]
  public void SetUp() {
    _scene = MjScene.Instance;
    _body = new GameObject("body").AddComponent<MjBody>();
    _site = new GameObject("site").AddComponent<MjSite>();
    _geom = new GameObject("geom").AddComponent<MjGeom>();
    _siteQuatSensor0 = new GameObject("sq0").AddComponent<MjSiteQuaternionSensor>();
    _siteQuatSensor0.Site = _site;
    _siteQuatSensor = new GameObject("sq").AddComponent<MjSiteQuaternionSensor>();
    _siteQuatSensor.Site = _site;
    _siteVectorSensor = new GameObject("sv").AddComponent<MjSiteVectorSensor>();
    _siteVectorSensor.Site = _site;
    _siteVectorSensor.SensorType = MjSiteVectorSensor.AvailableSensors.FramePos;
    _geomQuatSensor = new GameObject("gq").AddComponent<MjGeomQuaternionSensor>();
    _geomQuatSensor.Geom = _geom;
    _geomVectorSensor = new GameObject("gv").AddComponent<MjGeomVectorSensor>();
    _geomVectorSensor.Geom = _geom;
    _geomVectorSensor.SensorType = MjGeomVectorSensor.AvailableSensors.FramePos;
    _bodyQuatSensor = new GameObject("bq").AddComponent<MjBodyQuaternionSensor>();
    _bodyQuatSensor.Body = _body;
    _bodyVectorSensor = new GameObject("bv").AddComponent<MjBodyVectorSensor>();
    _bodyVectorSensor.Body = _body;
    _bodyVectorSensor.SensorType = MjBodyVectorSensor.AvailableSensors.FramePos;
    _site.transform.position = new Vector3(1, 2, 3);
    _site.transform.rotation = new Quaternion(0, 1, 2, 3);
    _geom.transform.position = new Vector3(4, 5, 6);
    _geom.transform.rotation = new Quaternion(4, 5, 6, 7);
    _body.transform.position = new Vector3(7, 8, 9);
    _body.transform.rotation = new Quaternion(7, 8, 9, 9);
    _scene.CreateScene(); // compilation happens here
  }

  [TearDown]
  public void TearDown() {
    GameObject.DestroyImmediate(_site.gameObject);
    GameObject.DestroyImmediate(_body.gameObject);
    GameObject.DestroyImmediate(_geom.gameObject);
    GameObject.DestroyImmediate(_siteQuatSensor0.gameObject);
    GameObject.DestroyImmediate(_siteQuatSensor.gameObject);
    GameObject.DestroyImmediate(_siteVectorSensor.gameObject);
    GameObject.DestroyImmediate(_geomQuatSensor.gameObject);
    GameObject.DestroyImmediate(_geomVectorSensor.gameObject);
    GameObject.DestroyImmediate(_bodyQuatSensor.gameObject);
    GameObject.DestroyImmediate(_bodyVectorSensor.gameObject);
    GameObject.DestroyImmediate(_scene.gameObject);
    GameObject.DestroyImmediate(MjScene.Instance);
  }

  [UnityTest]
  public IEnumerator CheckSensorData() {
    yield return new WaitForFixedUpdate(); // updating the transform
    yield return new WaitForFixedUpdate(); // updating the transform
    var vecComparer = new Vector3EqualityComparer(1e-4f);
    Assert.That(
        _siteVectorSensor.SensorReading,
        Is.EqualTo(_site.transform.position).Using(vecComparer));
    Assert.That(
        _geomVectorSensor.SensorReading,
        Is.EqualTo(_geom.transform.position).Using(vecComparer));
    Assert.That(
        _bodyVectorSensor.SensorReading,
        Is.EqualTo(_body.transform.position).Using(vecComparer));

    Assert.That(
      Quaternion.Angle(_siteQuatSensor0.SensorReading, _site.transform.rotation.normalized),
      Is.EqualTo(0).Within(1e-3));
    Assert.That(
      Quaternion.Angle(_siteQuatSensor.SensorReading, _site.transform.rotation.normalized),
      Is.EqualTo(0).Within(1e-3));
    Assert.That(
      Quaternion.Angle(_geomQuatSensor.SensorReading, _geom.transform.rotation.normalized),
      Is.EqualTo(0).Within(1e-3));
    Assert.That(
      Quaternion.Angle(_bodyQuatSensor.SensorReading, _body.transform.rotation.normalized),
      Is.EqualTo(0).Within(1e-3));
  }
}
}

#endif
