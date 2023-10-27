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
using UnityEngine;

namespace Mujoco {

public static class ConvertMj2Rca {
  private static MjActuator[] _actuators;
  private static MjExclude[] _excludes;
  public static GameObject ConvertTree(MjBody mjRoot) {
    _excludes = GameObject.FindObjectsOfType<MjExclude>();
    _actuators = GameObject.FindObjectsOfType<MjActuator>();
    // does it have joints with its parent?
    bool jointFound = false;
    foreach (Transform mjChild in mjRoot.transform) {
      var joint = mjChild.gameObject.GetComponent<MjHingeJoint>();
      if (joint) {
        jointFound = true;
        break;
      }
    }

    var rcaRoot = new GameObject("rcaRoot");
    rcaRoot.transform.position = mjRoot.transform.position + Vector3.right;
    rcaRoot.transform.rotation = mjRoot.transform.rotation;
    var rca = rcaRoot.AddComponent<ArticulationBody>();
    rca.linearDamping = 0f;  // Removes dampings.
    rca.angularDamping = 0f;
    rca.jointFriction = 0f;
    rca.maxLinearVelocity = 1.8446743E+19f; // Removes velocity limits.
    rca.maxAngularVelocity = 1.8446743E+19f;

    var inertial = mjRoot.GetComponentInChildren<MjInertial>();
    if (inertial != null) {
      rca.mass = inertial.Mass;
    }
    if (jointFound) {
      rca.immovable = true;
      // the ArticulationBody component will be added when the joint is converted
      var newRcaRoot = new GameObject(mjRoot.gameObject.name);
      newRcaRoot.transform.parent = rcaRoot.transform;
      newRcaRoot.transform.position = rcaRoot.transform.position;
      newRcaRoot.transform.rotation = rcaRoot.transform.rotation;
      ConvertSubtree(mjRoot, newRcaRoot);
    } else {
      ConvertSubtree(mjRoot, rcaRoot);
    }
    return rcaRoot;
  }

  private static void ConvertSubtree(MjBody mjParent, GameObject originalRcParent) {
    Quaternion reverseXZ = Quaternion.Euler(0, 180, 0);
    // may be overriden later:
    GameObject rcParent = originalRcParent;
    ArticulationBody rcb = null;

    // first pass: convert geoms, sites, cameras, lights
    foreach (Transform mjChild in mjParent.transform) {
      // there's only one mj component in a gameObject:
      var mjComponent = mjChild.gameObject.GetComponent<MjComponent>();
      if (mjComponent) {
        if (!(mjComponent is MjBody)) {
          var newGO = new GameObject(mjComponent.gameObject.name);
          newGO.transform.parent = rcParent.transform;
          // set global position:
          newGO.transform.position = mjComponent.transform.position + Vector3.right;
          newGO.transform.rotation = mjComponent.transform.rotation;
          if (mjComponent is MjGeom) {
            MjGeom geom = mjComponent as MjGeom;
            GameObject primitive = null;
            if (geom.ShapeType == MjShapeComponent.ShapeTypes.Capsule) {
              primitive = GameObject.CreatePrimitive(PrimitiveType.Capsule);
              primitive.name = newGO.name;
              primitive.transform.parent = newGO.transform.parent;
              primitive.transform.position = newGO.transform.position;
              primitive.transform.rotation = newGO.transform.rotation;
              // MISSING: this results in a skewed capsule - replace with a procedural capsule
              // component.
              primitive.transform.localScale = new Vector3(
                  geom.Capsule.Radius * 2,
                  geom.Capsule.HalfHeight + geom.Capsule.Radius,
                  geom.Capsule.Radius * 2);
              GameObject.DestroyImmediate(newGO);
            } else if (geom.ShapeType == MjShapeComponent.ShapeTypes.Sphere) {
              primitive = GameObject.CreatePrimitive(PrimitiveType.Sphere);
              primitive.name = newGO.name;
              primitive.transform.parent = newGO.transform.parent;
              primitive.transform.position = newGO.transform.position;
              primitive.transform.rotation = newGO.transform.rotation;
              var r = 2 * geom.Sphere.Radius;
              primitive.transform.localScale = new Vector3(r, r, r);
              GameObject.DestroyImmediate(newGO);
            } else if (geom.ShapeType == MjShapeComponent.ShapeTypes.Cylinder) {
              primitive = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
              primitive.name = newGO.name;
              primitive.transform.parent = newGO.transform.parent;
              primitive.transform.position = newGO.transform.position;
              primitive.transform.rotation = newGO.transform.rotation;
              primitive.transform.localScale = new Vector3(geom.Cylinder.Radius,
                                                           geom.Cylinder.HalfHeight * 2,
                                                           geom.Cylinder.Radius);
              GameObject.DestroyImmediate(newGO);
            } else if (geom.ShapeType == MjShapeComponent.ShapeTypes.Mesh) {
              if (geom.Settings.Filtering.Contype != 0 ||
                  geom.Settings.Filtering.Conaffinity != 0) {
                var collider = newGO.AddComponent<MeshCollider>();
                collider.sharedMesh = geom.Mesh.Mesh;
              }
              var mf = newGO.AddComponent<MeshFilter>();
              mf.sharedMesh = geom.Mesh.Mesh;
              var mr = newGO.AddComponent<MeshRenderer>();
              primitive = newGO;  // so that the material can be set below.
            } else {  // cube or ellipsoid
              primitive = GameObject.CreatePrimitive(PrimitiveType.Cube);
              primitive.name = newGO.name;
              primitive.transform.parent = newGO.transform.parent;
              primitive.transform.position = newGO.transform.position;
              primitive.transform.rotation = newGO.transform.rotation;
              Vector3 scale = new Vector3(1, 1, 1);
              if (geom.ShapeType == MjShapeComponent.ShapeTypes.Ellipsoid) {
                scale = 2 * geom.Ellipsoid.Radiuses;
              } else if (geom.ShapeType == MjShapeComponent.ShapeTypes.Box) {
                scale = 2 * geom.Box.Extents;
              }
              primitive.transform.localScale = scale;
              GameObject.DestroyImmediate(newGO);
            }
            if (geom.Settings.Filtering.Contype == 0 &&
                geom.Settings.Filtering.Conaffinity == 0) {
              var collider = primitive.GetComponent<Collider>();
              if (collider) {
                GameObject.DestroyImmediate(collider);
              }
            }

            primitive.GetComponent<MeshRenderer>().sharedMaterial =
              geom.gameObject.GetComponent<MeshRenderer>().sharedMaterial;
          } else {
            var nameParts = mjComponent.GetType().ToString().Split('.'); // remove namespace
            var elementType = nameParts[nameParts.Length - 1].Substring(2); // remove "Mj..."
            newGO.name = mjComponent.gameObject.name + "_" + elementType;
          }
        }
      } else {  // if it's a camera, copy it over:
        var mjCamera = mjChild.GetComponent<Camera>();
        if (mjCamera != null) {
          var newGO = new GameObject(mjChild.name);
          newGO.transform.parent = rcParent.transform;
          // set global position:
          newGO.transform.position = mjChild.transform.position + Vector3.right;
          newGO.transform.rotation = mjChild.transform.rotation;
          var pxCamera = newGO.AddComponent<Camera>();
          pxCamera.fieldOfView = mjCamera.fieldOfView;
          pxCamera.nearClipPlane = mjCamera.nearClipPlane;
        }
      }
    }

    // second pass: look for joints
    int jointsFound = 0;
    foreach (Transform mjChild in mjParent.transform) {
      var joint = mjChild.GetComponent<MjHingeJoint>();
      if (joint) {
        if (jointsFound == 0) { // first joint, create the rcb
          rcb = rcParent.AddComponent<ArticulationBody>();
          var inertial = mjParent.GetComponentInChildren<MjInertial>();
          if (inertial != null) {
            rcb.mass = inertial.Mass;
          }
          rcb.anchorPosition = joint.transform.localPosition;
          // This reversal is critical as it gives us the rest of the swizzling for free:
          rcb.anchorRotation = joint.transform.localRotation * reverseXZ;
          rcParent.name += "_" + joint.gameObject.name;
        } else {
          // Store children, move them later:
          List<Transform> childrenToMove = new List<Transform>();
          foreach (Transform child in rcParent.transform) {
            childrenToMove.Add(child);
          }
          var newrRcParent = new GameObject(mjParent.gameObject.name + "_" + joint.gameObject.name);
          // subsequent joints create new bodies, leave anchor as 0.
          newrRcParent.transform.parent = rcParent.transform;
          newrRcParent.transform.position = joint.transform.position + Vector3.right;
          newrRcParent.transform.rotation = joint.transform.rotation * reverseXZ;
          var mass = rcb.mass;
          rcb.mass = 0.001f;  // the previous body becomes a placeholder / dummy body
          rcParent = newrRcParent;  // override
          // move children (except the new one) to the new link, none of which should be rcb:
          foreach (Transform childToMove in childrenToMove) {
            childToMove.SetParent(rcParent.transform);
          }
          rcb = rcParent.AddComponent<ArticulationBody>();  // override the variable rcb
          // this is needed because the default anchor rotation is not 0:
          rcb.anchorRotation = Quaternion.identity;
          rcb.mass = mass;
        }
        rcb.jointType = ArticulationJointType.RevoluteJoint;
        if (joint.RangeLower == 0 && joint.RangeUpper == 0) {
          rcb.twistLock = ArticulationDofLock.FreeMotion;
        } else {
          rcb.twistLock = ArticulationDofLock.LimitedMotion;
        }
        rcb.jointFriction = joint.Settings.Solver.FrictionLoss;
        var drive = new ArticulationDrive();
        drive.lowerLimit = joint.RangeLower;
        drive.upperLimit = joint.RangeUpper;
        drive.damping = joint.Settings.Spring.Damping;
        var actuatorFound = false;
        foreach (var actuator in _actuators) {
          if (actuator.Joint == joint) {
            // Assumes symmetry between upper and lower limits of the force range:
            drive.forceLimit = actuator.CommonParams.ForceRange[1];
            drive.stiffness = actuator.CommonParams.Gear[0] * actuator.CustomParams.GainPrm[0];
            if (drive.stiffness == 0f) {
              if (drive.forceLimit == 0f) {
                throw new Exception(
                  "How do I parse an actuator without force limit nor stiffness?");
              } else {
                drive.stiffness = 1000f;  // we rely on force limit.
              }
            } else {
              if (drive.forceLimit == 0f) {
                drive.forceLimit = 1e10f;  // we rely on stiffness.
              }
            }
            actuatorFound = true;
            break;
          }
          if (!actuatorFound) {
            // passive joint, shouldn't be driven.
            drive.stiffness = 0f;
            drive.forceLimit = 0f;
          }
        }
        // MISSING: this drive should be the target of agent actuation.
        rcb.xDrive = drive;
        // PhysX's default is positive damping and limited max velocity:
        rcb.maxLinearVelocity = 1.8446743E+19f;
        rcb.maxAngularVelocity = 1.8446743E+19f;
        rcb.linearDamping = 0.0f;
        rcb.angularDamping = 0.0f;

        jointsFound += 1;
      }
    }

    if (jointsFound > 1) {
      var grandparent = originalRcParent.transform.parent?.gameObject;
      if (grandparent) {
        // MISSING: When we add an intermediate body in PhysX, we run a real risk of introducing
        // spurious contacts between collider pairs that previously were ignored (as a
        // parent-child), since they are now two or more steps away in the hierarchy. You need to
        // add a component here that adds Physics.IgnoreCollision statements when launched (see also
        // below).
      }
    }
    // the gameObject with all the colliders should have just the name of the body without joints:
    rcParent.name = mjParent.gameObject.name;

    // add contact exclusions, but only if we created an articulation body:
    var rcParentParent = rcParent.transform.parent?.GetComponentInParent<ArticulationBody>();
    if (rcParentParent != null && rcParent.GetComponent<ArticulationBody>() != null) {
      foreach (var exclude in _excludes) {
        string other = null;
        if (exclude.Body1 == mjParent) {
          other = exclude.Body2.gameObject.name;
        } else if (exclude.Body2 == mjParent) {
          other = exclude.Body1.gameObject.name;
        }

        // The list of available ArticulationBodies changes as the recursion advances
        foreach (var rcOther in GameObject.FindObjectsOfType<ArticulationBody>()) {
          // don't add explicit exclusion to immediate parent (no children yet)
          if (rcParentParent != rcOther && rcOther.gameObject.name == other) {
            // MISSING: Physics.IgnoreCollision statements between all colliders of rcParent
            // and rcOther.  IgnoreCollision can only be applied in Running mode, so if you're
            // running this script at Editor mode (e.g., for generating a prefab from the resulting
            // PhysX model, you need to add a component that would add IgnoreCollision when
            // launched.
            break;
          }
        }
      }
    }

    // third pass - build the next layer of the tree
    foreach (Transform mjChild in mjParent.transform) {
      // there's only one mj component in a gameObject:
      var mjComponent = mjChild.gameObject.GetComponent<MjComponent>();
      if (mjComponent & mjComponent is MjBody) {
        var rcChild = new GameObject(mjChild.name);
        rcChild.transform.parent = rcParent.transform;
        // set global position:
        rcChild.transform.position = mjComponent.transform.position + Vector3.right;
        rcChild.transform.rotation = mjComponent.transform.rotation;
        // main recursive call:
        ConvertSubtree(mjComponent as MjBody, rcChild);
      }
    }
  }
}
}