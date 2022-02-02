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
using System.Linq;
using UnityEngine;

namespace Mujoco {

// A set of helper methods for working with hierarchies of components.
public static class MjHierarchyTool {

  public static IEnumerable<MjComponent> LinearizeHierarchyBFS(Transform start) {
    var open = new Queue<Transform>();
    open.Enqueue(start);
    while (open.Count > 0) {
      var transform = open.Dequeue();

      var components = transform.GetComponents<MjComponent>();
      foreach (var component in components) {
        if (component.isActiveAndEnabled) {
          yield return component;
        }
      }

      foreach (Transform child in transform) {
        open.Enqueue(child);
      }
    }
  }

  // Recurses up the tree (excluding self) and returns the first instance of a MjComponent.
  public static MjComponent FindParentComponent(MonoBehaviour start) {
    return FindParentComponent<MjComponent>(start);
  }

  // This is not equivalent to Unity's GetComponentInParent as it excludes self from the search.
  public static T FindParentComponent<T>(MonoBehaviour start) where T : MonoBehaviour {
    if (start.transform.parent == null) {
      return null;
    }
    var loopTransform = start.transform.parent;
    T result = loopTransform.GetComponent<T>();
    while (result == null && loopTransform.parent != null) {
      loopTransform = loopTransform.parent;
      result = loopTransform.GetComponent<T>();
    }
    return result;
  }

  public static List<T> GetComponentsInImmediateChildren<T>(Transform parent)
      where T : MonoBehaviour {
    var components = new List<T>();
    foreach (Transform child in parent) {
      components.AddRange(child.gameObject.GetComponents<T>());
    }
    return components;
  }

  // Finds a single component that matches both the type and the name criteria.
  public static T FindComponentOfTypeAndName<T>(string name) where T : MjComponent {
    var components = UnityEngine.Object.FindObjectsOfType<T>().Where(
        component => component.gameObject.name == name);
    var numComponents = components.Count();
    if (components == null || numComponents == 0) {
      throw new ArgumentException($"No components named '{name}' were found.");
    }
    if (numComponents > 1) {
      throw new ArgumentException($"More than one component named '{name}' was found.");
    }
    return components.FirstOrDefault() as T;
  }
}
}
