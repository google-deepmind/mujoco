using System;
using System.IO;
using System.Runtime.InteropServices;
using UnityEditor;
using UnityEditor.PackageManager;
using UnityEngine;

namespace Mujoco {
public class MujocoBinaryRetriever {

  [InitializeOnLoadMethod]
  static void SubscribeToEvent() {
    // This causes the method to be invoked after the Editor registers the new list of packages.
    Events.registeredPackages += RegisteredPackagesEventHandler;
  }

  static void RegisteredPackagesEventHandler(
      PackageRegistrationEventArgs packageRegistrationEventArgs) {
    var mujocoPath = packageRegistrationEventArgs.added[0].assetPath;
    if (RuntimeInformation.IsOSPlatform(OSPlatform.OSX)) {
      if (AssetDatabase.LoadMainAssetAtPath(mujocoPath + "/mujoco.dylib") == null) {
        File.Copy(
            "/Applications/MuJoCo.app/Contents/Frameworks" +
            "/MuJoCo.framework/Versions/Current/libmujoco.2.1.1.dylib",
            mujocoPath + "/mujoco.dylib");
        AssetDatabase.Refresh();
      }
    } else if (RuntimeInformation.IsOSPlatform(OSPlatform.Linux)) {
      if (AssetDatabase.LoadMainAssetAtPath(mujocoPath + "/libmujoco.so") == null) {
        File.Copy(
            Environment.GetFolderPath(Environment.SpecialFolder.UserProfile) +
            "/.mujoco/mujoco-2.1.1/lib/libmujoco_nogl.so.2.1.1",
            mujocoPath + "/libmujoco.so");
        AssetDatabase.Refresh();
      }
    } else {
      if (AssetDatabase.LoadMainAssetAtPath(mujocoPath + "/mujoco.dll") == null) {
        File.Copy(
            Environment.GetFolderPath(Environment.SpecialFolder.UserProfile) +
            "\\MuJoCo\\bin\\mujoco.dll",
            mujocoPath + "\\mujoco.dll");
        AssetDatabase.Refresh();
      }
    }
  }
}
}
