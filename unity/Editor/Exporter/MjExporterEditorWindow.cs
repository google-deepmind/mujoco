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

using System.IO;
using System.Text;
using System.Xml;
using UnityEditor;
using UnityEngine;

namespace Mujoco
{

    // Mujoco scenes exporter window.
    public class MjExporterEditorWindow : EditorWindow
    {
        [MenuItem("Assets/Export MuJoCo Scene")]
        public static void Apply()
        {
            if (Application.isPlaying || MjScene.InstanceExists)
            {
                Debug.LogWarning("MJCF exporting only available outside Play mode, and with no MjScene in the hierarchy.");
                return;
            }

            string path = EditorUtility.SaveFilePanel("Save MJCF model", "", "model", "xml");
            if (string.IsNullOrEmpty(path)) return;

            // Due to singleton pattern, this creates a MjScene GameObject we need to remove at the end.
            var mjcf = MjScene.Instance.CreateScene(skipCompile: true);
            try
            {
                using (var stream = File.Open(path, FileMode.Create))
                {
                    using (var writer = new XmlTextWriter(stream, new UTF8Encoding(false)))
                    {
                        writer.Formatting = Formatting.Indented;
                        mjcf.WriteContentTo(writer);
                        Debug.Log($"MJCF saved to {path}");
                    }
                }
            }
            catch (IOException ex)
            {
                Debug.LogWarning("Failed to save Xml to a file: " + ex.ToString());
            }

            // Removing the added MjScene.
            DestroyImmediate(MjScene.Instance.gameObject);
        }
    }
}
