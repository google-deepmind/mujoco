// Copyright 2025 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

import * as THREE from "three"
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js"
import loadMujoco from "../dist/mujoco.js"

declare function loadMujoco(): Promise<MainModule>;

let mujoco: any;

const modelXml = `
<mujoco model="Particle">
  <statistic extent="1.5" meansize=".05"/>

  <option timestep="0.005" jacobian="sparse"/>

  <visual>
    <rgba haze="0.15 0.25 0.35 1"/>
    <quality shadowsize="4096"/>
    <map stiffness="700" shadowscale="0.5" fogstart="10" fogend="15" zfar="40" haze="0.3"/>
  </visual>

  <worldbody>
    <light directional="true" diffuse=".4 .4 .4" specular="0.1 0.1 0.1" pos="0 0 5.0" dir="0 0 -1" castshadow="false"/>
    <light directional="true" diffuse=".6 .6 .6" specular="0.2 0.2 0.2" pos="0 0 4" dir="0 0 -1"/>

    <geom name="ground" type="plane" size="0 0 1" pos="0 0 0" quat="1 0 0 0" condim="1"/>

    <body mocap="true" pos="-.1 .05 0" zaxis=".5 0 1">
      <geom type="capsule" size=".1 .1" group="1" condim="1"/>
    </body>
  </worldbody>

  <option solver="CG" tolerance="1e-6" timestep=".01"/>

  <size memory="1G"/>

  <visual>
    <map stiffness="100"/>
  </visual>

  <default>
    <default class="wall">
      <geom type="plane" size=".5 .5 .05"/>
    </default>
  </default>

  <worldbody>
    <geom name="+x" class="wall" zaxis="1 0 0"  pos="-.5 0 -.25"/>
    <geom name="-x" class="wall" zaxis="-1 0 0" pos=".5 0 -.25"/>
    <geom name="+y" class="wall" zaxis="0 1 0"  pos="0 -.5 -.25"/>
    <geom name="-y" class="wall" zaxis="0 -1 0" pos="0 .5 -.25"/>
    <replicate count="10" offset=".07 0 0">
      <replicate count="10" offset="0 .07 0">
        <replicate count="10" offset="0 0 .07">
          <body pos="-.315 -.315 1">
            <joint type="slide" axis="1 0 0"/>
            <joint type="slide" axis="0 1 0"/>
            <joint type="slide" axis="0 0 1"/>
            <geom size=".025" rgba=".8 .2 .1 1" condim="1"/>
          </body>
        </replicate>
      </replicate>
    </replicate>
  </worldbody>
</mujoco>
    `;

// Backport of CapsuleGeometry class introduced in THREE.js r139
class CapsuleGeometry extends THREE.BufferGeometry {
  readonly parameters: {
    readonly radius: number,
    readonly length: number,
    readonly capSegments: number,
    readonly radialSegments:  number
  };

  constructor(radius = 1, length = 1, capSegments = 4, radialSegments = 8) {
    const path = new THREE.Path();
    path.absarc(0, -length / 2, radius, Math.PI * 1.5, 0, false);
    path.absarc(0, length / 2, radius, 0, Math.PI * 0.5, false);
    const latheGeometry =
        new THREE.LatheGeometry(path.getPoints(capSegments), radialSegments);

    super();
    this.setIndex(latheGeometry.getIndex());
    this.setAttribute('position', latheGeometry.getAttribute('position'));
    this.setAttribute('normal', latheGeometry.getAttribute('normal'));
    this.setAttribute('uv', latheGeometry.getAttribute('uv'));

    this.type = 'CapsuleGeometry';

    this.parameters = {
      radius,
      length,
      capSegments,
      radialSegments,
    };
  }
}

class App {
  // TODO(matijak): We can use better types here by doing the following:
  // https://emscripten.org/docs/porting/connecting_cpp_and_javascript/embind.html#typescript-definitions
  mjModel: any;
  mjData: any;
  mjvOption: any;
  mjvPerturb: any;
  mjvCamera: any;
  mjvScene: any;

  paused = false;
  frameId: number|null = null;
  maxGeoms: number = 2 ** 15;

  scene: THREE.Scene;
  renderer: THREE.WebGLRenderer;
  camera: THREE.PerspectiveCamera;
  controls: OrbitControls;
  meshes: THREE.Mesh[] = [];
  bufferGeometryCache = new Map<string, THREE.BufferGeometry>();

  constructor() {
    this.mjvPerturb = new mujoco.MjvPerturb();
    this.mjvOption = new mujoco.MjvOption();
    this.mjvCamera = new mujoco.MjvCamera();

    this.scene = new THREE.Scene();

    this.renderer = new THREE.WebGLRenderer();
    this.renderer.setSize(window.innerWidth, window.innerHeight);
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    document.body.appendChild(this.renderer.domElement);

    this.camera = new THREE.PerspectiveCamera(
        45, window.innerWidth / window.innerHeight, .1, 1000);
    this.camera.up.set(0, 0, 1);  // Mujoco uses z-up
    this.camera.position.set(-2, 0, 2);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
  }

  dispose() {
    // Release all C++ objects
    if (this.mjvScene) {
      this.mjvScene.delete();
    }
    if (this.mjvCamera) {
      this.mjvCamera.delete();
    }
    if (this.mjvPerturb) {
      this.mjvPerturb.delete();
    }
    if (this.mjvOption) {
      this.mjvOption.delete();
    }
    if (this.mjData) {
      this.mjData.delete();
    }
    if (this.mjModel) {
      this.mjModel.delete();
    }

    // Release all the THREE.js objects
    this.meshes.forEach((mesh) => {
      if (mesh.material) {
        if (Array.isArray(mesh.material)) {
          mesh.material.forEach(material => material.dispose());
        } else {
          mesh.material.dispose();
        }
      }
      if (mesh.geometry) {
        mesh.geometry.dispose();
      }
    });
    this.bufferGeometryCache.clear();

    if (this.controls) {
      this.controls.dispose();
    }

    if (this.renderer) {
      this.renderer.dispose();
    }

    // Stop the animation loop since we've disposed of all the data
    if (this.frameId) {
      cancelAnimationFrame(this.frameId);
      this.frameId = null;
    }
  }

  loadModel(xmlContent: string) {
    this.mjModel = mujoco.MjModel.from_xml_string(xmlContent);
    if (!app.mjModel) {
      throw new Error('Failed to load model');
    }
    this.mjData = new mujoco.MjData(this.mjModel);
    if (!this.mjData) {
      throw new Error('Failed to load data');
    }

    this.initScene();
  }

  pauseButton() {
    this.paused = !this.paused;
    const button = document.getElementById('pause-button');
    if (button) {
      button.textContent = this.paused ? 'Resume' : 'Pause';
    }
  }

  // TODO(matijak): Fix the bug where contact cylinders are wrong if the
  // simulation is reset while they are being visualized
  resetButton() {
    if (this.mjModel && this.mjData) {
      console.log('Resetting model and data');

      mujoco.mj_resetData(this.mjModel, this.mjData);
      mujoco.mj_forward(this.mjModel, this.mjData);

      this.clearScene();
      this.initScene();
    }
  }

  contactButton() {
    const index = mujoco.mjtVisFlag.mjVIS_CONTACTPOINT.value;
    const value = this.mjvOption.flags[index];
    this.mjvOption.flags[index] = !value;

    const button = document.getElementById('contact-button');
    if (button) {
      button.textContent =
          this.mjvOption.flags[index] ? 'Hide Contacts' : 'Show Contacts';
    }

    this.clearScene();
    this.initScene();
  }

  initScene() {
    this.mjvScene = new mujoco.MjvScene(this.mjModel, this.maxGeoms);

    const pointLight = new THREE.PointLight(0xffffff, .4);
    pointLight.position.set(0, 0, 2);
    pointLight.castShadow = true;
    pointLight.shadow.mapSize.set(2048, 2048);
    this.scene.add(pointLight);

    const ambientLight = new THREE.AmbientLight(0xffffff, .2);
    this.scene.add(ambientLight);

    const spotLight = new THREE.SpotLight(0xffffff, .2);
    spotLight.position.set(0, 0, 2);
    spotLight.target.position.set(0, 0, 0);
    spotLight.castShadow = true;
    spotLight.shadow.mapSize.set(2048, 2048);
    this.scene.add(spotLight);
    this.scene.add(spotLight.target);
  }

  clearScene() {
    // clear cached meshes
    this.meshes.forEach((mesh) => {
      if (mesh.material) {
        if (Array.isArray(mesh.material)) {
          mesh.material.forEach(material => material.dispose());
        } else {
          mesh.material.dispose();
        }
      }
      if (mesh.geometry) {
        mesh.geometry.dispose();
      }
    });

    this.bufferGeometryCache.clear();
    this.meshes.length = 0;
    while (this.scene.children.length > 0) {
      this.scene.remove(this.scene.children[0]);
    }
    this.mjvScene.delete();
  }

  getBufferGeometry(mjvGeom: any): [boolean, THREE.BufferGeometry] {
    if (!(mjvGeom instanceof mujoco.MjvGeom)) {
      throw new Error('mjvGeom is not an instance of mujoco.MjvGeom');
    }

    const geomType = Number(mjvGeom.type);
    const geomDataId = Number(mjvGeom.dataid);
    const size = Array.from(mjvGeom.size).map((x: any) => Number(x));

    // Lookup the geometry and return it if found
    const key = JSON.stringify([geomType, size, geomDataId]);
    const found = this.bufferGeometryCache.get(key);
    if (found) {
      return [false, found];
    }

    // Create geometry
    let geom: THREE.BufferGeometry;
    if (geomType === mujoco.mjtGeom.mjGEOM_PLANE.value) {
      geom = new THREE.PlaneGeometry(
          2 * (size[0] ? size[0] : 10000),
          2 * (size[1] ? size[1] : 10000));
      const uv = geom.getAttribute('uv');
      for (let i = 0; i < uv.count; ++i) {
        uv.setY(i, 1 - uv.getY(i));
      }
    } else if (geomType === mujoco.mjtGeom.mjGEOM_SPHERE.value) {
      geom = new THREE.SphereGeometry(size[0]);
    } else if (geomType === mujoco.mjtGeom.mjGEOM_CAPSULE.value) {
      geom = new CapsuleGeometry(size[0], 2 * size[2], 32, 16);
      geom.rotateX(0.5 * Math.PI);
    } else if (geomType === mujoco.mjtGeom.mjGEOM_BOX.value) {
      geom = new THREE.BoxGeometry(
          2 * size[0], 2 * size[1], 2 * size[2]);
    } else if (geomType === mujoco.mjtGeom.mjGEOM_CYLINDER.value) {
      geom = new THREE.CylinderGeometry(
          size[0], size[1], 2 * size[2], 32);
      geom.rotateX(0.5 * Math.PI);
    } else if (geomType === mujoco.mjtGeom.mjGEOM_ELLIPSOID.value) {
      geom = new THREE.SphereGeometry(1);
      geom.scale(size[0], size[1], size[2]);
    } else {
      console.log('Unsupported geom type: ', geomType);
      geom = new THREE.BufferGeometry();
    }

    this.bufferGeometryCache.set(key, geom);
    return [true, geom];
  }

  update() {
    if (!this.mjModel || !this.mjData) {
      return;
    }

    app.controls.update();

    // Simulate physics for 1/60 sec
    if (!app.paused) {
      let sim_start = Number(app.mjData.time);
      while (Number(app.mjData.time) - sim_start < 1. / 60.) {
        mujoco.mj_step(app.mjModel, app.mjData);
      }
    }

    // Update the mujoco scene
    mujoco.mjv_updateScene(
        this.mjModel, this.mjData, this.mjvOption, this.mjvPerturb,
        this.mjvCamera, mujoco.mjtCatBit.mjCAT_ALL.value, this.mjvScene);

    const geoms = this.mjvScene.geoms;
    const geomCount = Number(geoms.size());
    for (let i = 0; i < geomCount; i++) {
      const mjvGeom = geoms.get(i);

      let mesh: THREE.Mesh;
      const rgba = Array.from(mjvGeom.rgba).map((x: any) => Number(x));
      const mat = Array.from(mjvGeom.mat).map((x: any) => Number(x));
      const pos = Array.from(mjvGeom.pos).map((x: any) => Number(x));

      if (i < this.meshes.length) {
        mesh = this.meshes[i];
        if (mesh.material instanceof THREE.MeshPhongMaterial) {
          mesh.material.color.setRGB(rgba[0], rgba[1], rgba[2]);
          mesh.material.opacity = rgba[3];
          mesh.material.transparent = rgba[3] < 1.0;
        }
      } else {
        const [added, geom] = this.getBufferGeometry(mjvGeom);

        // Create material
        let material = new THREE.MeshPhongMaterial();
        material.color.setRGB(rgba[0], rgba[1], rgba[2]);
        material.opacity = rgba[3];
        material.transparent = rgba[3] < 1.0;

        // Create mesh
        mesh = new THREE.Mesh(geom, material);
        mesh.castShadow = true;
        mesh.receiveShadow = true;

        this.meshes.push(mesh);
        this.scene.add(mesh);
      }

      mesh.matrixAutoUpdate = false;
      const sz = 1;
      mesh.matrix.set(
          mat[0], mat[1], mat[2] * sz, pos[0],
          mat[3], mat[4], mat[5] * sz, pos[1],
          mat[6], mat[7], mat[8] * sz, pos[2],
          0, 0, 0, 1);
      mesh.matrixWorldNeedsUpdate = true;

      mjvGeom.delete();
    }

    geoms.delete();
  }

  render() {
    this.renderer.render(this.scene, this.camera);
  }

  run() {
    const animate = () => {
      try {
        this.update();

        this.render();
      } catch (error) {
        console.error('Simulation error:', error);
      }

      // Request next frame
      this.frameId = requestAnimationFrame(animate);
    };

    // Request first frame
    this.frameId = requestAnimationFrame(animate);
  }
}

function setupWindowEvents() {
  // Add an event listener to clean up when the page is unloaded
  // Tip: put "window.dispatchEvent(new Event('unload'))" in the console to test
  window.addEventListener('unload', () => {
    app.dispose();
  });

  window.addEventListener('keydown', (event) => {
    if (event.code === 'Backspace') {
      app.resetButton();
    }
  });
  window.addEventListener('keydown', (event) => {
    if (event.code === 'Space') {
      app.pauseButton();
    }
  });
  window.addEventListener('keydown', (event) => {
    if (event.key === 'c') {
      app.contactButton();
    }
  });
}

let app: App;

async function main() {
  try {
    mujoco = await loadMujoco();

    app = new App();

    setupWindowEvents();

    // Note: all elements will be destroyed with the page
    const pauseButtonElement = document.getElementById('pause-button');
    if (pauseButtonElement) {
      pauseButtonElement.onclick = () => app.pauseButton();
    }
    const resetButtonElement = document.getElementById('reset-button');
    if (resetButtonElement) {
      resetButtonElement.onclick = () => app.resetButton();
    }
    const contactButtonElement = document.getElementById('contact-button');
    if (contactButtonElement) {
      contactButtonElement.onclick = () => app.contactButton();
    }

    app.loadModel(modelXml);

    app.run();

  } catch (error) {
    console.error('Initialization error: ', error);
    app.dispose();
  }
}
main();
