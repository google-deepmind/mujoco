// Copyright 2022 DeepMind Technologies Limited
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

#include <algorithm>
#include <cstddef>
#include <sstream>
#include <optional>

#include <mujoco/mjplugin.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mujoco.h>
#include "cable.h"


namespace mujoco::plugin::elasticity {
namespace {


//create color scale palette 
void scalar2rgba(float rgba[4], mjtNum Scalar, mjtNum min, mjtNum max) {
        mjtNum tmp;
        tmp = (Scalar - min) / (max - min);
        if (tmp < 0.2) {
            rgba[0] = 0.90 - 0.70 * (tmp / 0.2);
            rgba[2] = 0.80 * (tmp / 0.2) + 0.05;
            rgba[1] = 0.0;
        }
        else if (tmp > 0.8) {
            rgba[0] = 0.65 * ((tmp - 0.2) / 0.8) + 0.20;
            rgba[2] = 0.2 * ((tmp - 0.8) / 0.2) + 0.75;
            rgba[1] = 0.95;
        }
        else {
            rgba[0] = 0.65 * ((tmp - 0.2) / 0.8) + 0.20;
            rgba[2] = 0.85;            
            rgba[1] = (tmp-0.2)/0.6 * 0.95;
        }
}     
       

// compute quaternion difference between two frames in joint coordinates
void QuatDiff(mjtNum* quat, const mjtNum body_quat[4],
              const mjtNum joint_quat[4], bool pullback) {
  if (pullback == 0) {
    // contribution in local coordinates
    mju_mulQuat(quat, body_quat, joint_quat);
  } else {
    // contribution pulled-back in local coordinates of the other body
    mjtNum invquat[4];
    mju_mulQuat(invquat, body_quat, joint_quat);
    mju_negQuat(quat, invquat);
  }
}

// compute local force given material properties, orientation,
// and reference curvature
//   inputs:
//     stiffness   - material parameters
//     quat        - orientation of the body in local coordinates
//     omega0      - initial curvature
//     xquat       - cartesian orientation of the body (optional)
//     scl         - scaling of the force
//   outputs:
//     qfrc        - local torque contribution
void LocalForce(mjtNum qfrc[3], const mjtNum stiffness[9],
    const mjtNum quat[4], mjtNum omega0[3], mjtNum userdata[3], mjtNum userdatamin[3], mjtNum userdatamax[3],
    const mjtNum xquat[4], mjtNum scl) {
    mjtNum omega[3], lfrc[3];
    mjtNum Yield = stiffness[7]; //material yield stress limits   

    // compute curvature
    mju_quat2Vel(omega, quat, scl);

    // change of curvature from reference configuration ==> acts as strain component
    mjtNum d_omega[] = {
        omega[0] - omega0[0],
        omega[1] - omega0[1],
        omega[2] - omega0[2],
    };

    mjtNum trq[] = {
       -stiffness[0] * d_omega[0] / stiffness[3],
       -stiffness[1] * d_omega[1] / stiffness[3],
       -stiffness[2] * d_omega[2] / stiffness[3],
    }; // torque

    //compute maximum |stress| on the surface for each bending axis and the torsion
    mjtNum stress[] = {
       (-trq[0] * stiffness[4]), // sigma_t = Mt * z / J      
       (-trq[1] * stiffness[5]), // sigma_y = My * w / G
       (-trq[2] * stiffness[6]), // sigma_z = Mz * h / G
    };


    for (int u = 0; u < 3; u++) {
        //stress element exceeds yield stress level ==> end of elastic domain
        if (abs(stress[u]) > Yield) {

            //scaling principle of plastic deformation is idealized and varies in reality with material types
            //strain hardening and transition effects are not considered

            //change of cuvature scaled by stress relative to yield      
            omega0[u] += d_omega[u] * (stress[u] - Yield) / (stress[u]);

            // stress exceeds elastic domain ==> reduce torque
            trq[u] = -(Yield + stress[u]) / 2.0 / stiffness[u + 4];

        }
    }
            
    //calculate scalar data for visualization and readout
    userdata[0] = mju_sqrt((pow(stress[0] - stress[1], 2) + pow(stress[1] - stress[2], 2) + pow(stress[2] - stress[0], 2)) / 2); //von Mieses principal stress
    userdata[1] = mju_sqrt(pow(omega0[0], 2) + pow(omega0[1], 2) + pow(omega0[2], 2));  //element reference curvature 
    userdata[2] = mju_sqrt(pow(d_omega[0], 2) + pow(d_omega[1], 2) + pow(d_omega[2], 2));  //momentary curvature change

    userdatamax[0] = (userdata[0] > userdatamax[0]) ? userdata[0] : userdatamax[0]; //store maximum stress
    userdatamin[0] = (userdata[0] < userdatamin[0]) ? userdata[0] : userdatamin[0]; //store minimum stress
    userdatamax[1] = (userdata[1] > userdatamax[1]) ? userdata[1] : userdatamax[1]; //store maximum ref curvature
    userdatamin[1] = (userdata[1] < userdatamin[1]) ? userdata[1] : userdatamin[1]; //store minimum ref curvature
    userdatamax[2] = (userdata[2] > userdatamax[2]) ? userdata[2] : userdatamax[2]; //store maximum ref curvature change
    userdatamin[2] = (userdata[2] < userdatamin[2]) ? userdata[2] : userdatamin[2]; //store minimum ref curvature change

  
  
  // rotate into global frame
  if (xquat) {
    mju_rotVecQuat(lfrc, trq, xquat);
  } else {
    mju_copy3(lfrc, trq);
  }


  // add to total qfrc
  mju_addToScl3(qfrc, lfrc, scl);
}

// reads numeric attributes
bool CheckAttr(const char* name, const mjModel* m, int instance) {
  char *end;
  std::string value = mj_getPluginConfig(m, instance, name);
  value.erase(std::remove_if(value.begin(), value.end(), isspace), value.end());
  strtod(value.c_str(), &end);
  return end == value.data() + value.size();
}

}  // namespace

// factory function
std::optional<Cable> Cable::Create(
  const mjModel* m, mjData* d, int instance) {
  if (CheckAttr("twist", m, instance) && CheckAttr("bend", m, instance) && CheckAttr("yield", m, instance) ) {
    return Cable(m, d, instance);
  } else {
    mju_warning("Invalid parameter specification in cable plugin");
    return std::nullopt;
  }
}

// plugin constructor
Cable::Cable(const mjModel* m, mjData* d, int instance) {
  // parameters were validated by the factor function
  std::string flat = mj_getPluginConfig(m, instance, "flat");
  mjtNum G = strtod(mj_getPluginConfig(m, instance, "twist"), nullptr);
  mjtNum E = strtod(mj_getPluginConfig(m, instance, "bend"), nullptr);
  mjtNum Yield = strtod(mj_getPluginConfig(m, instance, "yield"), nullptr);

  // count plugin bodies
  n = 0;
  for (int i = 1; i < m->nbody; i++) {
    if (m->body_plugin[i] == instance) {
      if (!n++) {
        i0 = i;
      }
    }
  }

  // allocate arrays
  prev.assign(n, 0);         // index of previous body
  next.assign(n, 0);         // index of next body
  omega0.assign(3*n, 0);     // reference curvature
  userdata.assign(3 * n, 0);     // stress, reference curvature, curvature change
  userdatamin.assign(3, 0);     // 
  userdatamax.assign(3, 1);     // 
  stiffness.assign(9*n, 0);  // material parameters
  

  // run forward kinematics to populate xquat (mjData not yet initialized)
  mju_zero(d->mocap_quat, 4*m->nmocap);
  mju_copy(d->qpos, m->qpos0, m->nq);
  mj_kinematics(m, d);

  // compute initial curvature
  for (int b = 0; b < n; b++) {
    int i = i0 + b;
    if (m->body_plugin[i] != instance) {
      mju_error("This body does not have the requested plugin instance");
    }
    bool first = (b == 0), last = (b == n-1);
    prev[b] = first ? 0 : -1;
    next[b] =  last ? 0 : +1;

    // compute omega0: curvature at equilibrium
    if (prev[b] && flat != "true") {
      int qadr = m->jnt_qposadr[m->body_jntadr[i]] + m->body_dofnum[i]-3;
      mju_subQuat(omega0.data()+3*b, m->body_quat+4*i, d->qpos+qadr);
    } else {
      mju_zero3(omega0.data()+3*b);
    }

    // compute physical parameters
    int geom_i = m->body_geomadr[i];
    mjtNum J = 0, Iy = 0, Iz = 0, L = 0, a_J = 0, a_Iy = 0, a_Iz = 0;
        L = mju_dist3(d->xpos + 3 * i, d->xpos + 3 * (i + prev[b])); // distance from previous body
        if (m->geom_type[geom_i] == mjGEOM_CYLINDER ||
        m->geom_type[geom_i] == mjGEOM_CAPSULE) {
      // https://en.wikipedia.org/wiki/Torsion_constant#Circle
      // https://en.wikipedia.org/wiki/List_of_second_moments_of_area
      mjtNum r= m->geom_size[3*geom_i+0]; //radius
      J = mjPI * pow(r, 4) / 2;
      Iy = Iz = mjPI * pow(r, 4) / 4.;
      a_J = r / J;
      a_Iy = a_Iz = r / Iy;

    } else if (m->geom_type[geom_i] == mjGEOM_BOX) {
      // https://en.wikipedia.org/wiki/Torsion_constant#Rectangle
      // https://en.wikipedia.org/wiki/List_of_second_moments_of_area
      mjtNum h = m->geom_size[3*geom_i+1]; //half size height !
      mjtNum w = m->geom_size[3*geom_i+2]; //half size width !
      mjtNum a = std::max(h, w);
      mjtNum b = std::min(h, w);
      J = a*pow(b, 3)*(16./3.-3.36*b/a*(1-pow(b, 4)/pow(a, 4)/12));
      Iy = pow(2 * w, 3) * 2 * h / 12.; 
      Iz = pow(2 * h, 3) * 2 * w / 12.; 
      a_J = sqrt(pow(h,2)+ pow(w,2)) / J;
      a_Iy = w / Iy;
      a_Iz = h / Iz;
    }
    
    stiffness[9*b+0] = J * G;  //torsional stiffness = J * G / curvature
    stiffness[9*b+1] = Iy * E; //bending stiffness horizontal = E * Iy / curvature
    stiffness[9*b+2] = Iz * E; //bending stiffness vertical = E * Iz / curvature
    stiffness[9*b+3] = prev[b] ? L : 0; //Element length 
    stiffness[9*b+4] = a_J;    //factor a/J for calculation of sigma_t(curvature) torsional stress from torque Mt
    stiffness[9*b+5] = a_Iy;   //factor a/Iy for calculation of sigma_b(curvature) bending stress from torque My
    stiffness[9*b+6] = a_Iz;   //factor a/Iz for calculation of sigma_b(curvature) bending stress from torque Mz
    stiffness[9*b+7] = Yield;
  }
  //point userdata for visualization of skin colors to data->userdata
  d->userdata = userdata.data();
}

void Cable::Compute(const mjModel* m, mjData* d, int instance) {
  for (int b = 0; b < n; b++)  {
    // index into body array
    int i = i0 + b;
    if (m->body_plugin[i] != instance) {
      mju_error(
        "This body is not associated with the requested plugin instance");
    }

    // if no stiffness, skip body
    if (!stiffness[b*9+0] && !stiffness[b*9+1] && !stiffness[b*9+2]) {
      continue;
    }

    // elastic forces
    mjtNum quat[4] = {0};
    mjtNum xfrc[3] = {0};

    // local orientation
    if (prev[b]) {
      int qadr = m->jnt_qposadr[m->body_jntadr[i]] + m->body_dofnum[i]-3;
      QuatDiff(quat, m->body_quat+4*i, d->qpos+qadr, 0);

      // contribution of orientation i-1 to xfrc i
      LocalForce(xfrc, stiffness.data()+9*b, quat, omega0.data() + 3 * b, userdata.data() + 3 * b, 
            userdatamin.data(), userdatamax.data(), d->xquat+4*(i+prev[b]), 1);      
      //scalar2rgba(&m->geom_rgba[b * 4], userdata[b * 3], userdatamin[0], userdatamax[0]); //dynamic legend range
      scalar2rgba(&m->geom_rgba[b * 4], userdata[b * 3], 0, stiffness[9 * b + 7]*1.0); //fixed legend range 0...yield + 20%

    }

    if (next[b]) {
      int bn = b + next[b];
      int in = i + next[b];

      // local orientation
      int qadr = m->jnt_qposadr[m->body_jntadr[in]] + m->body_dofnum[in]-3;
      QuatDiff(quat, m->body_quat+4*in, d->qpos+qadr, 1);

      // contribution of orientation i+1 to xfrc i
      LocalForce(xfrc, stiffness.data()+9*bn, quat, omega0.data()+3*bn, userdata.data() + 3 * bn, 
            userdatamin.data(), userdatamax.data(), d->xquat+4*i, -1);      
      //scalar2rgba(&m->geom_rgba[bn * 4], userdata[bn * 3], userdatamin[0], userdatamax[0]);
      scalar2rgba(&m->geom_rgba[bn * 4], userdata[bn * 3], 0, stiffness[9 * bn + 7]*1.0);

    }

    // convert from global coordinates and apply torque to com
    mj_applyFT(m, d, 0, xfrc, d->xpos+3*i, i, d->qfrc_passive);
  }
}



mjPLUGIN_DYNAMIC_LIBRARY_INIT {
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "mujoco.elasticity.cable";
  plugin.type |= mjPLUGIN_PASSIVE;

  const char* attributes[] = {"twist", "bend", "flat", "yield"};
  plugin.nattribute = sizeof(attributes) / sizeof(attributes[0]);
  plugin.attributes = attributes;
  plugin.nstate = +[](const mjModel* m, int instance) { return 0; };

  plugin.init = +[](const mjModel* m, mjData* d, int instance) {
    auto elasticity_or_null = Cable::Create(m, d, instance);
    if (!elasticity_or_null.has_value()) {
      return -1;
    }
    d->plugin_data[instance] = reinterpret_cast<uintptr_t>(
        new Cable(std::move(*elasticity_or_null)));
return 0;
  };
  plugin.destroy = +[](mjData* d, int instance) {
    delete reinterpret_cast<Cable*>(d->plugin_data[instance]);
    d->plugin_data[instance] = 0;
  };
  plugin.compute = +[](const mjModel* m, mjData* d, int instance, int type) {
    auto* elasticity = reinterpret_cast<Cable*>(d->plugin_data[instance]);
    elasticity->Compute(m, d, instance);
  };

  mjp_registerPlugin(&plugin);
}

}  // namespace mujoco::plugin::elasticity
