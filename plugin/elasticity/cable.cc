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
void LocalForce(mjtNum qfrc[3], mujoco::plugin::elasticity::Cable::stiffness_ Sel, mujoco::plugin::elasticity::Cable::stiffness_consts_ Sconst,
    const mjtNum quat[4], mjtNum omega0[3], int userdatatype, mjtNum*  userdata, const mjtNum xquat[4], mjtNum scl) {
    
    mjtNum omega[3], lfrc[3];
    
    // compute curvature
    mju_quat2Vel(omega, quat, scl);

    // change of curvature from reference configuration ==> acts as strain component
    mjtNum d_omega[] = {
        omega[0] - omega0[0],
        omega[1] - omega0[1],
        omega[2] - omega0[2]
    };
    
    mjtNum strain[] = {
        1 / (-Sel.L_Dyz / d_omega[0]-1),
        1 / (-Sel.L_Dy / d_omega[1]-1),        
        1 / (-Sel.L_Dz / d_omega[2]-1)
    };
    
    //compute purely elastic stress below Yield and exponential plasticity above yield.
    mjtNum stress[] = {
       abs(strain[0]) < Sconst.epsY_G ? Sconst.G / (Sel.L_Dyz / d_omega[0] - 1) : Sconst.sigY + (Sconst.sig_UtY) * (1 - mju_exp(-Sconst.k_eps_UtY_G * (strain[0] - Sconst.epsY_G))),
       abs(strain[1]) < Sconst.epsY_E ? Sconst.E / (Sel.L_Dy  / d_omega[1] - 1) : Sconst.sigY + (Sconst.sig_UtY) * (1 - mju_exp(-Sconst.k_eps_UtY_E * (strain[1] - Sconst.epsY_E))),
       abs(strain[2]) < Sconst.epsY_E ? Sconst.E / (Sel.L_Dz  / d_omega[2] - 1) : Sconst.sigY + (Sconst.sig_UtY) * (1 - mju_exp(-Sconst.k_eps_UtY_E * (strain[2] - Sconst.epsY_E)))
    };
    
    //// torque from calculated beam stress
    //mjtNum tmp[] = {
    //   stress[0] * Sel.J_Dyz,
    //   stress[1] * Sel.Iy_Dy,
    //   stress[2] * Sel.Iz_Dz       
    //};                
      
    // permanent change of curvature    
    omega0[0] += abs(strain[0]) < Sconst.epsY_G ? 0 : (-Sel.L_Dyz * (strain[0] / (1 + strain[0])));
    omega0[1] += abs(strain[1]) < Sconst.epsY_E ? 0 : (-Sel.L_Dy *  (strain[1] / (1 + strain[1])));
    omega0[2] += abs(strain[2]) < Sconst.epsY_E ? 0 : (-Sel.L_Dz *  (strain[2] / (1 + strain[2])));
    
    //viscous damping proprtional to volume ~ Dxx, speed ~ d_omega
    mjtNum dampingfactor[] = {
        1/ (1 + Sel.Dyz * abs(d_omega[0]) * Sconst.Df),
        1/ (1 + Sel.Dy * abs(d_omega[1]) * Sconst.Df),
        1/ (1 + Sel.Dz * abs(d_omega[2]) * Sconst.Df),
    };

    // torque from new curvature
    mjtNum trq[] = {
        -Sconst.G * Sel.J_Dyz / Sel.L_Dyz * (omega[0] - omega0[0]) * dampingfactor[0],
       -Sconst.E * Sel.Iy_Dy / Sel.L_Dy * (omega[1] - omega0[1]) * dampingfactor[1],
       -Sconst.E * Sel.Iz_Dz / Sel.L_Dz * (omega[2] - omega0[2]) * dampingfactor[2]
    };
        
    //calculate scalar data for visualization and readout
    if (userdatatype & 1) {
        *userdata = mju_sqrt((pow(stress[0] - stress[1], 2) + pow(stress[1] - stress[2], 2) + pow(stress[2] - stress[0], 2)) / 2); //von Mieses principal stress
    }
    else if (userdatatype & 2) {
        *userdata = mju_sqrt(pow(omega0[0], 2) + pow(omega0[1], 2) + pow(omega0[2], 2));  //element reference curvature 
    }
    else if (userdatatype & 4) {
        *userdata = mju_sqrt(pow(d_omega[0], 2) + pow(d_omega[1], 2) + pow(d_omega[2], 2));  //momentary curvature change
    }
      
          
  
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
  if (CheckAttr("twist", m, instance) && CheckAttr("bend", m, instance)) {
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
  mjtNum YieldStress = strtod(mj_getPluginConfig(m, instance, "yieldstress"), nullptr); 
  mjtNum UtStress = strtod(mj_getPluginConfig(m, instance, "ultimatestress"), nullptr); 
  mjtNum UtStrain = strtod(mj_getPluginConfig(m, instance, "ultimatestrain"), nullptr);
  mjtNum YieldStrain_G = 1, YieldStrain_E = 1, dYield_UtY = 0, k_deltaStrain_G = 0, k_deltaStrain_E = 0;
  
  if (YieldStress) {
      YieldStrain_G = YieldStress / G;
      YieldStrain_E = YieldStress / E;      

  if (!UtStress || !UtStrain) {
      //switch from strain hardening to constant stress model for plastic domain
      UtStress = YieldStress;
      UtStrain = 1.0;
      mju_warning("ultimateStress or ultimateStrain paramters not set: perfect plasticity is assumed (max Stress = Yield Stress)");
  }
  else {
      dYield_UtY = UtStress - YieldStress; //strain hardening stress delta
      k_deltaStrain_G = 5 / (UtStrain - YieldStrain_G); //exponential constant for torsional strain hardening
      k_deltaStrain_E = 5 / (UtStrain - YieldStrain_E); //exponential constant for bending strain hardening
      
     //check if paramters are reasonable. Yieldstrain must not be larger than ultimate strain
      if (YieldStress / (E > G ? G : E) > UtStrain || UtStress < YieldStress) {
          mju_error("Error: elasto-plastic material parameters are not reasonable !\nPlease make sure, that Yield Stress < ultimate Stress and Yield Strain (=Yield Stress / Stiffness)  < ultimate Strain.");
      }
  }
  }
  else {
      mju_warning("YieldStress is notset: perfect elasticity is assumed !");
      YieldStress = 1e20;
      UtStress = YieldStress;
      UtStrain = 1.0;
  }

  Sconst.E = E; //youngs modulus
  Sconst.G = G; //torsion modulus
  Sconst.sig_UtY = dYield_UtY;
  Sconst.k_eps_UtY_E = k_deltaStrain_G;
  Sconst.k_eps_UtY_G = k_deltaStrain_E;
  Sconst.sigY = YieldStress;
  Sconst.epsY_G = YieldStrain_G;
  Sconst.epsY_E = YieldStrain_E;
  Sconst.Df = 10000;

  //exponential model interpolation, reference: https://monarch.qucosa.de/api/qucosa%3A19721/attachment/ATT-0/ Page 10-19
  
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
  userdatamin = 0;              //limits for scalar coloring
  userdatamax = YieldStress;  
  Sel.resize(n);  // elementwise depending material parameters
  

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

    // compute physical parameters based on exponential elasto-plastic model
    int geom_i = m->body_geomadr[i];
    mjtNum J = 0, Iy = 0, Iz = 0, L = 0;      
    
    //Dyz,Dy, Dz are the distances of the outer fabrics to the center axis of the beam
    mjtNum L_Dyz = 0, L_Dy = 0, L_Dz = 0, J_Dyz = 0, Iy_Dy = 0, Iz_Dz = 0, Dyz = 0, Dy = 0, Dz = 0;

      L = mju_dist3(d->xpos + 3 * i, d->xpos + 3 * (i + prev[b])); // distance from previous body
      if (m->geom_type[geom_i] == mjGEOM_CYLINDER ||
         m->geom_type[geom_i] == mjGEOM_CAPSULE) {
      // https://en.wikipedia.org/wiki/Torsion_constant#Circle
      // https://en.wikipedia.org/wiki/List_of_second_moments_of_area
      Dyz = Dy = Dz = m->geom_size[3*geom_i+0]; //radius
      J = mjPI * pow(Dyz, 4) / 2;
      Iy = Iz = mjPI * pow(Dyz, 4) / 4.;
      J_Dyz = J / Dyz;
      Iy_Dy = Iz_Dz = Iy / Dyz;
      L_Dyz = L_Dy = L_Dz = L / Dyz;
      

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
      Dyz = sqrt(pow(h, 2) + pow(w, 2));
      Dy = w;
      Dz = h;
      J_Dyz = J / Dyz; //diagonal distance of edge
      Iy_Dy = Iy / Dy;
      Iz_Dz = Iz / Dz;
      L_Dyz = L / Dyz;
      L_Dy = L / Dy;
      L_Dz = L / Dz;
      
    }
    // precalculated elementwise constants for faster calculation of exponential model
    Sel[b].dOmega_Yield[0] = -L_Dyz / (1 + 1 / YieldStrain_G);
    Sel[b].dOmega_Yield[1] = -L_Dy / (1 + 1 / YieldStrain_E);
    Sel[b].dOmega_Yield[2] = -L_Dz / (1 + 1 / YieldStrain_E);
    Sel[b].J_Dyz = J_Dyz; 
    Sel[b].Iy_Dy = Iy_Dy; 
    Sel[b].Iz_Dz = Iz_Dz; 
    Sel[b].L_Dyz = L_Dyz; 
    Sel[b].L_Dy = L_Dy;
    Sel[b].L_Dz = L_Dz;
    Sel[b].Dyz = Dyz;
    Sel[b].Dy = Dy;
    Sel[b].Dz = Dz;
        
    //reference: https://www.continuummechanics.org/beambending.html
      }
  //point userdata for visualization of skin colors to data->userdata
  //d->userdata = userdata.data();
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
    if (!Sel[b].J_Dyz && !Sel[b].Iy_Dy && !Sel[b].Iz_Dz && !Sel[b].L_Dyz && !Sel[b].L_Dy && !Sel[b].L_Dz) {
      continue;
    }

    // elastic forces
    mjtNum quat[4] = {0};
    mjtNum xfrc[3] = {0};
    mjtNum userdata = 0;
    // local orientation
    if (prev[b]) {
      int qadr = m->jnt_qposadr[m->body_jntadr[i]] + m->body_dofnum[i]-3;
      QuatDiff(quat, m->body_quat+4*i, d->qpos+qadr, 0);   
      // contribution of orientation i-1 to xfrc i      
     
     LocalForce(xfrc, Sel[b], Sconst, quat, omega0.data() + 3 * b, 1, &userdata, d->xquat + 4 * (i + prev[b]), 1);               
          
    }

    if (next[b]) {
      int bn = b + next[b];
      int in = i + next[b];

      // local orientation
      int qadr = m->jnt_qposadr[m->body_jntadr[in]] + m->body_dofnum[in]-3;
      QuatDiff(quat, m->body_quat+4*in, d->qpos+qadr, 1);
      // contribution of orientation i+1 to xfrc i
      LocalForce(xfrc, Sel[bn], Sconst, quat, omega0.data()+3*bn, 1, &userdata, d->xquat+4*i, -1);      
     
    }

    // set geometry color based on userdata
    scalar2rgba(&m->geom_rgba[i * 4], userdata, userdatamin, userdatamax); //fixed legend range 0...ultimate strain + 10%

    // convert from global coordinates and apply torque to com
    mj_applyFT(m, d, 0, xfrc, d->xpos+3*i, i, d->qfrc_passive);
  }
}
mjPLUGIN_DYNAMIC_LIBRARY_INIT {
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "mujoco.elasticity.cable";
  plugin.type |= mjPLUGIN_PASSIVE;

  const char* attributes[] = {"twist", "bend", "flat", "yieldstress", "ultimatestress", "ultimatestrain"};
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
