// Copyright 2026 DeepMind Technologies Limited
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

// Tests for compliant-tendon muscle actuators (engine_muscle.c).

#include "src/engine/engine_muscle.h"

#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::HasSubstr;
using ::testing::NotNull;
using MuscleTest = MujocoTest;

// a point mass hanging from a muscle; the anchor is at z = 0.3 and the insertion at z = qpos
std::string Hanging(const std::string& muscle, double mass = 20) {
  return R"(
  <mujoco>
    <option timestep="0.001"/>
    <worldbody>
      <site name="anchor" pos="0 0 0.3"/>
      <body>
        <joint name="slide" type="slide" axis="0 0 1"/>
        <site name="insertion"/>
        <geom type="sphere" size="0.01" mass=")" + std::to_string(mass) + R"("/>
      </body>
    </worldbody>
    <tendon>
      <spatial name="path">
        <site site="anchor"/>
        <site site="insertion"/>
      </spatial>
    </tendon>
    <actuator>
      )" + muscle + R"(
    </actuator>
  </mujoco>)";
}

// OpenSim's default curves: d is dynprm, g gainprm, b biasprm
constexpr mjtNum kDyn[mjNDYN] = {0.01, 0.04, 0.01, 0.1, 1.4, 0.25, 5, 0.15, 0.6, 0.9};
constexpr mjtNum kGain[mjNGAIN] = {1000, 0.1, 0.2, 10, 0, 0.4441, 0.73, 1.8123, 0.8616, 0};
constexpr mjtNum kBias[mjNBIAS] = {0,     0.7,          0.2, 2.0/0.7, 0.75,
                                   0.049, 1.375/0.049, 2.0/3, 0.5,    0};

// a Millard2012 curve
mjtNum Millard(int curve, mjtNum x, const mjtNum* dynprm, const mjtNum* gainprm,
               const mjtNum* biasprm, mjtNum* slope) {
  return mju_muscleCurve(mjDYN_MILLARD2012, curve, x, dynprm, gainprm, biasprm, slope);
}

// Geyer and Herr's defaults
constexpr mjtNum kGeyerDyn[mjNDYN] = {0.01, 0.04, 0, 0, 5};
constexpr mjtNum kGeyerGain[mjNGAIN] = {1000, 0.1, 0.2, 12, 0.56, 0.05};
constexpr mjtNum kGeyerBias[mjNBIAS] = {1, 0.56, 0.04, 0};

// a Geyer2010 curve at the defaults
mjtNum Geyer(int curve, mjtNum x, mjtNum* slope = nullptr) {
  return mju_muscleCurve(mjDYN_GEYER2010, curve, x, kGeyerDyn, kGeyerGain, kGeyerBias, slope);
}

// ---------------------------------- curves ------------------------------------------------------

// the curves against OpenSim's own SmoothSegmentedFunctionFactory and evaluator, sampled at the
// default parameters across and beyond each curve's domain
TEST_F(MuscleTest, CurvesMatchOpenSim) {
  std::ifstream in(GetTestDataFilePath("engine/testdata/millard2012_opensim.csv"));
  ASSERT_TRUE(in.good());
  std::string line;
  ASSERT_TRUE(std::getline(in, line));
  EXPECT_EQ(line, "curve,x,y,dydx");

  const std::map<std::string, int> curve = {{"afl", mjMUSCLECURVE_ACTIVE},
                                            {"pfl", mjMUSCLECURVE_PASSIVE},
                                            {"tfl", mjMUSCLECURVE_TENDON},
                                            {"fv", mjMUSCLECURVE_VELOCITY}};
  std::map<int, int> count;
  while (std::getline(in, line)) {
    std::stringstream ss(line);
    std::string name, x, y, dydx;
    std::getline(ss, name, ',');
    std::getline(ss, x, ',');
    std::getline(ss, y, ',');
    std::getline(ss, dydx, ',');
    int c = curve.at(name);
    mjtNum slope;
    mjtNum value = Millard(c, std::stod(x), kDyn, kGain, kBias, &slope);
    EXPECT_NEAR(value, std::stod(y), MjTol(1e-12, 1e-5)) << name << " at " << x;
    EXPECT_NEAR(slope, std::stod(dydx), MjTol(1e-9, 1e-3) * std::fmax(1, std::fabs(slope)))
        << name << " at " << x;
    count[c]++;
  }
  for (int c = 0; c < 4; c++) {
    EXPECT_GT(count[c], 200);
  }
}

// OpenSim's classic Hill model (compliant tendon, no damping) raises the active force-length floor
// to 0.1 and the force-velocity slopes at vmax to half the slopes near vmax
TEST_F(MuscleTest, UndampedModelRaisesCurveFloors) {
  mjtNum dyn[mjNDYN], bias[mjNBIAS];
  std::copy(kDyn, kDyn + mjNDYN, dyn);
  std::copy(kBias, kBias + mjNBIAS, bias);
  dyn[3] = 0.0009;  // below OpenSim's MIN_NONZERO_DAMPING_COEFFICIENT

  mjtNum slope;
  EXPECT_EQ(Millard(mjMUSCLECURVE_ACTIVE, 0.2, dyn, kGain, bias, &slope), mjtNum(0.1));
  Millard(mjMUSCLECURVE_VELOCITY, -1.5, dyn, kGain, bias, &slope);
  EXPECT_MJTNUM_EQ(slope, 0.5 * dyn[5]);
  Millard(mjMUSCLECURVE_VELOCITY, 1.5, dyn, kGain, bias, &slope);
  EXPECT_MJTNUM_EQ(slope, 0.5 * dyn[7]);

  // the damped model, and the rigid tendon without damping, keep the floors at 0
  EXPECT_EQ(Millard(mjMUSCLECURVE_ACTIVE, 0.2, kDyn, kGain, kBias, &slope), 0);
  bias[9] = 1;
  EXPECT_EQ(Millard(mjMUSCLECURVE_ACTIVE, 0.2, dyn, kGain, bias, &slope), 0);
}

// ---------------------------------- model --------------------------------------------------------

// the shortcut sets the documented parameter layout, with OpenSim's defaults
TEST_F(MuscleTest, ShortcutSetsLayout) {
  auto model = LoadModelFromString(Hanging(
      R"(<millard2012 tendon="path" force="1000" optlength="0.1" slacklength="0.2"/>)"));
  ASSERT_THAT(model, NotNull());
  EXPECT_EQ(model->actuator_dyntype[0], mjDYN_MILLARD2012);
  EXPECT_EQ(model->actuator_gaintype[0], mjGAIN_MILLARD2012);
  EXPECT_EQ(model->actuator_biastype[0], mjBIAS_MILLARD2012);
  EXPECT_EQ(model->actuator_actnum[0], 2);
  for (int i = 0; i < mjNDYN; i++) EXPECT_DOUBLE_EQ(model->actuator_dynprm[i], kDyn[i]) << i;
  for (int i = 0; i < mjNGAIN; i++) EXPECT_DOUBLE_EQ(model->actuator_gainprm[i], kGain[i]) << i;
  for (int i = 0; i < mjNBIAS; i++) EXPECT_DOUBLE_EQ(model->actuator_biasprm[i], kBias[i]) << i;
}

// a strain without its stiffness takes OpenSim's derived stiffness
TEST_F(MuscleTest, ShortcutDerivesStiffness) {
  auto model = LoadModelFromString(Hanging(
      R"(<millard2012 tendon="path" passivecurve="-0.03 0.5" tendoncurve="0.033"/>)"));
  ASSERT_THAT(model, NotNull());
  EXPECT_MJTNUM_EQ(model->actuator_biasprm[mjMILLARD_PFL_KISO], 2 / 0.53);
  EXPECT_MJTNUM_EQ(model->actuator_biasprm[mjMILLARD_TFL_KISO], 1.375 / 0.033);
}

TEST_F(MuscleTest, CompilerRejectsInvalidParameters) {
  struct Case {
    std::string muscle;
    std::string error;
  };
  const Case cases[] = {
      {R"(<millard2012 tendon="path" force="0"/>)", "force must be positive"},
      {R"(<millard2012 tendon="path" activecurve="0.8 0.73"/>)", "active force-length"},
      {R"(<millard2012 tendon="path" passivecurve="0 0.7 0.2 1"/>)", "passive force-length"},
      {R"(<millard2012 tendon="path" tendoncurve="0.049 10"/>)", "tendon force-length"},
      {R"(<millard2012 tendon="path" velocitycurve="0.9"/>)", "force-velocity"},
      {R"(<millard2012 tendon="path" fiberdamping="0" minactivation="0.001"/>)",
       "at least 0.01 without fiber damping"},
      {R"(<millard2012 tendon="path" pennation="1.5"/>)", "pennation"},
      {R"(<general tendon="path" dyntype="millard2012"/>)",
       "must match"},
      {R"(<millard2012 tendon="path" actrange="0 1" actlimited="true"/>)", ""},
  };
  for (const Case& c : cases) {
    char error[1024];
    auto model = LoadModelFromString(Hanging(c.muscle), error, sizeof(error));
    EXPECT_EQ(model, nullptr) << c.muscle;
    EXPECT_THAT(error, HasSubstr(c.error)) << c.muscle;
  }
}

// the shortcut survives a save and a reload, to the precision of the saved file
TEST_F(MuscleTest, SaveRoundTrip) {
  auto model = LoadModelFromString(Hanging(
      R"(<millard2012 tendon="path" force="525" pennation="0.3" fiberdamping="0.01"
                      rigidtendon="true" activecurve="0.25 0.77 1.9 0.75"
                      passivecurve="0.04 0.7 0.2 3.03"/>)"));
  ASSERT_THAT(model, NotNull());
  std::string saved = SaveAndReadXml(model.get());
  auto reloaded = LoadModelFromString(saved);
  ASSERT_THAT(reloaded, NotNull()) << saved;
  for (int i = 0; i < mjNDYN; i++) {
    EXPECT_NEAR(model->actuator_dynprm[i], reloaded->actuator_dynprm[i],
                MjTol(1e-5, 1e-5) * std::fabs(model->actuator_dynprm[i])) << i;
  }
  for (int i = 0; i < mjNGAIN; i++) {
    EXPECT_NEAR(model->actuator_gainprm[i], reloaded->actuator_gainprm[i],
                MjTol(1e-5, 1e-5) * std::fabs(model->actuator_gainprm[i])) << i;
  }
  for (int i = 0; i < mjNBIAS; i++) {
    EXPECT_NEAR(model->actuator_biasprm[i], reloaded->actuator_biasprm[i],
                MjTol(1e-5, 1e-5) * std::fabs(model->actuator_biasprm[i])) << i;
  }
}

// a scaled frame scales the force and the fiber and tendon lengths, leaving the normalized fiber
// state of the scaled pose unchanged
TEST_F(MuscleTest, FrameScale) {
  auto model = LoadModelFromString(Hanging(R"(<millard2012 tendon="path" pennation="0.2"/>)"));
  std::string scaled_xml = Hanging(R"(<millard2012 tendon="path" pennation="0.2"/>)");
  scaled_xml.replace(scaled_xml.find("<worldbody>"), 11, "<worldbody><frame scale=\"2\">");
  scaled_xml.replace(scaled_xml.find("</worldbody>"), 12, "</frame></worldbody>");
  auto scaled = LoadModelFromString(scaled_xml);
  ASSERT_THAT(model, NotNull());
  ASSERT_THAT(scaled, NotNull()) << scaled_xml;
  EXPECT_MJTNUM_EQ(scaled->actuator_gainprm[mjMILLARD_LOPT], 2 * 0.1);
  EXPECT_MJTNUM_EQ(scaled->actuator_gainprm[mjMILLARD_LSLACK], 2 * 0.2);

  auto data = MakeData(model);
  auto sdata = MakeData(scaled);
  data->qpos[0] = 0.3 - 0.31;
  sdata->qpos[0] = 2 * (0.3 - 0.31);
  data->act[1] = sdata->act[1] = 0.5;
  for (auto [m, d] : {std::pair{model.get(), data.get()}, std::pair{scaled.get(), sdata.get()}}) {
    mj_forward(m, d);
    mj_muscleEquilibrium(m, d);
    mj_forward(m, d);
  }
  EXPECT_NEAR(sdata->act[0], data->act[0], MjTol(1e-9, 1e-5));
  mjtNum ratio = scaled->actuator_gainprm[mjMILLARD_FMAX] / model->actuator_gainprm[mjMILLARD_FMAX];
  EXPECT_NEAR(sdata->actuator_force[0], ratio * data->actuator_force[0],
              MjTol(1e-8, 1e-5) * std::fabs(ratio * data->actuator_force[0]));
}

// ---------------------------------- dynamics -----------------------------------------------------

// a zeroed state is a fiber at its optimal length
TEST_F(MuscleTest, ResetFiberIsOptimal) {
  auto model = LoadModelFromString(Hanging(R"(<millard2012 tendon="path"/>)"));
  ASSERT_THAT(model, NotNull());
  auto data = MakeData(model);
  EXPECT_EQ(data->act[0], 0);
  data->qpos[0] = 0.3 - 0.3;  // path at l_opt + l_slack
  mj_forward(model.get(), data.get());
  EXPECT_LT(std::fabs(data->act_dot[0]), 1);
}

// isometric equilibria from OpenSim's computeEquilibrium, for two real muscles at full
// activation: BIClong (MoBL-ARMS) and glmax3_r (Rajagopal 2016, pennated, retuned curves)
TEST_F(MuscleTest, EquilibriumMatchesOpenSim) {
  struct Case {
    std::string muscle;
    double lmtu, lce;
  };
  const Case cases[] = {
      {R"(<millard2012 tendon="path" force="525.1" optlength="0.1157" slacklength="0.2723"
                       passivecurve="0 0.7 0.2 2.8571" tendoncurve="0.049 28.0612 0.6667"/>)",
       0.3851, 0.10016},
      {R"(<millard2012 tendon="path" force="947.754098360656" optlength="0.16699999962441678"
                       slacklength="0.103" pennation="0.38241613"
                       activecurve="0.25 0.77 1.9 0.75"
                       passivecurve="0.039777684490424015 0.7004987591743358 0.2
                                     3.026995924046766"/>)",
       0.1554, 0.08020},
  };
  for (const Case& c : cases) {
    auto model = LoadModelFromString(Hanging(c.muscle));
    ASSERT_THAT(model, NotNull()) << c.muscle;
    auto data = MakeData(model);
    data->qpos[0] = 0.3 - c.lmtu;
    data->act[1] = 1;
    data->ctrl[0] = 1;
    mj_forward(model.get(), data.get());
    mj_muscleEquilibrium(model.get(), data.get());
    mj_forward(model.get(), data.get());
    double lopt = model->actuator_gainprm[mjMILLARD_LOPT];
    EXPECT_NEAR(lopt * (1 + data->act[0]), c.lce, MjTol(1e-4, 1e-4)) << c.muscle;
    EXPECT_NEAR(data->act_dot[0], 0, MjTol(1e-6, 1e-2)) << c.muscle;
    EXPECT_LT(data->actuator_force[0], -1) << c.muscle;
  }
}

// the force is gain*act + bias at the solved fiber, which is the tendon force: a hanging mass
// comes to rest with the muscle carrying its weight
TEST_F(MuscleTest, StaticEquilibriumCarriesTheLoad) {
  auto model = LoadModelFromString(Hanging(
      R"(<millard2012 tendon="path" force="3000" optlength="0.15" slacklength="0.15"
                      pennation="0.15"/>)"));
  ASSERT_THAT(model, NotNull());
  auto data = MakeData(model);
  model->dof_damping[0] = 200;
  data->qpos[0] = 0.3 - 0.31;
  data->ctrl[0] = 0.5;
  for (int i = 0; i < 5000; i++) {
    mj_step(model.get(), data.get());
  }
  mjtNum weight = 20 * 9.81;
  EXPECT_NEAR(-data->actuator_force[0], weight, MjTol(1e-3, 1e-3) * weight);
  EXPECT_NEAR(data->qvel[0], 0, MjTol(1e-6, 1e-5));
}

TEST_F(MuscleTest, ForwardIsIdempotent) {
  auto model = LoadModelFromString(Hanging(R"(<millard2012 tendon="path"/>)"));
  ASSERT_THAT(model, NotNull());
  auto data = MakeData(model);
  data->ctrl[0] = 0.7;
  for (int i = 0; i < 50; i++) {
    mj_step(model.get(), data.get());
  }
  mj_forward(model.get(), data.get());
  mjtNum force = data->actuator_force[0], fiber = data->act_dot[0];
  mj_forward(model.get(), data.get());
  EXPECT_EQ(data->actuator_force[0], force);
  EXPECT_EQ(data->act_dot[0], fiber);
}

// with a rigid tendon OpenSim puts a fiber on a path shorter than the tendon at its minimum
// length, where it carries no force (MuscleFixedWidthPennationModel::calcFiberLength)
TEST_F(MuscleTest, RigidTendonShortPathCarriesNoForce) {
  auto model = LoadModelFromString(Hanging(
      R"(<millard2012 tendon="path" optlength="0.15" slacklength="0.25" pennation="0.3"
                      rigidtendon="true"/>)"));
  ASSERT_THAT(model, NotNull());
  auto data = MakeData(model);
  data->act[1] = 1;
  for (double lmtu : {0.24, 0.2, 0.15, 0.1, 0.01}) {
    data->qpos[0] = 0.3 - lmtu;
    mj_forward(model.get(), data.get());
    EXPECT_EQ(data->actuator_force[0], 0) << lmtu;
  }

  // a taut, lengthening rigid tendon pulls
  data->qpos[0] = 0.3 - 0.4;
  mj_forward(model.get(), data.get());
  EXPECT_LT(data->actuator_force[0], -100);
}

// the analytic derivatives against central differences of the force at a fixed state
TEST_F(MuscleTest, DerivativesMatchFiniteDifferences) {
  for (const char* muscle :
       {R"(<millard2012 tendon="path" pennation="0.3"/>)",
        R"(<millard2012 tendon="path" pennation="0.3" rigidtendon="true"/>)",
        R"(<geyer2010 tendon="path" fiberdamping="0.1"/>)",
        R"(<geyer2010 tendon="path" fiberdamping="0.1" rigidtendon="true"/>)"}) {
    auto model = LoadModelFromString(Hanging(muscle));
    ASSERT_THAT(model, NotNull());
    auto data = MakeData(model);
    data->qpos[0] = 0.3 - 0.31;
    data->qvel[0] = 0.05;
    data->act[0] = 0.02;
    data->act[1] = 0.6;
    data->ctrl[0] = 0.6;
    mj_forward(model.get(), data.get());
    mjtNum len = mj_muscleForceLen(model.get(), data.get(), 0);
    mjtNum vel = mj_muscleForceVel(model.get(), data.get(), 0);

    // length: dl/dq = -1
    const mjtNum h = MjEps(1e-7, 1e-4);
    mjtNum q = data->qpos[0], v = data->qvel[0];
    data->qpos[0] = q - h;
    mj_forward(model.get(), data.get());
    mjtNum f1 = data->actuator_force[0];
    data->qpos[0] = q + h;
    mj_forward(model.get(), data.get());
    mjtNum f2 = data->actuator_force[0];
    data->qpos[0] = q;
    EXPECT_NEAR(len, -(f2 - f1) / (2*h), MjTol(1e-5, 1e-2) * std::fabs(len)) << muscle;

    // velocity: dl/dt = -qvel
    data->qvel[0] = v - h;
    mj_forward(model.get(), data.get());
    f1 = data->actuator_force[0];
    data->qvel[0] = v + h;
    mj_forward(model.get(), data.get());
    f2 = data->actuator_force[0];
    EXPECT_NEAR(vel, -(f2 - f1) / (2*h), MjTol(1e-5, 1e-2) * std::fmax(1, std::fabs(vel)))
        << muscle;
  }
}

// ---------------------------------- Geyer2010 ----------------------------------------------------

// the curves at the points that define them (Geyer and Herr 2010, Appendix II), Song's eccentric
// branch in his two-step form, and the slopes against central differences
TEST_F(MuscleTest, GeyerCurves) {
  const mjtNum w = 0.56, K = 5, N = 1.5;
  EXPECT_EQ(Geyer(mjMUSCLECURVE_ACTIVE, 1), 1);
  EXPECT_NEAR(Geyer(mjMUSCLECURVE_ACTIVE, 1 - w), 0.05, MjTol(1e-15, 1e-6));
  EXPECT_NEAR(Geyer(mjMUSCLECURVE_ACTIVE, 1 + w), 0.05, MjTol(1e-15, 1e-6));
  EXPECT_EQ(Geyer(mjMUSCLECURVE_PASSIVE, 1), 0);
  EXPECT_NEAR(Geyer(mjMUSCLECURVE_PASSIVE, 1 + w), 1, MjTol(1e-15, 1e-6));
  EXPECT_EQ(Geyer(mjMUSCLECURVE_TENDON, 1), 0);
  EXPECT_NEAR(Geyer(mjMUSCLECURVE_TENDON, 1.04), 1, MjTol(1e-14, 1e-5));
  EXPECT_EQ(Geyer(mjMUSCLECURVE_BUFFER, 1 - w), 0);
  EXPECT_NEAR(Geyer(mjMUSCLECURVE_BUFFER, 1 - 1.5*w), 1, MjTol(1e-15, 1e-5));
  EXPECT_EQ(Geyer(mjMUSCLECURVE_VELOCITY, -1), 0);
  EXPECT_EQ(Geyer(mjMUSCLECURVE_VELOCITY, 0), 1);
  EXPECT_EQ(Geyer(mjMUSCLECURVE_VELOCITY, 1), N);
  for (mjtNum v : {0.1, 0.5, 0.9, 3.0}) {
    mjtNum t = (v - 1) / (7.56*K*v + 1);
    EXPECT_NEAR(Geyer(mjMUSCLECURVE_VELOCITY, v), N - t/(t - 1), MjTol(1e-15, 1e-6)) << v;
  }

  const mjtNum h = MjEps(1e-6, 1e-3);
  for (int curve = mjMUSCLECURVE_ACTIVE; curve <= mjMUSCLECURVE_BUFFER; curve++) {
    for (mjtNum x : {-0.7, 0.3, 0.8, 1.2, 1.7}) {
      mjtNum slope;
      Geyer(curve, x, &slope);
      mjtNum fd = (Geyer(curve, x + h) - Geyer(curve, x - h)) / (2*h);
      EXPECT_NEAR(slope, fd, MjTol(1e-6, 1e-2) * std::fmax(1, std::fabs(fd)))
          << curve << " at " << x;
    }
  }
}

// the shortcut sets the documented parameter layout, with Geyer and Herr's defaults; a rest length
// without its strain takes the width of the active curve
TEST_F(MuscleTest, GeyerShortcutSetsLayout) {
  auto model = LoadModelFromString(Hanging(R"(<geyer2010 tendon="path"/>)"));
  ASSERT_THAT(model, NotNull());
  EXPECT_EQ(model->actuator_dyntype[0], mjDYN_GEYER2010);
  EXPECT_EQ(model->actuator_gaintype[0], mjGAIN_GEYER2010);
  EXPECT_EQ(model->actuator_biastype[0], mjBIAS_GEYER2010);
  EXPECT_EQ(model->actuator_actnum[0], 2);
  EXPECT_EQ(model->actuator_actearly[0], 1);
  for (int i = 0; i < mjNDYN; i++) EXPECT_EQ(model->actuator_dynprm[i], kGeyerDyn[i]) << i;
  for (int i = 0; i < mjNGAIN; i++) EXPECT_EQ(model->actuator_gainprm[i], kGeyerGain[i]) << i;
  for (int i = 0; i < mjNBIAS; i++) EXPECT_EQ(model->actuator_biasprm[i], kGeyerBias[i]) << i;

  model = LoadModelFromString(Hanging(
      R"(<geyer2010 tendon="path" activecurve="0.9" passivecurve="1.2"/>)"));
  ASSERT_THAT(model, NotNull());
  EXPECT_EQ(model->actuator_biasprm[mjGEYER_PFL_REST], mjtNum(1.2));
  EXPECT_EQ(model->actuator_biasprm[mjGEYER_PFL_STRAIN], mjtNum(0.9));
}

TEST_F(MuscleTest, GeyerCompilerRejectsInvalidParameters) {
  struct Case {
    std::string muscle;
    std::string error;
  };
  const Case cases[] = {
      {R"(<geyer2010 tendon="path" slacklength="0"/>)", "tendon slack length"},
      {R"(<geyer2010 tendon="path" activecurve="0.56 1"/>)", "residual force"},
      {R"(<geyer2010 tendon="path" passivecurve="0"/>)", "parallel element"},
      {R"(<geyer2010 tendon="path" tendoncurve="0"/>)", "tendon reference strain"},
      {R"(<geyer2010 tendon="path" velocitycurve="0.1"/>)", "pole"},
      {R"(<geyer2010 tendon="path" fiberdamping="-1"/>)", "fiber damping"},
      {R"(<general tendon="path" dyntype="geyer2010" gaintype="millard2012"
                 biastype="millard2012"/>)", "must match"},
  };
  for (const Case& c : cases) {
    char error[1024];
    auto model = LoadModelFromString(Hanging(c.muscle), error, sizeof(error));
    EXPECT_EQ(model, nullptr) << c.muscle;
    EXPECT_THAT(error, HasSubstr(c.error)) << c.muscle;
  }
}

TEST_F(MuscleTest, GeyerSaveRoundTrip) {
  for (const char* muscle :
       {R"(<geyer2010 tendon="path" force="95.3" optlength="0.18" slacklength="0.065"
                       vmax="10.4" timeconst="0.02 0.05" tausmooth="0.1" fiberdamping="0.1"
                       activecurve="0.96 0.07" passivecurve="1.17 0.47" tendoncurve="0.033"
                       velocitycurve="3.33" rigidtendon="true"/>)",
        R"(<geyer2010 tendon="path" activecurve="0.7"/>)"}) {
    auto model = LoadModelFromString(Hanging(muscle));
    ASSERT_THAT(model, NotNull());
    std::string saved = SaveAndReadXml(model.get());
    auto reloaded = LoadModelFromString(saved);
    ASSERT_THAT(reloaded, NotNull()) << saved;
    for (int i = 0; i < mjNDYN; i++) {
      EXPECT_EQ(model->actuator_dynprm[i], reloaded->actuator_dynprm[i]) << i;
    }
    for (int i = 0; i < mjNGAIN; i++) {
      EXPECT_EQ(model->actuator_gainprm[i], reloaded->actuator_gainprm[i]) << i;
    }
    for (int i = 0; i < mjNBIAS; i++) {
      EXPECT_EQ(model->actuator_biasprm[i], reloaded->actuator_biasprm[i]) << i;
    }
  }
}

// a hanging mass comes to rest with the muscle carrying its weight, from the isometric
// equilibrium or from a fiber at its optimal length
TEST_F(MuscleTest, GeyerStaticEquilibriumCarriesTheLoad) {
  auto model = LoadModelFromString(Hanging(
      R"(<geyer2010 tendon="path" force="3000" optlength="0.15" slacklength="0.15"/>)"));
  ASSERT_THAT(model, NotNull());
  model->dof_damping[0] = 200;
  mjtNum weight = 20 * 9.81;
  for (bool equilibrium : {true, false}) {
    auto data = MakeData(model);
    data->qpos[0] = 0.3 - 0.31;
    data->ctrl[0] = 0.5;
    data->act[1] = 0.5;
    mj_forward(model.get(), data.get());
    if (equilibrium) {
      mj_muscleEquilibrium(model.get(), data.get());
      EXPECT_EQ(data->act_dot[0], 0);
      mj_forward(model.get(), data.get());
      EXPECT_NEAR(data->act_dot[0], 0, MjTol(1e-6, 1e-2));
    }
    for (int i = 0; i < 5000; i++) {
      mj_step(model.get(), data.get());
    }
    EXPECT_NEAR(-data->actuator_force[0], weight, MjTol(1e-3, 1e-3) * weight) << equilibrium;
    EXPECT_NEAR(data->qvel[0], 0, MjTol(1e-6, 1e-5)) << equilibrium;
  }
}

// a rigid tendon on a path too short for the fiber to pull compresses the buffer element, and the
// tendon, which only pulls, carries no force
TEST_F(MuscleTest, GeyerRigidTendonShortPathCarriesNoForce) {
  auto model = LoadModelFromString(Hanging(
      R"(<geyer2010 tendon="path" optlength="0.1" slacklength="0.2" rigidtendon="true"/>)"));
  ASSERT_THAT(model, NotNull());
  auto data = MakeData(model);
  data->act[1] = 0.01;
  for (double lmtu : {0.2 + 0.3*0.1, 0.2, 0.1}) {
    data->qpos[0] = 0.3 - lmtu;
    mj_forward(model.get(), data.get());
    EXPECT_EQ(data->actuator_force[0], 0) << lmtu;
  }
  data->act[1] = 1;
  data->qpos[0] = 0.3 - 0.31;
  mj_forward(model.get(), data.get());
  EXPECT_LT(data->actuator_force[0], -500);
}

}  // namespace
}  // namespace mujoco
