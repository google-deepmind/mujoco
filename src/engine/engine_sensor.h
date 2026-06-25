// Copyright 2021 DeepMind Technologies Limited
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

#ifndef MUJOCO_SRC_ENGINE_ENGINE_SENSOR_H_
#define MUJOCO_SRC_ENGINE_ENGINE_SENSOR_H_

#include <mujoco/mjdata.h>
#include <mujoco/mjexport.h>
#include <mujoco/mjmodel.h>

#ifdef __cplusplus
extern "C" {
#endif

//-------------------------------- sensors ---------------------------------------------------------

// compute value for one sensor, write to sensordata, apply cutoff
void mj_computeSensor(const mjModel* m, mjData* d, int i, mjtNum* sensordata);

// position-dependent sensors
MJAPI void mj_sensorPos(const mjModel* m, mjData* d);

// velocity-dependent sensors
MJAPI void mj_sensorVel(const mjModel* m, mjData* d);

// acceleration/force-dependent sensors
MJAPI void mj_sensorAcc(const mjModel* m, mjData* d);

// Gaussian log-likelihood of obs given current sensordata and per-sensor noise.
// obs has the same layout as d->sensordata (length nsensordata).
// Sensors with sensor_noise[i] <= 0 are skipped; they do not contribute.
// Suitable as a particle filter importance weight: w = exp(mj_sensorLogLik(m, d, obs)).
MJAPI mjtNum mj_sensorLogLik(const mjModel* m, const mjData* d, const mjtNum* obs);

// Particle filtering utilities ================================================

// Compute log-partition function for importance weights using log-sum-exp trick.
// Prevents underflow/overflow when computing log(sum(exp(log_weights[i]))).
// Input:  log_weights: array of length n containing log-importance weights
//         n: number of particles
// Output: log_sum_exp: log(sum(exp(log_weights[i])))
// Returns: 0 on success
MJAPI int mj_normalizeWeights(mjtNum* log_weights, int n, mjtNum* log_sum_exp);

// Compute effective sample size (ESS) from normalized log-weights.
// ESS = 1 / sum(w_i^2) where w_i = exp(log_weights[i] - log_sum_exp)
// Helps assess particle filter degeneracy: ESS < N/2 typically triggers resampling.
// Input:  log_weights: array of length n containing log-importance weights
//         n: number of particles
//         log_sum_exp: log-partition function (from mj_normalizeWeights, or compute internally if NULL)
// Output: ess: effective sample size in [0, n]
// Returns: 0 on success, 1 if n <= 0
MJAPI int mj_effectiveSampleSize(const mjtNum* log_weights, int n,
                                  const mjtNum* log_sum_exp, mjtNum* ess);

// Systematic resampling of particles based on importance weights.
// Implements low-variance systematic resampling: deterministic, numerically stable.
// Particles with high weights are replicated; low-weight particles are removed.
// Input:  log_weights: array of length n containing log-importance weights
//         n: number of particles
//         seed: random seed for tie-breaking (for stochastic resampling variants)
// Output: indices: array of length n containing resampled particle indices [0, n-1]
//         Each entry indicates which original particle to copy for this new particle.
// Returns: 0 on success, 1 if n <= 0 or weights are all -inf
MJAPI int mj_resampleParticles(const mjtNum* log_weights, int n, uint64_t seed,
                                int* indices);


//-------------------------------- energy ----------------------------------------------------------

// position-dependent energy (potential)
MJAPI void mj_energyPos(const mjModel* m, mjData* d);

// velocity-dependent energy (kinetic)
MJAPI void mj_energyVel(const mjModel* m, mjData* d);

#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_ENGINE_ENGINE_SENSOR_H_
