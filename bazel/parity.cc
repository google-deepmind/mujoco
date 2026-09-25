#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <vector>

#include <mujoco/mujoco.h>

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "Usage: parity MODEL STEPS\n";
    return 2;
  }
  const int steps = std::atoi(argv[2]);
  if (steps <= 0) return 2;
  char error[1024] = {};
  mjModel* model = mj_loadXML(argv[1], nullptr, error, sizeof(error));
  if (!model) {
    std::cerr << error << '\n';
    return 1;
  }
  mjData* data = mj_makeData(model);
  mj_forward(model, data);
  const auto start = std::chrono::steady_clock::now();
  for (int step = 0; step < steps; ++step) {
    for (int i = 0; i < model->nu; ++i) {
      data->ctrl[i] = 0.1 * std::sin(0.01 * (step + i));
    }
    mj_step(model, data);
  }
  const double seconds = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start).count();
  std::vector<mjtNum> state(mj_stateSize(model, mjSTATE_INTEGRATION));
  mj_getState(model, data, state.data(), mjSTATE_INTEGRATION);
  std::cout << std::setprecision(17) << "{\"version\":" << mj_version()
            << ",\"nq\":" << model->nq << ",\"nv\":" << model->nv
            << ",\"contacts\":" << data->ncon << ",\"seconds\":" << seconds
            << ",\"state\":[";
  for (size_t i = 0; i < state.size(); ++i) {
    if (i) std::cout << ',';
    std::cout << state[i];
  }
  std::cout << "]}\n";
  mj_deleteData(data);
  mj_deleteModel(model);
}
