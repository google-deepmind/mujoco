#include <mujoco/mjplugin.h>

#include "j2.h"

namespace mujoco::plugin::plasticity {

mjPLUGIN_LIB_INIT(plasticity) { J2::RegisterPlugin(); }

}  // namespace mujoco::plugin::plasticity