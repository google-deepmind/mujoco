mujoco_test(
  kinematics_test
)

target_sources(
  kinematics_test
  PRIVATE
  ${PROJECT_SOURCE_DIR}/plugin/plasticity/j2.cc
)

target_include_directories(
  kinematics_test
  PRIVATE
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/plugin/plasticity
)


mujoco_test(
  trial_state_test
)

target_sources(
  trial_state_test
  PRIVATE
  ${PROJECT_SOURCE_DIR}/plugin/plasticity/j2.cc
)

target_include_directories(
  trial_state_test
  PRIVATE
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/plugin/plasticity
)


mujoco_test(
  radial_return_test
)

target_sources(
  radial_return_test
  PRIVATE
  ${PROJECT_SOURCE_DIR}/plugin/plasticity/j2.cc
)

target_include_directories(
  radial_return_test
  PRIVATE
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/plugin/plasticity
)


mujoco_test(
  plugin_state_test
)

target_sources(
  plugin_state_test
  PRIVATE
  ${PROJECT_SOURCE_DIR}/plugin/plasticity/j2.cc
)

target_include_directories(
  plugin_state_test
  PRIVATE
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/plugin/plasticity
)

mujoco_test(
  plastic_force_test
)

target_sources(
  plastic_force_test
  PRIVATE
  ${PROJECT_SOURCE_DIR}/plugin/plasticity/j2.cc
)

target_include_directories(
  plastic_force_test
  PRIVATE
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/plugin/plasticity
)

mujoco_test(
  force_integration_test
)

target_sources(
  force_integration_test
  PRIVATE
  ${PROJECT_SOURCE_DIR}/plugin/plasticity/j2.cc
)

target_include_directories(
  force_integration_test
  PRIVATE
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/plugin/plasticity
)

mujoco_test(
  aluminum_single_tet_test
)

target_sources(
  aluminum_single_tet_test
  PRIVATE
  ${PROJECT_SOURCE_DIR}/plugin/plasticity/j2.cc
)

target_include_directories(
  aluminum_single_tet_test
  PRIVATE
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/plugin/plasticity
)
