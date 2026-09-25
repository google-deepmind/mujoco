set(MUJOCO_FILAMENT_LIBRARIES
    filament backend filabridge filaflat utils smol-v
    image ktxreader basis_transcoder zstd)
if(TARGET bluegl)
  list(APPEND MUJOCO_FILAMENT_LIBRARIES bluegl)
endif()
if(TARGET bluevk)
  list(APPEND MUJOCO_FILAMENT_LIBRARIES bluevk)
endif()
install(TARGETS ${MUJOCO_FILAMENT_LIBRARIES}
    ARCHIVE DESTINATION lib COMPONENT mujoco)
if(NOT WASM)
  install(TARGETS matc resgen cmgen RUNTIME DESTINATION bin COMPONENT mujoco)
endif()
foreach(header_dir
    filament/include filament/backend/include libs/math/include
    libs/utils/include libs/filabridge/include libs/filaflat/include
    libs/image/include libs/ktxreader/include third_party/robin-map/include)
  install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/${header_dir}/"
      DESTINATION include COMPONENT mujoco)
endforeach()
