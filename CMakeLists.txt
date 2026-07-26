cmake_minimum_required(VERSION 3.16)
project(adora_nano_dora_navigation NONE)

add_custom_target(build_all
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/scripts/build_all.sh"
  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  USES_TERMINAL)

enable_testing()
add_test(NAME standalone_paths
  COMMAND python3 "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_standalone_paths.py")
add_test(NAME standalone_entrypoints
  COMMAND python3 "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_entrypoints.py")
