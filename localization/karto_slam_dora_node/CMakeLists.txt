cmake_minimum_required(VERSION 3.16)
project(karto_slam_dora_node LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(
  ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/slam_toolbox_core
  ${CMAKE_CURRENT_BINARY_DIR}/slam_toolbox_core
)

find_package(nlohmann_json REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(Threads REQUIRED)

set(DORA_INCLUDE_DIR "$ENV{HOME}/dora-main/apis/c/node")
set(DORA_LIB_PATH "$ENV{HOME}/dora-main/target/release/libdora_node_api_c.a")
if(NOT EXISTS "${DORA_INCLUDE_DIR}/node_api.h" OR NOT EXISTS "${DORA_LIB_PATH}")
  message(FATAL_ERROR "Dora C API not found under $ENV{HOME}/dora-main")
endif()

add_library(karto_dora_support
  src/archive.cpp
  src/ceres_pose_graph_solver.cpp
  src/config.cpp
  src/karto_mapper.cpp
  src/message_adapter.cpp
  src/odometry_buffer.cpp
  src/occupancy_grid.cpp
  src/pose_extrapolator.cpp
  src/runtime.cpp
  src/scan_deskewer.cpp
  src/scan_preparer.cpp
)
target_include_directories(karto_dora_support PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(karto_dora_support PUBLIC
  nlohmann_json::nlohmann_json
  slam_toolbox_karto_core
  Ceres::ceres
  OpenSSL::Crypto
)
target_compile_options(karto_dora_support PRIVATE -Wall -Wextra)

add_executable(karto_slam_dora_node src/main.cpp)
target_link_libraries(karto_slam_dora_node PRIVATE slam_toolbox_karto_core karto_dora_support
  ${DORA_LIB_PATH} Threads::Threads m rt dl)
target_include_directories(karto_slam_dora_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${DORA_INCLUDE_DIR})

add_executable(replay_dataset tools/replay_dataset.cpp)
target_link_libraries(replay_dataset PRIVATE karto_dora_support)
target_include_directories(replay_dataset PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

enable_testing()
add_test(
  NAME no_ros_dependency
  COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/tests/no_ros_dependency_test.sh
          $<TARGET_FILE:karto_slam_dora_node>
)
add_test(NAME replay_determinism
  COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/tests/replay_determinism_test.sh
          $<TARGET_FILE:replay_dataset>
          ${CMAKE_CURRENT_SOURCE_DIR}/testdata/synthetic_room.jsonl)

foreach(test_name message_adapter odometry_buffer scan_deskewer scan_preparer config pose_graph_solver karto_scan_match constraint_origin karto_graph_snapshot archive occupancy_grid runtime_mapping runtime_continue runtime_localization runtime_pose_stream rectangle_mapping localization_health pose_extrapolator)
  add_executable(${test_name}_test tests/${test_name}_test.cpp)
  target_include_directories(${test_name}_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(${test_name}_test PRIVATE karto_dora_support)
  add_test(NAME ${test_name} COMMAND ${test_name}_test)
endforeach()
