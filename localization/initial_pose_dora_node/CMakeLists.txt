cmake_minimum_required(VERSION 3.10)
project(initial_pose_dora_node)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)

set(DORA_INCLUDE_DIR "")
set(DORA_LIB_PATH "")
foreach(base "$ENV{HOME}/dora" "$ENV{HOME}/dora-main" "$ENV{HOME}/Public/dora")
  if(EXISTS "${base}/apis/c/node/node_api.h" AND NOT DORA_INCLUDE_DIR)
    set(DORA_INCLUDE_DIR "${base}/apis/c/node")
  endif()
endforeach()
foreach(lib
    "$ENV{HOME}/dora/target/release/libdora_node_api_c.a"
    "$ENV{HOME}/dora-main/target/release/libdora_node_api_c.a"
    "$ENV{HOME}/Public/dora/target/release/libdora_node_api_c.a")
  if(EXISTS "${lib}" AND NOT DORA_LIB_PATH)
    set(DORA_LIB_PATH "${lib}")
  endif()
endforeach()

if(NOT DORA_INCLUDE_DIR OR NOT DORA_LIB_PATH)
  message(FATAL_ERROR "Dora C API headers/library not found")
endif()

add_executable(initial_pose_dora_node src/initial_pose_dora_node.cpp)
target_include_directories(initial_pose_dora_node PRIVATE ${DORA_INCLUDE_DIR})
target_link_libraries(initial_pose_dora_node PRIVATE ${DORA_LIB_PATH} m rt dl pthread)
