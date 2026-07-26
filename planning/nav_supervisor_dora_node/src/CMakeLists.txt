cmake_minimum_required(VERSION 3.10)
project(nav_supervisor_dora_node)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

find_package(Threads REQUIRED)

set(DORA_INCLUDE_DIR "")
set(DORA_OPERATOR_DIR "")
set(DORA_LIB_PATH "")

foreach(_dora_base
    "$ENV{HOME}/dora"
    "$ENV{HOME}/dora-main"
    "$ENV{HOME}/Public/dora"
)
    if(EXISTS "${_dora_base}/apis/c/node/node_api.h" AND NOT DORA_INCLUDE_DIR)
        set(DORA_INCLUDE_DIR "${_dora_base}/apis/c/node")
        set(DORA_OPERATOR_DIR "${_dora_base}/apis/c/operator")
    endif()
endforeach()

foreach(_dora_lib
    "$ENV{HOME}/dora/target/release/libdora_node_api_c.a"
    "$ENV{HOME}/dora-main/target/release/libdora_node_api_c.a"
    "$ENV{HOME}/Public/dora/target/release/libdora_node_api_c.a"
)
    if(EXISTS "${_dora_lib}" AND NOT DORA_LIB_PATH)
        set(DORA_LIB_PATH "${_dora_lib}")
    endif()
endforeach()

if(NOT DORA_INCLUDE_DIR)
    message(FATAL_ERROR "DORA C API headers not found")
endif()
if(NOT DORA_LIB_PATH)
    message(FATAL_ERROR "DORA C API library not found. Build dora-node-api-c first.")
endif()

include_directories(
  ${DORA_INCLUDE_DIR}
  ${DORA_OPERATOR_DIR}
)

add_executable(nav_supervisor_dora_node
  src/nav_supervisor_dora_node.cpp
  src/supervisor_goal.cpp
)

target_link_libraries(nav_supervisor_dora_node
  Threads::Threads
  ${DORA_LIB_PATH}
  m
  rt
  dl
  pthread
)

enable_testing()
add_executable(nav_supervisor_goal_test tests/nav_supervisor_goal_test.cpp src/supervisor_goal.cpp)
target_include_directories(nav_supervisor_goal_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
add_test(NAME nav_supervisor_goal COMMAND nav_supervisor_goal_test)
