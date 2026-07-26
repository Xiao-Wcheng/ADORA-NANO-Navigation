cmake_minimum_required(VERSION 3.10)
project(adora_nano_global_planner_node)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(yaml-cpp REQUIRED)

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

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${YAML_CPP_INCLUDE_DIR}
    ${DORA_INCLUDE_DIR}
    ${DORA_OPERATOR_DIR}
    /usr/local/include
)

# Source files for library
set(SOURCES
    src/map_loader.cpp
    src/astar.cpp
    src/nav2_simple_smoother.cpp
)

# Create library
add_library(adora_nano_path_planning_lib ${SOURCES})
target_link_libraries(adora_nano_path_planning_lib ${YAML_CPP_LIBRARIES})

enable_testing()
add_executable(boost_astar_test tests/boost_astar_test.cpp)
target_link_libraries(boost_astar_test adora_nano_path_planning_lib)
add_test(NAME boost_astar COMMAND boost_astar_test)
add_executable(nav2_simple_smoother_test tests/nav2_simple_smoother_test.cpp)
target_link_libraries(nav2_simple_smoother_test adora_nano_path_planning_lib)
add_test(NAME nav2_simple_smoother COMMAND nav2_simple_smoother_test)
add_executable(self_filter_test tests/self_filter_test.cpp)
add_test(NAME self_filter COMMAND self_filter_test)

# DORA node executable
add_executable(adora_nano_global_planner_node src/dora_planning_node.cpp)
target_link_libraries(adora_nano_global_planner_node
				adora_nano_path_planning_lib
				${DORA_LIB_PATH}
)
