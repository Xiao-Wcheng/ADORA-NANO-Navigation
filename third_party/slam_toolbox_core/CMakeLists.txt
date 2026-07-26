cmake_minimum_required(VERSION 3.16)
project(slam_toolbox_karto_core LANGUAGES CXX)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system serialization filesystem thread)
find_package(TBB REQUIRED)

add_library(slam_toolbox_karto_core SHARED
  src/Karto.cpp
  src/Mapper.cpp
)
target_compile_features(slam_toolbox_karto_core PUBLIC cxx_std_17)
target_include_directories(slam_toolbox_karto_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(slam_toolbox_karto_core PUBLIC
  Eigen3::Eigen
  Boost::system
  Boost::serialization
  Boost::filesystem
  Boost::thread
  TBB::tbb
)
target_compile_options(slam_toolbox_karto_core PRIVATE -Wall -Wextra)
