cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)
#set(ROS_BUILD_TYPE Debug)

rosbuild_init()

FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
message("OPENMP FOUND")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

file(GLOB_RECURSE ITOMP_HEADER_FILES RELATIVE ${PROJECT_SOURCE_DIR} *.h)

rosbuild_add_library(itomp_ca
src/planner/itomp_planner_node.cpp
src/model/itomp_robot_model.cpp
src/model/itomp_planning_group.cpp
src/model/treefksolverjointposaxis.cpp
src/model/treefksolverjointposaxis_partial.cpp
src/trajectory/itomp_cio_trajectory.cpp
src/cost/smoothness_cost.cpp
src/cost/trajectory_cost_accumulator.cpp
src/cost/trajectory_cost.cpp
src/contact/contact_point.cpp
src/contact/ground_manager.cpp
src/contact/contact_force_solver.cpp
src/visualization/visualization_manager.cpp
src/util/min_jerk_trajectory.cpp
src/util/planning_parameters.cpp
src/util/point_to_triangle_projection.cpp
src/optimization/itomp_optimizer.cpp
src/optimization/evaluation_manager.cpp
src/optimization/evaluation_data.cpp
src/optimization/improvement_manager.cpp
src/optimization/improvement_manager_chomp.cpp
src/optimization/rollout.cpp
src/precomputation/precomputation.cpp
${ITOMP_HEADER_FILES}
)
set(LIBRARY_INPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(LIBRARY_NAME itomp_ca_planner_plugin)
rosbuild_add_library(${LIBRARY_NAME} src/itomp_plugin.cpp src/itomp_planning_interface.cpp)
rosbuild_link_boost(${LIBRARY_NAME} thread)

target_link_libraries(${LIBRARY_NAME} itomp_ca)
