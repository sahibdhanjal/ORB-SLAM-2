cmake_minimum_required(VERSION 2.8.3)
project(orb_slam)

#########################################
# CONFIGURE DEFAULTS
#########################################
IF(NOT ROS_BUILD_TYPE)
  SET(ROS_BUILD_TYPE Release)
ENDIF()
MESSAGE("Build type: " ${ROS_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-deprecated -Wno-deprecated-declarations -Wno-format -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated -Wno-deprecated-declarations -Wno-format -Wno-conversion-null -O3 -march=native")

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    add_definitions(-DCOMPILEDWITHC11)
    message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    add_definitions(-DCOMPILEDWITHC0X)
    message(STATUS "Using flag -std=c++0x.")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()
find_package(OpenMP)
find_package(Pangolin REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Boost COMPONENTS system)


#########################################
# BUILD G2O, DBOW2 and ORB SLAM Libraries
#########################################
find_package(catkin REQUIRED 
  roscpp rospy 
  nav_msgs 
  geometry_msgs 
  sensor_msgs 
  std_msgs 
  tf
  tf2 
  tf2_ros 
  image_transport 
  cv_bridge 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
)

set(GLOBAL_LIBS
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Boost_LIBRARIES}
)

### BUILD DBOW2 Library
add_library(DBoW2 SHARED
  ${PROJECT_SOURCE_DIR}/include/thirdparty/dbow2/dbow2/BowVector.cpp
  ${PROJECT_SOURCE_DIR}/include/thirdparty/dbow2/dbow2/FORB.cpp
  ${PROJECT_SOURCE_DIR}/include/thirdparty/dbow2/dbow2/FeatureVector.cpp
  ${PROJECT_SOURCE_DIR}/include/thirdparty/dbow2/dbow2/ScoringObject.cpp
  ${PROJECT_SOURCE_DIR}/include/thirdparty/dbow2/dutils/Random.cpp
  ${PROJECT_SOURCE_DIR}/include/thirdparty/dbow2/dutils/Timestamp.cpp
)
target_link_libraries(DBoW2 ${GLOBAL_LIBS})

### BUILD G2O Library
add_library(g2o SHARED
  # types
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/types/types_sba.h
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/types/types_six_dof_expmap.h
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/types/types_sba.cpp
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/types/types_six_dof_expmap.cpp
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/types/types_seven_dof_expmap.cpp
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/types/types_seven_dof_expmap.h
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/types/se3quat.h
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/types/se3_ops.h
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/types/se3_ops.hpp
  # core libraries
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/base_edge.h
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/base_binary_edge.h
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/hyper_graph_action.cpp
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/base_binary_edge.hpp
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/hyper_graph_action.h
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/base_multi_edge.h
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/hyper_graph.cpp
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/base_multi_edge.hpp
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/hyper_graph.h
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/base_unary_edge.h
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/linear_solver.h
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/base_unary_edge.hpp
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/marginal_covariance_cholesky.cpp
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/base_vertex.h
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/marginal_covariance_cholesky.h
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/base_vertex.hpp
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/matrix_structure.cpp
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/batch_stats.cpp
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/matrix_structure.h
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/batch_stats.h
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/openmp_mutex.h
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/block_solver.h
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/block_solver.hpp
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/parameter.cpp
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/parameter.h
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/cache.cpp
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/cache.h
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/optimizable_graph.cpp
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/optimizable_graph.h
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/solver.cpp
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/solver.h
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/creators.h
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/optimization_algorithm_factory.cpp
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/estimate_propagator.cpp
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/optimization_algorithm_factory.h
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/estimate_propagator.h
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/factory.cpp
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/optimization_algorithm_property.h
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/factory.h
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/sparse_block_matrix.h
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/sparse_optimizer.cpp
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/sparse_block_matrix.hpp
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/sparse_optimizer.h
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/hyper_dijkstra.cpp
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/hyper_dijkstra.h
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/parameter_container.cpp
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/parameter_container.h
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/optimization_algorithm.cpp
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/optimization_algorithm.h
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/optimization_algorithm_with_hessian.cpp
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/optimization_algorithm_with_hessian.h
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/optimization_algorithm_levenberg.cpp
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/optimization_algorithm_levenberg.h
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/jacobian_workspace.cpp
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/jacobian_workspace.h
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/robust_kernel.cpp
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/robust_kernel.h
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/robust_kernel_factory.cpp
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/robust_kernel_factory.h
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/robust_kernel_impl.cpp
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/core/robust_kernel_impl.h
  # macros and other stuff
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/stuff/string_tools.h
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/stuff/color_macros.h
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/stuff/macros.h
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/stuff/timeutil.cpp
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/stuff/misc.h
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/stuff/timeutil.h
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/stuff/os_specific.c
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/stuff/os_specific.h
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/stuff/string_tools.cpp
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/stuff/property.cpp
  ${PROJECT_SOURCE_DIR}/include/thirdparty/g2o/stuff/property.h
)
target_link_libraries(g2o ${GLOBAL_LIBS})

### BUILD ORB SLAM Library
set(ORB_SLAM_LIBS
  ${GLOBAL_LIBS}
  ${Pangolin_LIBRARIES}
  DBoW2
  g2o
)
add_library(${PROJECT_NAME} SHARED
  ${PROJECT_SOURCE_DIR}/src/orb_core/Converter.cc
  ${PROJECT_SOURCE_DIR}/src/orb_core/Frame.cc
  ${PROJECT_SOURCE_DIR}/src/orb_core/FrameDrawer.cc
  ${PROJECT_SOURCE_DIR}/src/orb_core/Initializer.cc
  ${PROJECT_SOURCE_DIR}/src/orb_core/KeyFrame.cc
  ${PROJECT_SOURCE_DIR}/src/orb_core/KeyFrameDatabase.cc
  ${PROJECT_SOURCE_DIR}/src/orb_core/LocalMapping.cc
  ${PROJECT_SOURCE_DIR}/src/orb_core/LoopClosing.cc
  ${PROJECT_SOURCE_DIR}/src/orb_core/Map.cc
  ${PROJECT_SOURCE_DIR}/src/orb_core/MapDrawer.cc
  ${PROJECT_SOURCE_DIR}/src/orb_core/MapPoint.cc
  ${PROJECT_SOURCE_DIR}/src/orb_core/Optimizer.cc
  ${PROJECT_SOURCE_DIR}/src/orb_core/ORBextractor.cc
  ${PROJECT_SOURCE_DIR}/src/orb_core/ORBmatcher.cc
  ${PROJECT_SOURCE_DIR}/src/orb_core/PnPsolver.cc
  ${PROJECT_SOURCE_DIR}/src/orb_core/Sim3Solver.cc
  ${PROJECT_SOURCE_DIR}/src/orb_core/System.cc
  ${PROJECT_SOURCE_DIR}/src/orb_core/Tracking.cc
  ${PROJECT_SOURCE_DIR}/src/orb_core/Viewer.cc
)
target_link_libraries(${PROJECT_NAME} ${ORB_SLAM_LIBS})


#########################################
# BUILD ORB SLAM ROS Package
#########################################
set(ROS_LIBS
  ${ORB_SLAM_LIBS}
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${Boost_LIBRARIES}
)

catkin_package(
  # INCLUDE_DIRS include
)

add_executable(mono_slam src/ros_mono.cc)
target_link_libraries(mono_slam ${ROS_LIBS})
