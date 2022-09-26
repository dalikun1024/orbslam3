cmake_minimum_required(VERSION 3.8)
project(orbslam3)

# IF(NOT ROS_BUILD_TYPE)
#   SET(ROS_BUILD_TYPE Release)
# ENDIF()

MESSAGE("Build type: " ${ROS_BUILD_TYPE})

# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

# Check C++14 or C++0x support
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# include(CheckCXXCompilerFlag)
# CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
# CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
# if(COMPILER_SUPPORTS_CXX14)
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
#    add_definitions(-DCOMPILEDWITHC11)
#    message(STATUS "Using flag -std=c++14.")
# elseif(COMPILER_SUPPORTS_CXX0X)
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
#    add_definitions(-DCOMPILEDWITHC0X)
#    message(STATUS "Using flag -std=c++0x.")
# else()
#    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
# endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake_modules)



find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(OpenCV 4.4)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 4.4 not found.")
   endif()
MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/../../../
${PROJECT_SOURCE_DIR}/../../../include
${PROJECT_SOURCE_DIR}/../../../include/CameraModels
${PROJECT_SOURCE_DIR}/../../../Thirdparty/Sophus
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
)

set(LIBS 
${OpenCV_LIBS} 
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/../../../Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o.so
${PROJECT_SOURCE_DIR}/../../../lib/libORB_SLAM3.so
-lboost_system
)


# Node for stereo camera
add_executable(Stereo
src/ros_stereo.cc
)
ament_target_dependencies(Stereo rclcpp std_msgs sensor_msgs image_transport cv_bridge geometry_msgs tf2 tf2_ros nav_msgs)

# Node for stereo image publisher
add_executable(StereoPublisher
src/ros_stereo_publisher.cc
)
ament_target_dependencies(StereoPublisher rclcpp std_msgs sensor_msgs image_transport cv_bridge)

target_link_libraries(Stereo
${LIBS}
)

set_target_properties(Stereo PROPERTIES 
  INSTALL_RPATH "/work/orb_slam3/lib")

target_link_libraries(StereoPublisher
  ${LIBS}
)
  
set_target_properties(StereoPublisher PROPERTIES 
    INSTALL_RPATH "/work/orb_slam3/lib")

install(TARGETS Stereo StereoPublisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()