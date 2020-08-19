cmake_minimum_required(VERSION 2.8.3)
project(thorvald_hilly_nav)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2_geometry_msgs
  visualization_msgs
  gazebo_ros
  gazebo_plugins
  pcl_ros
)

if(CMAKE_HOST_UNIX)
  catkin_add_env_hooks(hilly_nav_gazebo SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env_hooks)
else()
  message(FATAL_ERROR "Only UNIX builds supported.")
endif()

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
find_package(gazebo REQUIRED)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
pkg_check_modules( EIGEN3 REQUIRED eigen3 )

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES thorvald_hilly_nav
#  CATKIN_DEPENDS cv_bridge geometry_msgs message_generation nav_msgs roscpp rospy sensor_msgs std_msgs tf tf2_geometry_msgs visualization_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/thorvald_hilly_nav.cpp
# )

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(orchards_hilly_nav src/orchards_hilly_nav.cpp)
add_executable(orchards_hilly_nav_offline src/orchards_hilly_nav_offline.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(orchards_hilly_nav
  ${OpenCV_LIBS} ${catkin_LIBRARIES} Eigen3::Eigen
)
target_link_libraries(orchards_hilly_nav_offline
  ${OpenCV_LIBS} ${catkin_LIBRARIES} Eigen3::Eigen
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(orchards_hilly_nav ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(orchards_hilly_nav_offline ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  scripts/online_da_predict.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY
  include
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
