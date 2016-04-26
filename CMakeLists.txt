cmake_minimum_required(VERSION 2.8.3)
project(arm_slam_calib)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  roscpp
  cv_bridge
  image_transport
  joint_state_recorder
  apriltags
  pcl_ros
)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mssse3")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mssse3")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   sensor_msgs#   std_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES arm_slam_calib
  CATKIN_DEPENDS geometry_msgs std_msgs sensor_msgs visualization_msgs joint_state_recorder cv_bridge apriltags
  DEPENDS Eigen boost dart aikido aikido_rviz assimp
)

find_package(DART REQUIRED COMPONENTS core)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Boost REQUIRED COMPONENTS thread system)
find_package(Eigen REQUIRED)
find_package(GTSAM REQUIRED)
find_package(OMPL REQUIRED)
find_package(TBB REQUIRED)
find_package(OpenCV REQUIRED)
find_package(brisk REQUIRED)
find_package(OpenGV REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(aikido REQUIRED)
find_package(aikido_rviz REQUIRED)
find_package(assimp REQUIRED)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
   include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${DART_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${TBB_INCLUDE_DIRS}
  ${OPENCV_INCLUDE_DIRS}
  ${BRISK_INCLUDE_DIRS}
  ${OPENGV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${aikido_INCLUDE_DIRS}
  ${aikido_rviz_INCLUDE_DIRS}
  ${assimp_INCLUDE_DIRS}
)

## Declare a cpp library
 add_library(arm_slam_calib
	src/ArmSlamCalib.cpp src/FrontEnd.cpp src/RelativePoseAdapter.cpp src/Timer.cpp src/PerlinNoise.cpp src/Utils.cpp src/RobotModifier.cpp src/RobotGenerator.cpp
 )

## Declare a cpp executable
add_executable(sim src/sim.cpp)
add_executable(sim_genrobot src/sim_genrobot.cpp)
add_executable(real src/real.cpp)
add_executable(sim_image src/sim_image.cpp)

target_link_libraries(arm_slam_calib
${catkin_LIBRARIES}
${Boost_LIBRARIES}
${Eigen_LIBRARIES}
${DART_LIBRARIES}
${GTSAM_LIBS}
${OMPL_LIBRARIES}
${TBB_LIBRARIES}
${OPENCV_LIBRARIES}
${BRISK_LIBRARIES}
${OPENGV_LIBS}
${PCL_LIBRARIES}
${aikido_LIBRARIES}
${assimp_LIBRARIES}
${aikido_rviz_LIBRARIES}
)

target_link_libraries(sim
arm_slam_calib
${catkin_LIBRARIES}
)

target_link_libraries(real
arm_slam_calib
${catkin_LIBRARIES}
)


target_link_libraries(sim_image
arm_slam_calib
${catkin_LIBRARIES}
)

target_link_libraries(sim_genrobot
arm_slam_calib
${catkin_LIBRARIES}
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(arm_slam_calib_node arm_slam_calib_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(arm_slam_calib_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS arm_slam_calib arm_slam_calib_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_arm_slam_calib.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
