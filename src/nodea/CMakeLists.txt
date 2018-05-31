cmake_minimum_required(VERSION 2.8.3)
project(nodea)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_definitions(-w -fpermissive)

# SET (CMAKE_USE_RELATIVE_PATHS ON)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

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
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  roslib
  testgui
)

catkin_package(
  CATKIN_DEPENDS
   roslib
#   message_runtime
#  INCLUDE_DIRS include
#  LIBRARIES button_control
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include ${catkin_INCLUDE_DIRS}
)


link_directories(
  ${catkin_LIB_DIRS}
)
## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
set(CMAKE_AUTOMOC ON)
add_executable(NodeA
    src/NodeA.cpp
    )
target_link_libraries(NodeA ${catkin_LIBRARIES} )
install(TARGETS NodeA RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
## Copy launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)
