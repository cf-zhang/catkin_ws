cmake_minimum_required(VERSION 2.8.3)
project(robot_log)

if(NOT WIN32)
  set_directory_properties(PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra")
endif()


option(UseQt5 "UseQt5" ON)
if (UseQt5)
  find_package(Qt5 REQUIRED COMPONENTS Core Sql Widgets OpenGL)
  # set variable names already used with Qt4
  set(QT_LIBRARIES Qt5::Widgets Qt5::Sql)
  set(QTVERSION ${Qt5Widgets_VERSION})
else()
  find_package(Qt4 REQUIRED COMPONENTS QtCore QtSql QtGui QtOpenGL)
  include(${QT_USE_FILE})
endif()
add_definitions(-DQT_NO_KEYWORDS)



find_package(catkin REQUIRED COMPONENTS roscpp rosgraph_msgs)

catkin_package()

include_directories(
  include ${catkin_INCLUDE_DIRS}
  include ${catkin_INCLUDE_DIRS})

add_executable(robot_log
  src/dbmanager.cpp src/robot_log.cpp)
target_link_libraries(robot_log ${catkin_LIBRARIES} ${QT_LIBRARIES})

add_executable(t src/t.cpp)
target_link_libraries(t ${catkin_LIBRARIES})

