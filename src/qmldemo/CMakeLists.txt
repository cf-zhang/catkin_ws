cmake_minimum_required(VERSION 2.8.3)
project(qmldemo)

## Compile as C++11, supported in ROS Kinetic and newer
#set(CMAKE_BUILD_TYPE "Release") #using CMAKE_BUILD_TYPE to set the cmake method [Debug | Release]
# CMAKE_CXX_FLAGS CMake C++
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2 -pthread -fPIE")  #-fopenmp add-on multi-thread openmp support



#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
aux_source_directory(src/    SRC_LIST)

# Add in Qt
find_package(Qt5 COMPONENTS Core Quick REQUIRED)
#find_package(Qt5Core)
#find_package(Qt5Widgets)
#find_package(Qt5Gui)
#find_package(Quick)
#find_package(QtQML)
# ${QT_INCLUDE_DIRS} ${QT_LIBRARIES} and so on are all Qt4 Macro Definitions!!!!!
#include_directories(${Qt5Core_INCLUDE_DIRS}
#    ${Qt5Gui_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS}
#    ${Qt5Quick_INCLUDE_DIRS} ${Qt5QML_INCLUDE_DIRS})
#add_definitions(${Qt5Core_DEFINITIONS})

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS roscpp rosnode std_msgs message_generation)
include_directories(${catkin_INCLUDE_DIRS})
# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything.
#add_service_files(
#   FILES
# )
#generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
#  )
#catkin_package()

set(CMAKE_AUTOMOC ON)

include_directories(
  )
#set(SPEC_OPENCV_LIBS libopencv_features2d.so libopencv_imgproc.so libopencv_calib3d.so libopencv_highgui.so libopencv_core.so libopencv_ml.so)

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/testgui/*.hpp)

QT5_ADD_RESOURCES(qrc_FILES ${QT_RESOURCES})
#QT5_WRAP_UI(ui_FILES ${QT_FORMS})
#QT5_WRAP_CPP(moc_HPP ${QT_MOC})

add_executable(${PROJECT_NAME} ${SRC_LIST} ${qrc_FILES})
#add_executable(${PROJECT_NAME} ${SRC_LIST} ${ui_FILES} ${qrc_FILES} ${moc_HPP})
# Use the Widgets module from Qt 5
qt5_use_modules(${PROJECT_NAME} Core Widgets Quick Qml Gui)


target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    sqlite3
    )

