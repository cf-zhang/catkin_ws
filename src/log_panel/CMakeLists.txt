cmake_minimum_required(VERSION 2.8.3)
project(log_panel)

include(CheckCXXCompilerFlag)  
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)  
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)  
if(COMPILER_SUPPORTS_CXX11)  
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")  
elseif(COMPILER_SUPPORTS_CXX0X)  
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")  
else()  
     message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")  
endif()  

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

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
)
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
add_service_files(FILES
  robotCtl.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################a
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS
  message_runtime
#  INCLUDE_DIRS include
#  LIBRARIES log_panel
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
  include/log_panel/antlr-2.7.7/
  include/log_panel/grammar
  include/log_panel/
)


## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
set(CMAKE_AUTOMOC ON)
#add_executable(log_panel
add_library(log_panel
  src/CipherDialog.cpp
  src/sqlitedb.cpp
  src/sqlitetablemodel.cpp
  src/FilterLineEdit.cpp
  src/sqlitetypes.cpp
  src/FilterTableHeader.cpp
  src/Setting.cpp
  src/ExtendedTableWidget.cpp
  src/widget.cpp
  #src/main.cpp
  src/DbStructureModel.cpp
  src/FileDialog.cpp
  src/grammar/Sqlite3Lexer.cpp
  src/grammar/Sqlite3Parser.cpp
  src/antlr/ANTLRUtil.cpp
  src/antlr/ASTFactory.cpp
  src/antlr/ASTNULLType.cpp
  src/antlr/ASTRefCount.cpp
  src/antlr/BaseAST.cpp
  src/antlr/BitSet.cpp
  src/antlr/CharBuffer.cpp
  src/antlr/CharScanner.cpp
  src/antlr/CommonAST.cpp
  src/antlr/CommonASTWithHiddenTokens.cpp
  src/antlr/CommonHiddenStreamToken.cpp
  src/antlr/CommonToken.cpp
  src/antlr/dll.cpp
  src/antlr/InputBuffer.cpp
  src/antlr/LLkParser.cpp
  src/antlr/MismatchedCharException.cpp
  src/antlr/MismatchedTokenException.cpp
  src/antlr/NoViableAltException.cpp
  src/antlr/NoViableAltForCharException.cpp
  src/antlr/Parser.cpp
  src/antlr/RecognitionException.cpp
  src/antlr/String.cpp
  src/antlr/TokenBuffer.cpp
  src/antlr/Token.cpp
  src/antlr/TokenRefCount.cpp
  src/antlr/TokenStreamBasicFilter.cpp
  src/antlr/TokenStreamHiddenTokenFilter.cpp
  src/antlr/TokenStreamRewriteEngine.cpp
  src/antlr/TokenStreamSelector.cpp
  src/antlr/TreeParser.cpp  
)
add_dependencies(log_panel ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(log_panel
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES}
  /usr/lib/x86_64-linux-gnu/libsqlite3.so
  -std=c++11
)
MESSAGE("the install path ${CATKIN_PACKAGE_LIB_DESTINATION}")
install(TARGETS
  ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES  
  src/widget.h 
  src/ExtendedTableWidget.h
  src/FilterTableHeader.h
  src/sqlitedb.h
  src/DbStructureModel.h
  src/sqlitetablemodel.h
  src/Settings.h
  src/sqlite.h
  src/FileDialog.h
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/include)


