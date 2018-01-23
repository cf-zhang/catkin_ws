## BEGIN_TUTORIAL
## This CMakeLists.txt file for rviz_plugin_tutorials builds both the
## TeleopPanel tutorial and the ImuDisplay tutorial.
##
## First start with some standard catkin stuff.
cmake_minimum_required(VERSION 2.8.3)
project(rviz_plugin_server)
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
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp 
  rviz
  geometry_msgs
  visualization_msgs
  interactive_markers
  tf
  log_panel
  )


add_service_files(
  FILES
  robotCtl.srv
  robotInfo.srv
  OptGoal.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  interactive_markers 
  roscpp rviz
  visualization_msgs 
  tf
  )
MESSAGE(STATUS “HOME dir: ${catkin_INCLUDE_DIRS}”)
include_directories(
  include ${catkin_INCLUDE_DIRS}
  /home/cfzhang/catkin_ws/devel/include
  include/log_panel/antlr-2.7.7/
  include/log_panel/grammar
  include/log_panel/
  )
link_directories(${catkin_LIBRARY_DIRS})

## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif()

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

## Here we specify the list of source files.
## The generated MOC files are included automatically as headers.
set(SRC_FILES
  src/teleop_mappanel.cpp
  src/teleop_goalpanel.cpp
  src/teleop_model.cpp
  src/teleop_entry.cpp
  src/plant_flag_tool.cpp

  src/log_panel/CipherDialog.cpp
  src/log_panel/sqlitedb.cpp
  src/log_panel/sqlitetablemodel.cpp
  src/log_panel/FilterLineEdit.cpp
  src/log_panel/sqlitetypes.cpp
  src/log_panel/FilterTableHeader.cpp
  src/log_panel/Setting.cpp
  src/log_panel/ExtendedTableWidget.cpp
  src/log_panel/widget.cpp
  #sr/log_panelc/main.cpp
  src/log_panel/DbStructureModel.cpp
  src/log_panel/FileDialog.cpp
  src/log_panel/grammar/Sqlite3Lexer.cpp
  src/log_panel/grammar/Sqlite3Parser.cpp
  src/log_panel/antlr/ANTLRUtil.cpp
  src/log_panel/antlr/ASTFactory.cpp
  src/log_panel/antlr/ASTNULLType.cpp
  src/log_panel/antlr/ASTRefCount.cpp
  src/log_panel/antlr/BaseAST.cpp
  src/log_panel/antlr/BitSet.cpp
  src/log_panel/antlr/CharBuffer.cpp
  src/log_panel/antlr/CharScanner.cpp
  src/log_panel/antlr/CommonAST.cpp
  src/log_panel/antlr/CommonASTWithHiddenTokens.cpp
  src/log_panel/antlr/CommonHiddenStreamToken.cpp
  src/log_panel/antlr/CommonToken.cpp
  src/log_panel/antlr/dll.cpp
  src/log_panel/antlr/InputBuffer.cpp
  src/log_panel/antlr/LLkParser.cpp
  src/log_panel/antlr/MismatchedCharException.cpp
  src/log_panel/antlr/MismatchedTokenException.cpp
  src/log_panel/antlr/NoViableAltException.cpp
  src/log_panel/antlr/NoViableAltForCharException.cpp
  src/log_panel/antlr/Parser.cpp
  src/log_panel/antlr/RecognitionException.cpp
  src/log_panel/antlr/String.cpp
  src/log_panel/antlr/TokenBuffer.cpp
  src/log_panel/antlr/Token.cpp
  src/log_panel/antlr/TokenRefCount.cpp
  src/log_panel/antlr/TokenStreamBasicFilter.cpp
  src/log_panel/antlr/TokenStreamHiddenTokenFilter.cpp
  src/log_panel/antlr/TokenStreamRewriteEngine.cpp
  src/log_panel/antlr/TokenStreamSelector.cpp
  src/log_panel/antlr/TreeParser.cpp 

)

## An rviz plugin is just a shared library, so here we declare the
## library to be called ``${PROJECT_NAME}`` (which is
## "rviz_plugin_tutorials", or whatever your version of this project
## is called) and specify the list of source files we collected above
## in ``${SRC_FILES}``.
add_library(${PROJECT_NAME} ${SRC_FILES}
/usr/lib/x86_64-linux-gnu/libsqlite3.so)

## Link the myviz executable with whatever Qt libraries have been defined by
## the ``find_package(Qt4 ...)`` line above, or by the
## ``set(QT_LIBRARIES Qt5::Widgets)``, and with whatever libraries
## catkin has included.
##
## Although this puts "rviz_plugin_tutorials" (or whatever you have
## called the project) as the name of the library, cmake knows it is a
## library and names the actual file something like
## "librviz_plugin_tutorials.so", or whatever is appropriate for your
## particular OS.
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})
## END_TUTORIAL

## Install rules

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY media/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/media)

install(DIRECTORY icons/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons)

install(PROGRAMS scripts/send_test_msgs.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})