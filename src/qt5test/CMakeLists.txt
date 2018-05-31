cmake_minimum_required(VERSION 2.8.3)
project(qt5test)

#set(CMAKE_BUILD_TYPE "Release") #using CMAKE_BUILD_TYPE to set the cmake method [Debug | Release]
# CMAKE_CXX_FLAGS CMake C++
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2 -pthread -fPIE")  #-fopenmp add-on multi-thread openmp support



#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
aux_source_directory(src/    SRC_LIST)

# Add in Qt
find_package(Qt5Core)
find_package(Qt5Widgets)
find_package(Qt5Gui)
find_package(Qt5OpenGL)
find_package(Qt5Xml)
# ${QT_INCLUDE_DIRS} ${QT_LIBRARIES} and so on are all Qt4 Macro Definitions!!!!!
include_directories(${Qt5Core_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS} ${Qt5OpenGL_INCLUDE_DIRS} ${Qt5Xml_INCLUDE_DIRS})
#add_definitions(${Qt5Core_DEFINITIONS})



# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS roscpp rosnode std_msgs message_generation)
include_directories(${catkin_INCLUDE_DIRS})
# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything.
add_service_files(
   FILES
   gui.srv
 )
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
  )

catkin_package()

set(CMAKE_AUTOMOC ON)

include_directories(
  include/log_panel/antlr-2.7.7/
  include/log_panel/grammar
  include/log_panel/
  )
#set(SPEC_OPENCV_LIBS libopencv_features2d.so libopencv_imgproc.so libopencv_calib3d.so libopencv_highgui.so libopencv_core.so libopencv_ml.so)

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/testgui/*.hpp)

QT5_ADD_RESOURCES(qrc_FILES ${QT_RESOURCES})
QT5_WRAP_UI(ui_FILES ${QT_FORMS})
QT5_WRAP_CPP(moc_HPP ${QT_MOC})

add_executable(${PROJECT_NAME} ${SRC_LIST} ${ui_FILES} ${qrc_FILES} ${moc_HPP}
      src/log_panel/CipherDialog.cpp
      src/log_panel/sqlitedb.cpp
      src/log_panel/sqlitetablemodel.cpp
      src/log_panel/FilterLineEdit.cpp
      src/log_panel/sqlitetypes.cpp
      src/log_panel/FilterTableHeader.cpp
      src/log_panel/Setting.cpp
      src/log_panel/ExtendedTableWidget.cpp
      src/log_panel/widget.cpp
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

# Use the Widgets module from Qt 5
qt5_use_modules(${PROJECT_NAME} Core Widgets OpenGL Xml Gui)


target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    sqlite3
#    /home/cfzhang/xcode/catkin_qt/src/qt5test/lib/log_panel/liblog_panel.so
    )







































