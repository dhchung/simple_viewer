cmake_minimum_required(VERSION 3.5)

project(simple_viewer LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt5 COMPONENTS Widgets OpenGL REQUIRED)
find_package(OpenGL REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  include/
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  include
  ${PCL_LIBRARY_DIRS}
)

add_executable(${PROJECT_NAME}
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
    myglwidget.cpp
    myglwidget.h
)

target_link_libraries(${PROJECT_NAME} 
    PRIVATE Qt5::Widgets
    ${OPENGL_LIBRARIES} 
    ${PCL_LIBRARIES}
)
