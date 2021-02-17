cmake_minimum_required(VERSION 3.5)

project(LidarQT LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(VTK
  COMPONENTS
    # CommonCore
    GUISupportQt
    # IOImage
    # ImagingColor
    # ImagingGeneral
    # InteractionImage
    #InteractionStyle
    # InteractionWidgets
    )
    
find_package(PCL
    REQUIRED
)

find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED QUIET)
include_directories("/usr/local/include/vtk-9.0/")
#list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
#add_definitions(${PCL_DEFINITIONS})

set(PROJECT_SOURCES
        main.cpp
        mainwindow.cpp
        mainwindow.h
        mainwindow.ui
)

add_executable(LidarQT ${PROJECT_SOURCES})
target_link_libraries(LidarQT pthread ${PCL_LIBRARIES} Qt5::Core Qt5::Gui Qt5::Widgets VTK::GUISupportQt pthread)

vtk_module_autoinit(
  TARGETS LidarQT
  MODULES ${VTK_LIBRARIES})
