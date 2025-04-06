cmake_minimum_required(VERSION 3.5)
project(noise-filtering-pcl)

find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(VTK REQUIRED)
find_package(PCL REQUIRED)

include(${VTK_USE_FILE})

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

add_executable(${PROJECT_NAME}
    main.cpp
    # MainWindow.cpp
    # MainWindow.h
    # MainWindow.ui
    main_window.h main_window.cpp main_window.ui
)

target_link_libraries(${PROJECT_NAME}
    Qt5::Widgets
    ${PCL_LIBRARIES}
    ${VTK_LIBRARIES}
)
