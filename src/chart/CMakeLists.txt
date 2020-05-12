cmake_minimum_required(VERSION 3.1.0)

add_compile_options(-std=c++14)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(QT5_PATH "" CACHE PATH "Path to QT > 5.9")
set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};${QT5_PATH}")


find_package(Qt5UiTools 5.12 REQUIRED)
find_package(Qt5Widgets 5.12 REQUIRED)
find_package(Qt5Charts 5.12 REQUIRED)

add_library(chart_widget STATIC chart.cpp chart_resources.qrc)
target_link_libraries(chart_widget PUBLIC Qt5::Widgets Qt5::Charts Qt5::UiTools)
