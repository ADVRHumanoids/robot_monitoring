cmake_minimum_required(VERSION 3.1.0)


if(TARGET Qt5::Charts)

    add_library(chart_widget STATIC chart.cpp chart_resources.qrc)
    target_link_libraries(chart_widget PUBLIC Qt5::Widgets Qt5::Charts Qt5::UiTools)

endif()
