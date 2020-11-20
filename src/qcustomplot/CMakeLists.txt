find_package(OpenGL REQUIRED)

add_library(qcustomplot STATIC
    qcustomplot-source/qcustomplot.cpp)

target_link_libraries(qcustomplot
    PUBLIC
    Qt5::Widgets
    Qt5::PrintSupport
    Qt5::UiTools
    GL)


target_compile_definitions(qcustomplot PUBLIC QCUSTOMPLOT_USE_OPENGL)

target_include_directories(qcustomplot
    PUBLIC qcustomplot-source/)

add_library(qcustom_chart
    STATIC
    qcustom_chart.cpp
    qcustomchart_resources.qrc)

target_link_libraries(qcustom_chart qcustomplot)
