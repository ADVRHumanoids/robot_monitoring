qt_add_library(xbot2_gui_liveplot STATIC)

qt_add_qml_module(xbot2_gui_liveplot
    URI LivePlot
    VERSION 1.0
    QML_FILES
    Plotter.qml
    PlotterLegend.qml
)

target_link_libraries(xbot2_gui PRIVATE xbot2_gui_liveplotplugin)
