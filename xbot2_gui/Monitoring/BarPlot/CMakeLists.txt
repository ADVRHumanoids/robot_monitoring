add_library(xbot2_gui_monitoring_bar_plot STATIC)

qt_add_qml_module(xbot2_gui_monitoring_bar_plot
    URI Monitoring.BarPlot
    VERSION 1.0
    QML_FILES
    BarPlot.js
    BarPlot.qml
    BarPlotItem.qml
    BarPlotStack.qml
    ReferenceMarker.ui.qml
    TriangleItem.qml
    TwoSideBar.qml
    TwoSideBarForm.ui.qml
)

target_link_libraries(xbot2_gui PRIVATE xbot2_gui_monitoring_bar_plotplugin)
