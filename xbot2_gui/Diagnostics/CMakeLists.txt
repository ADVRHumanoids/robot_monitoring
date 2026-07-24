add_library(xbot2_gui_diagnostics STATIC)

qt_add_qml_module(xbot2_gui_diagnostics
    URI Diagnostics
    VERSION 1.0
    QML_FILES
    SOURCES
    treeitem.cpp
    treeitem.h
    treemodel.cpp
    treemodel.h
    diagnosticfiltermodel.cpp
    diagnosticfiltermodel.h
    QML_FILES
    Diagnostics.qml
    NameDelegate.qml
    LevelDelegate.qml
    DetailedView.qml
    Diagnostics.js
    QML_FILES DiagnosticsTreeView.qml
    QML_FILES DiagnosticsSummary.qml
)

target_link_libraries(xbot2_gui_diagnostics PUBLIC Qt6::Gui Qt6::Quick)

target_link_libraries(xbot2_gui_diagnosticsplugin PUBLIC Qt6::Gui Qt6::Quick)

target_link_libraries(xbot2_gui PRIVATE xbot2_gui_diagnosticsplugin)


