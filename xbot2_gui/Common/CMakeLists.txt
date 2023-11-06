qt_add_library(xbot2_gui_common STATIC)

set_source_files_properties(CommonProperties.qml PROPERTIES
    QT_QML_SINGLETON_TYPE TRUE
)

qt_add_qml_module(xbot2_gui_common
    URI Common
    VERSION 1.0
    QML_FILES
    AspectRatio.qml
    Card.qml
    CommonProperties.qml
    DebugRectangle.qml
    DoubleSpinBox.qml
    LabeledProgressBar.qml
    MaterialResponsiveGrid.qml
    MultiColumnPage.qml
    MultiMouseArea.qml
    SectionHeader.qml
    SmallToolButton.qml
)

target_link_libraries(xbot2_gui PRIVATE xbot2_gui_commonplugin)
