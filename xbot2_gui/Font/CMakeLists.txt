qt_add_library(xbot2_gui_font STATIC)

qt_add_qml_module(xbot2_gui_font
    URI Font
    VERSION 1.0
    QML_FILES
    MaterialSymbols.qml
)

qt_add_resources(xbot2_gui_font "app_fonts"
    PREFIX "Font/"
    FILES
    "fontawesome-free-6.3.0-desktop/otfs/Font Awesome 6 Free-Regular-400.subset.otf"
    "fontawesome-free-6.3.0-desktop/otfs/Font Awesome 6 Free-Solid-900.subset.otf"
    "materialsymbols/MaterialSymbolsOutlined[opsz,wght,FILL,GRAD@20,200,1,200].otf"
    "materialsymbols/MaterialSymbolsOutlined[opsz,wght,FILL,GRAD@20,200,0,200].otf"
)

target_link_libraries(xbot2_gui PRIVATE xbot2_gui_fontplugin)
