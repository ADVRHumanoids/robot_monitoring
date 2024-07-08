add_library(xbot2_gui_launcher STATIC)

qt_add_qml_module(xbot2_gui_launcher
    URI Launcher
    VERSION 1.0
    QML_FILES
    ConfigurePanel.js
    ConfigurePanel.qml
    Console.qml
    ConsoleCard.js
    ConsoleCard.qml
    Launcher.js
    Launcher.qml
    LauncherConsoleItem.qml
    PluginCard.qml
    ProcessCard.qml
    CustoCommand.qml
    Dashboard.qml
    QML_FILES Dashboard.js
    QML_FILES DashboardStatus.qml
)

target_link_libraries(xbot2_gui PRIVATE xbot2_gui_launcherplugin)
