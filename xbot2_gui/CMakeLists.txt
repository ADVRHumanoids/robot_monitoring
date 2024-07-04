cmake_minimum_required(VERSION 3.20)
project(xbot2_gui)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt6 COMPONENTS
    Qml
    Core
    Gui
    Quick
    Quick3D
    WebSockets
    Widgets
    QuickControls2
    Network
    ShaderTools
    DataVisualization
    TextToSpeech
    REQUIRED)

if(NOT EMSCRIPTEN)

    find_package(Qt6 COMPONENTS
        WebView
        Protobuf
        3DCore
        3DRender
        3DInput
        REQUIRED
    )

endif()

qt_standard_project_setup(REQUIRES 6.6)


## ui main
qt_add_executable(xbot2_gui main.cpp)

set_property(TARGET xbot2_gui APPEND PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/android)

qt_add_qml_module(xbot2_gui
    VERSION 1.0
    URI Main
    QML_FILES
    main.qml
    PageItem.qml
    ClientEndpoint.qml
    ClientEndpoint.js
    DeserializationWorker.js
    sharedData.js
)

target_compile_definitions(xbot2_gui
    PRIVATE $<$<OR:$<CONFIG:Debug>,$<CONFIG:RelWithDebInfo>>:QT_QML_DEBUG>)

target_link_libraries(xbot2_gui
    PUBLIC
    Qt6::DataVisualization
    Qt6::Core
    Qt6::Quick
    Qt6::WebSockets
    Qt6::QuickControls2
    Qt6::Widgets
    Qt6::TextToSpeech
    Qt6::Quick3D
)

if(NOT EMSCRIPTEN)

    target_link_libraries(xbot2_gui
        PUBLIC
        Qt6::Protobuf
        Qt6::3DCore
        Qt6::3DRender
        Qt6::3DInput
        Qt6::WebView
        mesh_geometry
    )

endif()


## version information
target_compile_definitions(xbot2_gui PRIVATE -DXBOT2_GUI_VERSION_MAJOR=${robot_monitoring_VERSION_MAJOR})
target_compile_definitions(xbot2_gui PRIVATE -DXBOT2_GUI_VERSION_MINOR=${robot_monitoring_VERSION_MINOR})
target_compile_definitions(xbot2_gui PRIVATE -DXBOT2_GUI_VERSION_PATCH=${robot_monitoring_VERSION_PATCH})

## add modules

add_subdirectory(Audio)

add_subdirectory(Common)

add_subdirectory(Concert)

add_subdirectory(Horizon)

add_subdirectory(Font)

add_subdirectory(Ecat)

add_subdirectory(ExpandableBottomBar)

add_subdirectory(Home)

add_subdirectory(LivePlot)

add_subdirectory(Launcher)

add_subdirectory(Menu)

add_subdirectory(Monitoring)

add_subdirectory(Network)

if(NOT EMSCRIPTEN)

    add_subdirectory(Protobuf)

endif()

add_subdirectory(TestThings)

add_subdirectory(Joy)

add_subdirectory(Video)

add_subdirectory(Viewer3D)

add_subdirectory(ViewerQuick3D)


# ws receiver
#add_library(WebsocketReceiver STATIC websocket_receiver.cpp)
#qt_add_qml_module(xbot2_gui
#    URI Common
#    VERSION 1.0
#    SOURCES
#        websocket_receiver.cpp
#)


# robot model
add_subdirectory(RobotModel/)

# quick3d
if(NOT EMSCRIPTEN)
add_library(mesh_geometry STATIC
    ViewerQuick3D/meshgeometry.cpp)

target_link_libraries(mesh_geometry PUBLIC Qt${QT_VERSION_MAJOR}::Quick3D)
endif()

# deploy
if(NOT ANDROID AND NOT EMSCRIPTEN)

    set(QT_ENABLE_VERBOSE_DEPLOYMENT TRUE)

    # Install the executable into "${CMAKE_INSTALL_PREFIX}/bin".
    install(TARGETS xbot2_gui
        BUNDLE  DESTINATION .
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    )

    # Generate the deployment script for the target MyApp.
    qt_generate_deploy_qml_app_script(
        TARGET xbot2_gui
        OUTPUT_SCRIPT deploy_script
        NO_UNSUPPORTED_PLATFORM_ERROR
    )

    # Call the deployment script on "cmake --install".
    install(SCRIPT ${deploy_script})

    # Hack! Add missing files
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/HackDeployMissingFiles.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/cmake/HackDeployMissingFiles.cmake
        @ONLY
    )

    install(SCRIPT ${CMAKE_CURRENT_BINARY_DIR}/cmake/HackDeployMissingFiles.cmake)

endif()

