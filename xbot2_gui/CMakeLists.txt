cmake_minimum_required(VERSION 3.14)
project(xbot2_gui)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# QtCreator supports the following variables for Android, which are identical to qmake Android variables.
# Check https://doc.qt.io/qt/deployment-android.html for more information.
# They need to be set before the find_package(...) calls below.

#if(ANDROID)
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
#    if (ANDROID_ABI STREQUAL "armeabi-v7a")
#        set(ANDROID_EXTRA_LIBS
#            ${CMAKE_CURRENT_SOURCE_DIR}/path/to/libcrypto.so
#            ${CMAKE_CURRENT_SOURCE_DIR}/path/to/libssl.so)
#    endif()
#endif()

find_package(Qt6 COMPONENTS
    Core
    Quick
    WebSockets
    Widgets
    QuickControls2
    ShaderTools
#    Protobuf
#    3DRender
#    3DCore
    Quick3D
    REQUIRED)

qt_standard_project_setup(REQUIRES 6.6)

set(PROJECT_SOURCES
        main.cpp
        qml.qrc
)

qt_add_executable(xbot2_gui main.cpp)

qt_add_qml_module(xbot2_gui
    VERSION 1.0
    URI Main
    QML_FILES
        main.qml
        PageItem.qml
        ClientEndpoint.qml
        ClientEndpoint.js
        sharedData.js
)

target_compile_definitions(xbot2_gui
  PRIVATE $<$<OR:$<CONFIG:Debug>,$<CONFIG:RelWithDebInfo>>:QT_QML_DEBUG>)

target_link_libraries(xbot2_gui
  PRIVATE Qt${QT_VERSION_MAJOR}::Core
          Qt${QT_VERSION_MAJOR}::Quick
          Qt${QT_VERSION_MAJOR}::WebSockets
          Qt${QT_VERSION_MAJOR}::QuickControls2
          Qt${QT_VERSION_MAJOR}::Widgets
#          Qt${QT_VERSION_MAJOR}::Protobuf
          mesh_geometry
#          Qt${QT_VERSION_MAJOR}::3DRender
#          Qt${QT_VERSION_MAJOR}::3DCore
      )

add_subdirectory(Common)

add_subdirectory(Font)

add_subdirectory(Home)

add_subdirectory(LivePlot)

add_subdirectory(Menu)

add_subdirectory(Protobuf)

add_subdirectory(TestThings)


# ws receiver
#add_library(WebsocketReceiver STATIC websocket_receiver.cpp)
#qt_add_qml_module(xbot2_gui
#    URI Common
#    VERSION 1.0
#    SOURCES
#        websocket_receiver.cpp
#)


# video support
add_subdirectory(Video/theora_cmake)

add_library(VideoStream STATIC
    Video/videostreampainter.cpp
    Menu/CMakeLists.txt
    Home/CMakeLists.txt
    LivePlot/CMakeLists.txt
    TestThings/CMakeLists.txt)

target_link_libraries(VideoStream PRIVATE
    PUBLIC theora::theora
            Qt${QT_VERSION_MAJOR}::Core
            Qt${QT_VERSION_MAJOR}::Quick
            Qt${QT_VERSION_MAJOR}::WebSockets
            Qt${QT_VERSION_MAJOR}::QuickControls2
            Qt${QT_VERSION_MAJOR}::Widgets)

target_link_libraries(xbot2_gui PRIVATE  VideoStream)

qt_add_shaders(xbot2_gui "xbot2_gui-shaders"
    BATCHABLE # Without this it fails
    PREFIX
        "/"
    FILES
        "Video/yuv2rgb.vert"
        "Video/yuv2rgb.frag"
)

# robot model
add_subdirectory(RobotModel/)

target_link_libraries(xbot2_gui PRIVATE RobotModel)

# quick3d
add_library(mesh_geometry STATIC
    ViewerQuick3D/meshgeometry.cpp)

target_link_libraries(mesh_geometry PUBLIC Qt${QT_VERSION_MAJOR}::Quick3D)
