# eigen
add_library(InternalEigen INTERFACE IMPORTED)

target_include_directories(InternalEigen PUBLIC INTERFACE
    external/eigen/)


# urdf parser
set(URDFPARSER_ROOT "external/URDF_Parser")

add_library(InternalUrdfParser STATIC
    ${URDFPARSER_ROOT}/src/common.cpp
    ${URDFPARSER_ROOT}/src/geometry.cpp
    ${URDFPARSER_ROOT}/src/joint.cpp
    ${URDFPARSER_ROOT}/src/link.cpp
    ${URDFPARSER_ROOT}/src/model.cpp
    ${URDFPARSER_ROOT}/src/tinyxml.cpp
    ${URDFPARSER_ROOT}/src/tinyxmlerror.cpp
    ${URDFPARSER_ROOT}/src/tinyxmlparser.cpp
)

target_include_directories(InternalUrdfParser PUBLIC
    ${URDFPARSER_ROOT}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/external/boost/include
)


# rbdl
set(RBDL_ROOT "external/rbdl-orb")

add_library(InternalRbdl STATIC
    ${RBDL_ROOT}/src/Dynamics.cc
    ${RBDL_ROOT}/src/Joint.cc
    ${RBDL_ROOT}/src/Kinematics.cc
    ${RBDL_ROOT}/src/Model.cc
    ${RBDL_ROOT}/src/rbdl_errors.cc
    ${RBDL_ROOT}/src/rbdl_mathutils.cc
    ${RBDL_ROOT}/src/rbdl_utils.cc
    ${RBDL_ROOT}/src/rbdl_version.cc
    ${RBDL_ROOT}/addons/urdfreader/urdfreader.cc
)

target_include_directories(InternalRbdl PUBLIC
    ${RBDL_ROOT}/include
    ${CMAKE_CURRENT_BINARY_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${RBDL_ROOT}/addons/urdfreader

)

target_link_libraries(InternalRbdl
    PUBLIC
    InternalUrdfParser
    InternalEigen)




# use rbdl

add_library(RobotModel STATIC
    robot_model.cpp)

target_link_libraries(RobotModel PRIVATE InternalRbdl PUBLIC Qt6::Core)

