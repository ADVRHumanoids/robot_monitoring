# compile eigen
add_library(InternalEigen STATIC IMPORTED)
target_include_directories(InternalEigen PUBLIC INTERFACE
    external/eigen/)

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
)
target_include_directories(InternalRbdl PUBLIC
    ${RBDL_ROOT}/include)


# use rbdl

add_library(robot_model STATIC
    robot_model.cpp)

target_link_libraries(robot_model RBDL)

