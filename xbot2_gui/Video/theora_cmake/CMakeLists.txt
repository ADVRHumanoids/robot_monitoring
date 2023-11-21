cmake_minimum_required(VERSION 3.19)
project(theora_cmake)

# ogg
add_subdirectory(external/ogg)
add_library(ogglib::ogglib ALIAS ogg)

# theora
set(THEORA_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/theora/include)
file(GLOB THEORA_SRC_FILES external/theora/lib/*.c)

message(STATUS "****THEORA_INCLUDE_DIR=${THEORA_INCLUDE_DIR}")
include_directories(${THEORA_INCLUDE_DIR})

add_library(theora STATIC
    ${THEORA_SRC_FILES}
)

target_include_directories(theora
    PUBLIC
    ${THEORA_INCLUDE_DIR}
    )

target_link_libraries(theora
    PUBLIC ogglib::ogglib
)

add_library(theora::theora
    ALIAS theora
)
