cmake_minimum_required(VERSION 3.19)
project(theora_cmake)


set(THEORA_LIB_DIR external/theora/lib)
set(THEORA_INCLUDE_DIR external/theora/include)


file(GLOB SRC_FILES ${THEORA_LIB_DIR}/*.c)


add_library(theora STATIC
    ${SRC_FILES}
)

add_library(theora::theora
    ALIAS theora
)

target_include_directories(theora
    PUBLIC
    ${THEORA_INCLUDE_DIR}
    )
