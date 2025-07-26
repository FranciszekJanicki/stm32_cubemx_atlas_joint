cmake_minimum_required(VERSION 4.0)

include("cubemx/cmake/gcc-arm-none-eabi.cmake")

project(${PROJECT_NAME} LANGUAGES C CXX ASM)

set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(BLA_VENDOR "" CACHE STRING "" FORCE)
set(BLAS_LIBRARIES "" CACHE STRING "" FORCE)
set(LAPACK_LIBRARIES "" CACHE STRING "" FORCE)

set(CMAKE_C_STANDARD 23)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

set(PROJECT_DIR ${CMAKE_SOURCE_DIR})
set(MAIN_DIR ${PROJECT_DIR}/main)
set(COMPONENTS_DIR ${PROJECT_DIR}/components)
set(THIRD_PARTY_DIR ${PROJECT_DIR}/third_party)
set(CUBEMX_DIR ${PROJECT_DIR}/cubemx/cmake/stm32cubemx)

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

set(CMAKE_C_FLAGS_DEBUG "-O0 -g")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

add_subdirectory(${THIRD_PARTY_DIR})
add_subdirectory(${CUBEMX_DIR})
add_subdirectory(${COMPONENTS_DIR})
add_subdirectory(${MAIN_DIR})

target_compile_options(stm32cubemx INTERFACE
    -w
)
