cmake_minimum_required(VERSION 4.0)

set(CMAKE_DIR ${CMAKE_CURRENT_LIST_DIR}/cmake)

include(${CMAKE_DIR}/build.cmake)
include(${CMAKE_DIR}/cubemx.cmake)
include(${CMAKE_DIR}/main.cmake)
include(${CMAKE_DIR}/common.cmake)
include(${CMAKE_DIR}/components.cmake)
include(${CMAKE_DIR}/submodules.cmake)

project(${CMAKE_PROJECT_NAME} LANGUAGES C CXX ASM)
message(STATUS "Project name: ${CMAKE_PROJECT_NAME}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Toolchain file: ${CMAKE_TOOLCHAIN_FILE}") 

setup_cubemx()
setup_main()
setup_components()
setup_submodules()
