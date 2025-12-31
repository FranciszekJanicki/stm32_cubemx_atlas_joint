include_guard(GLOBAL)

include(${CMAKE_CURRENT_LIST_DIR}/common.cmake)

function(setup_cubemx)
    set(CUBEMX_BUILD_FILE ${CUBEMX_DIR}/../gcc-arm-none-eabi.cmake)
    if(EXISTS ${CUBEMX_BUILD_FILE})
        include(${CUBEMX_BUILD_FILE})
    else()
        message(WARNING "CubeMX CMake build file not found")
    endif()

    add_subdirectory(${CUBEMX_DIR})
endfunction(setup_cubemx)
