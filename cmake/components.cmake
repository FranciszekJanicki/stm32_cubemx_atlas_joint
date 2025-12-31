include_guard(GLOBAL)

include(${CMAKE_CURRENT_LIST_DIR}/common.cmake)

function(setup_components)
    add_subdirectories(${COMPONENTS_DIR})
endfunction(setup_components)
