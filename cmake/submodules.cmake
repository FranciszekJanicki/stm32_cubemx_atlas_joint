include_guard(GLOBAL)

include(${CMAKE_CURRENT_LIST_DIR}/common.cmake)

function(setup_submodules)
    add_subdirectories(${SUBMODULES_DIR})
endfunction(setup_submodules)
