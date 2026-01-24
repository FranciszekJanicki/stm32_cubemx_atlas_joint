add_library(common STATIC)

target_sources(common PRIVATE
    manager.c
)

target_include_directories(common PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(common PUBLIC
    atlas_core
    utility
    stm32cubemx
    bus_task
    handle_manager
)

target_compile_options(common PUBLIC
    -std=c23
    -Wall
    -Wextra
    -Wconversion
    -Wshadow
    -Wpedantic
    -Wnarrowing
    -Waddress
    -pedantic
    -Wdeprecated
    -Wsign-conversion
    -Wduplicated-cond
    -Wduplicated-branches
    -Wlogical-op
    -Wnull-dereference
    -Wdouble-promotion
    -Wimplicit-fallthrough
    -Wcast-align
    -Wformat=2
    -Wformat-security
    -Wmissing-prototypes
    -Wmissing-declarations
    -Wstrict-prototypes
    -Wold-style-definition
    -Wundef
    -Wvla
    -Wpointer-arith
    -Wstrict-aliasing=2
)

option(JOINT_PACKET_TEST "Perform joint packet test" OFF)

if (JOINT_PACKET_TEST)
    target_compile_definitions(common PUBLIC
        JOINT_PACKET_TEST
    )
endif()

option(ROBOT_PACKET_TEST "Perform robot packet test" OFF)

if (ROBOT_PACKET_TEST)
    target_compile_definitions(common PUBLIC
        ROBOT_PACKET_TEST
    )
endif()

option(JOINT_TEST "Perform joint move test" ON)

if (JOINT_TEST)
    target_compile_definitions(common PUBLIC
        JOINT_TEST
    )
endif()