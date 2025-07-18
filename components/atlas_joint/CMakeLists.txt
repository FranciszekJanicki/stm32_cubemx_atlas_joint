add_subdirectory(common)
add_subdirectory(system_task)
add_subdirectory(joint_task)
add_subdirectory(packet_task)

add_library(atlas_joint STATIC)

target_sources(atlas_joint PRIVATE 
    atlas_joint.c
)  

target_include_directories(atlas_joint PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(atlas_joint PUBLIC 
    common
    system_task
    joint_task
    uart_task
    packet_task
    stm32cubemx
)

target_compile_options(atlas_joint PUBLIC
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
