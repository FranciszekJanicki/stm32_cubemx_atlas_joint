add_subdirectory(common)
add_subdirectory(system_task)
add_subdirectory(joints_task)
add_subdirectory(packet_task)

add_library(atlas_rob STATIC)

target_sources(atlas_rob PRIVATE 
    atlas_rob.c
)  

target_include_directories(atlas_rob PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(atlas_rob PUBLIC 
    common
    system_task
    joints_task
    uart_task
    packet_task
    stm32cubemx
)

target_compile_options(atlas_rob PUBLIC
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