option(USE_WATCHDOG_TASK "Enable watchdog task" OFF)

if(USE_WATCHDOG_TASK)
    add_library(watchdog_task STATIC)

    target_sources(watchdog_task PRIVATE 
        watchdog_task.c
        watchdog_manager.c
    )

    target_include_directories(watchdog_task PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
    )

    target_link_libraries(watchdog_task PUBLIC
        common
        cubemx
    )

    target_compile_options(watchdog_task PUBLIC
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

    target_compile_definitions(watchdog_task PUBLIC
        USE_WATCHDOG_TASK
    )
endif()