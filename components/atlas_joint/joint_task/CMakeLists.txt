add_library(joint_task STATIC)

target_sources(joint_task PRIVATE 
    joint_task.c
    joint_manager.c
    joint_interface.c
)

target_include_directories(joint_task PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(joint_task PUBLIC
    motor_driver
    drv8825
    step_motor
    pid_regulator
    stm32cubemx
    as5600
    tca9548
    ina226
    common
)

target_compile_options(joint_task PUBLIC
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