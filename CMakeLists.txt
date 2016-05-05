cmake_minimum_required(VERSION 3.2)
project(MotorControllerNode)

include(../cmake-xc16/MPLABX.cmake)

set(SOURCES
    src/main.c
    gen/motor_controller.c
)

set(HEADERS
    gen/motor_controller.h
)

set(NODE_FILES
    motor_controller.ncf
)

add_custom_command(OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/gen/motor_controller.h ${CMAKE_CURRENT_SOURCE_DIR}/gen/motor_controller.c
                   COMMAND LIN -t PIC${CMAKE_SYSTEM_PROCESSOR} -i uart1 ${NODE_FILES}
                   DEPENDS ${NODE_FILES}
                   WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(MotorControllerNode ${SOURCES} ${HEADERS} ${NODE_FILES})
target_compile_definitions(MotorControllerNode PUBLIC -DFCY=2000000)
target_compile_options(MotorControllerNode PUBLIC -std=gnu99)
target_include_directories(MotorControllerNode PUBLIC include gen)
generate_mplabx_project(MotorControllerNode)
