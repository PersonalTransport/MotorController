cmake_minimum_required(VERSION 3.2)
project(MotorControllerNode)

include(../cmake-xc16/MPLABX.cmake)

set(SOURCES
    main.c
    as5048a.c
    mcc_generated_files/mcc.c
    mcc_generated_files/spi2.c
    mcc_generated_files/pwm.c
    mcc_generated_files/adc1.c
    mcc_generated_files/pin_manager.c
    lin_generated_files/motor_controller.c
    mcc_generated_files/interrupt_manager.c
)

set(HEADERS
    pid.h
    park.h
    clarke.h
    as5048a.h
    math_utils.h
    igbt_thermistor.h
    space_vector_modulation.h
    mcc_generated_files/mcc.h
    mcc_generated_files/spi2.h
    mcc_generated_files/pwm.h
    mcc_generated_files/adc1.h
    mcc_generated_files/pin_manager.h
    lin_generated_files/motor_controller.h
    mcc_generated_files/interrupt_manager.h
)

set(NODE_FILES
    motor_controller.ncf
)

add_custom_command(OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/lin_generated_files/motor_controller.h ${CMAKE_CURRENT_SOURCE_DIR}/lin_generated_files/motor_controller.c
                   COMMAND LIN -t dsPIC${CMAKE_SYSTEM_PROCESSOR} -i uart1 -mcc -o lin_generated_files ${NODE_FILES}
                   DEPENDS ${NODE_FILES}
                   WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(MotorControllerNode ${SOURCES} ${HEADERS} ${NODE_FILES})
target_link_libraries(MotorControllerNode q)
target_compile_definitions(MotorControllerNode PUBLIC -DFCY=59881250UL)
target_compile_options(MotorControllerNode PUBLIC -std=gnu99)
generate_mplabx_project(MotorControllerNode)
