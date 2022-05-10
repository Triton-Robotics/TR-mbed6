# Copyright (c) 2020 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.19.0 FATAL_ERROR)

set(MBED_PATH ${CMAKE_CURRENT_SOURCE_DIR}/mbed-os CACHE INTERNAL "")
set(MBED_CONFIG_PATH ${CMAKE_CURRENT_BINARY_DIR} CACHE INTERNAL "")
set(APP_TARGET TR-mbed6)

set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(UTIL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/util)

include(${MBED_PATH}/tools/cmake/app.cmake)

set(SOURCES
    ${UTIL_DIR}/algorithms/pid.hpp
    ${UTIL_DIR}/communications/DJIRemote.cpp
    ${UTIL_DIR}/communications/canHandler.hpp
    ${UTIL_DIR}/communications/CANMsg.h
    ${UTIL_DIR}/communications/djiremoteuart.hpp
    ${UTIL_DIR}/communications/SerialCommunication.hpp
    ${UTIL_DIR}/communications/include/DJIRemote.hpp
    ${UTIL_DIR}/motor/motor.hpp
    ${UTIL_DIR}/motor/pwmmotor.cpp
    ${UTIL_DIR}/testing/inputtester.hpp
    ${UTIL_DIR}/helperFunctions.hpp
    ${SRC_DIR}/main.hpp
    ${SRC_DIR}/robots/engineer.cpp
    ${SRC_DIR}/robots/hero.cpp
    ${SRC_DIR}/robots/infantry.cpp
    ${SRC_DIR}/robots/sentry.cpp
    ${SRC_DIR}/robots/include/engineer.hpp
    ${SRC_DIR}/robots/include/hero.hpp
    ${SRC_DIR}/robots/include/infantry.hpp
    ${SRC_DIR}/robots/include/robot.hpp
    ${SRC_DIR}/robots/include/sentry.hpp
    ${SRC_DIR}/subsystems/ChassisSubsystem.hpp
    ${SRC_DIR}/subsystems/TurretSubsystem.hpp

)
add_executable(${APP_TARGET} ${SOURCES})

add_subdirectory(${MBED_PATH})




mbed_configure_app_target(${APP_TARGET})

include(ExternalProject)

externalproject_add(libmicroros_project
    PREFIX     ${CMAKE_BINARY_DIR}/libmicroros-prefix
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}
    BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}
    CONFIGURE_COMMAND ""
    BUILD_COMMAND
        make -j -f ${CMAKE_CURRENT_SOURCE_DIR}/libmicroros.mk
            X_CC=${CMAKE_C_COMPILER}
            X_AR=${CMAKE_AR}
            X_STRIP=${CMAKE_STRIP}
            X_CFLAGS=${CMAKE_C_FLAGS}
            X_CXX=${CMAKE_CXX_COMPILER}
            X_CXXFLAGS=${CMAKE_CXX_FLAGS}
            BUILD_DIR=${CMAKE_BINARY_DIR}
            EXTENSIONS_DIR=${CMAKE_CURRENT_SOURCE_DIR}
    INSTALL_COMMAND ""
    BUILD_BYPRODUCTS ${CMAKE_CURRENT_SOURCE_DIR}/libmicroros.a
    )

project(${APP_TARGET})

target_sources(${APP_TARGET}
    PRIVATE
        main.cpp
        transports.cpp
)


add_dependencies(${APP_TARGET} libmicroros_project)
target_include_directories(${APP_TARGET} 
    PRIVATE 
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/util
)


target_link_libraries(${APP_TARGET}
    PRIVATE
        mbed-os
        ${CMAKE_CURRENT_SOURCE_DIR}/libmicroros.a
)

mbed_set_post_build(${APP_TARGET})

option(VERBOSE_BUILD "Have a verbose build process")
if(VERBOSE_BUILD)
    set(CMAKE_VERBOSE_MAKEFILE ON)
endif()
