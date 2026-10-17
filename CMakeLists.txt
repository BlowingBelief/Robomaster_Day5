cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(TOOLCHAIN_PREFIX "D:\\ST\\STM32CubeCLT_1.22.0\\GNU-tools-for-STM32\\bin")
set(CMAKE_C_COMPILER "${TOOLCHAIN_PREFIX}\\arm-non-eabi-gcc.exe")
set(CMAKE_CXX_COMPILER "${TOOLCHAIN_PREFIX}\\arm-non-eabi-g++.exe")
set(CMAKE_ASM_COMPILER "${TOOLCHAIN_PREFIX}\\arm-non-eabi-gcc.exe")

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME RobomasterTest001)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        Core/Src/callback_1.c
        Core/Inc/pids.h
        Core/Src/pids.cpp
        Core/Inc/Kalman_online.h
        Core/Src/Kalman_online.cpp
        Core/Inc/Kalman_offline.h
        Core/Src/Kalman_offline.cpp
        UserCode/can_user.h
        UserCode/can_user.c
        UserCode/callback.cpp
        UserCode/Motor.h
        UserCode/Motor.cpp)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE UserCode)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
