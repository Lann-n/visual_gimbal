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
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIREdD ON)
set(CMAKE_CXX_EXTENSIONS ON)
set(SIZE arm-none-eabi-size)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME visual_gimbal)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Add color diagnostics
set(CMAKE_COLOR_DIAGNOSTICS ON)

include("cmake/gcc-arm-none-eabi.cmake") # toolchain

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C CXX ASM)

# Remove warning
add_compile_options(-w)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Generate .map file automatically
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
        "-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.map"
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# ==============================
# 包含dir以下后缀文件定义函数
# ==============================
function(USER_ADD_DIR dir)
    file(GLOB USER_SOURCE
            ${dir}/*.h
            ${dir}/*.c
            ${dir}/*.hpp
            ${dir}/*.cpp
            ${dir}/*.cc
    )
    target_sources(${CMAKE_PROJECT_NAME} PUBLIC ${USER_SOURCE})
    target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC ${dir})
endfunction()
# ==============================
# 模块开关定义函数
# ==============================
function(ADD_MODULE name default_state)
    # ON 或 OFF
    if(NOT default_state MATCHES "ON|OFF")
        message(FATAL_ERROR "ADD_MODULE(${name} …) 第二个参数必须是 ON 或 OFF")
    endif()

    # 判断用户是否已经在命令行中配置了值
    if(NOT DEFINED USE_${name})
        set(USE_${name} ${default_state} CACHE BOOL "Enable ${name} module")
    endif()

    # 获取额外路径：去掉前两个参数
    set(extra_paths ${ARGV})
    list(REMOVE_AT extra_paths 0)
    list(REMOVE_AT extra_paths 0)

    if(USE_${name})
        message(STATUS "✅ ${name} ENABLED (default=${default_state})")
        add_compile_definitions(USE_${name}=1)

        foreach(path IN LISTS extra_paths)
            USER_ADD_DIR(${path})
        endforeach()
    else()
        message(STATUS "❌ ${name} DISABLED (default=${default_state})")
    endif()
endfunction()
# ==============================
# 计入模块选择
# ==============================
ADD_MODULE(SINGLE_USART OFF
        ${CMAKE_SOURCE_DIR}/ACE_ECF_26/Bsp/USART
)
if(EXISTS "${CMAKE_SOURCE_DIR}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c")
    ADD_MODULE(SINGLE_CAN OFF
            ${CMAKE_SOURCE_DIR}/ACE_ECF_26/Bsp/DWT
            ${CMAKE_SOURCE_DIR}/ACE_ECF_26/Bsp/CAN
            ${CMAKE_SOURCE_DIR}/ACE_ECF_26/Bsp/CAN/fdcan
    )
else()
    ADD_MODULE(SINGLE_CAN OFF
            ${CMAKE_SOURCE_DIR}/ACE_ECF_26/Bsp/DWT
            ${CMAKE_SOURCE_DIR}/ACE_ECF_26/Bsp/CAN
            ${CMAKE_SOURCE_DIR}/ACE_ECF_26/Bsp/CAN/can
    )
endif()
if(EXISTS "${CMAKE_SOURCE_DIR}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c")
    ADD_MODULE(DM_MOTOR OFF
            ${CMAKE_SOURCE_DIR}/ACE_ECF_26/Bsp/DWT
            ${CMAKE_SOURCE_DIR}/ACE_ECF_26/App/safe_task
            ${CMAKE_SOURCE_DIR}/ACE_ECF_26/Bsp/CAN
            ${CMAKE_SOURCE_DIR}/ACE_ECF_26/Bsp/CAN/fdcan
            ${CMAKE_SOURCE_DIR}/ACE_ECF_26/module/motor
            ${CMAKE_SOURCE_DIR}/ACE_ECF_26/module/motor/DM
    )
else()
    ADD_MODULE(DM_MOTOR ON
            ${CMAKE_SOURCE_DIR}/ACE_ECF_26/Bsp/DWT
            ${CMAKE_SOURCE_DIR}/ACE_ECF_26/App/safe_task
            ${CMAKE_SOURCE_DIR}/ACE_ECF_26/Bsp/CAN
            ${CMAKE_SOURCE_DIR}/ACE_ECF_26/Bsp/CAN/can
            ${CMAKE_SOURCE_DIR}/ACE_ECF_26/module/motor
            ${CMAKE_SOURCE_DIR}/ACE_ECF_26/module/motor/DM
    )
endif()
if(EXISTS "${CMAKE_SOURCE_DIR}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c")
    ADD_MODULE(DJI_MOTOR OFF
            ${CMAKE_SOURCE_DIR}/ACE_ECF_26/Bsp/DWT
            ${CMAKE_SOURCE_DIR}/ACE_ECF_26/App/safe_task
            ${CMAKE_SOURCE_DIR}/ACE_ECF_26/Bsp/CAN
            ${CMAKE_SOURCE_DIR}/ACE_ECF_26/Bsp/CAN/fdcan
            ${CMAKE_SOURCE_DIR}/ACE_ECF_26/module/motor
            ${CMAKE_SOURCE_DIR}/ACE_ECF_26/module/motor/DJI
    )
else()
    ADD_MODULE(DJI_MOTOR ON
            ${CMAKE_SOURCE_DIR}/ACE_ECF_26/Bsp/DWT
            ${CMAKE_SOURCE_DIR}/ACE_ECF_26/App/safe_task
            ${CMAKE_SOURCE_DIR}/ACE_ECF_26/Bsp/CAN
            ${CMAKE_SOURCE_DIR}/ACE_ECF_26/Bsp/CAN/can
            ${CMAKE_SOURCE_DIR}/ACE_ECF_26/module/motor
            ${CMAKE_SOURCE_DIR}/ACE_ECF_26/module/motor/DJI
    )
endif()
ADD_MODULE(DR16 ON
        ${CMAKE_SOURCE_DIR}/ACE_ECF_26/Bsp/DWT
        ${CMAKE_SOURCE_DIR}/ACE_ECF_26/App/safe_task
        ${CMAKE_SOURCE_DIR}/ACE_ECF_26/Bsp/USART
        ${CMAKE_SOURCE_DIR}/ACE_ECF_26/module/dr16
)
ADD_MODULE(REFEREE OFF
        ${CMAKE_SOURCE_DIR}/ACE_ECF_26/Bsp/DWT
        ${CMAKE_SOURCE_DIR}/ACE_ECF_26/App/safe_task
        ${CMAKE_SOURCE_DIR}/ACE_ECF_26/Bsp/USART
        ${CMAKE_SOURCE_DIR}/ACE_ECF_26/module/referee
)
ADD_MODULE(VT13 OFF
        ${CMAKE_SOURCE_DIR}/ACE_ECF_26/Bsp/DWT
        ${CMAKE_SOURCE_DIR}/ACE_ECF_26/App/safe_task
        ${CMAKE_SOURCE_DIR}/ACE_ECF_26/Bsp/USART
        ${CMAKE_SOURCE_DIR}/ACE_ECF_26/module/referee
)
ADD_MODULE(SERVO OFF
        ${CMAKE_SOURCE_DIR}/ACE_ECF_26/Bsp/PWM
        ${CMAKE_SOURCE_DIR}/ACE_ECF_26/module/servo
)
ADD_MODULE(MUSIC OFF
        ${CMAKE_SOURCE_DIR}/ACE_ECF_26/Bsp/PWM
        ${CMAKE_SOURCE_DIR}/ACE_ECF_26/Module/buzzer
        ${CMAKE_SOURCE_DIR}/ACE_ECF_26/App/music_task
)
ADD_MODULE(EXTI OFF
        ${CMAKE_SOURCE_DIR}/ACE_ECF_26/Bsp/EXTI
)
ADD_MODULE(IMU ON
        ${CMAKE_SOURCE_DIR}/ACE_ECF_26/Bsp/SPI
        ${CMAKE_SOURCE_DIR}/ACE_ECF_26/Bsp/DWT
        ${CMAKE_SOURCE_DIR}/ACE_ECF_26/Bsp/PWM
        ${CMAKE_SOURCE_DIR}/ACE_ECF_26/Module/imu
)
ADD_MODULE(SINGLE_FLASH OFF
        ${CMAKE_SOURCE_DIR}/ACE_ECF_26/Bsp/FLASH
)
ADD_MODULE(LOCAL_FLASH_LOG OFF
        ${CMAKE_SOURCE_DIR}/ACE_ECF_26/Bsp/FLASH
        ${CMAKE_SOURCE_DIR}/ACE_ECF_26/Module/log
        ${CMAKE_SOURCE_DIR}/ACE_ECF_26/Module/log/LocalFlash
)
ADD_MODULE(POWER_CTRL OFF
        ${CMAKE_SOURCE_DIR}/ACE_ECF_26/Bsp/DWT
        ${CMAKE_SOURCE_DIR}/ACE_ECF_26/App/safe_task
        ${CMAKE_SOURCE_DIR}/ACE_ECF_26/Bsp/USART
        ${CMAKE_SOURCE_DIR}/ACE_ECF_26/module/referee
        ${CMAKE_SOURCE_DIR}/ACE_ECF_26/App/power_ctrl
)

# ==============================
# 26框架库固定文件
# ==============================
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/ACE_ECF_26)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/ACE_ECF_26/App/robo_cmd)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/ACE_ECF_26/Algorithm)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/ACE_ECF_26/Algorithm/CRC)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/ACE_ECF_26/Algorithm/filter_alg)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/ACE_ECF_26/Algorithm/kalman_filter)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/ACE_ECF_26/Algorithm/lqr)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/ACE_ECF_26/Algorithm/PID)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/ACE_ECF_26/Algorithm/quaternion_kalman)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/ACE_ECF_26/Algorithm/user_maths)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/ACE_ECF_26/Algorithm/user_maths/cpp_files)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/ACE_ECF_26/Algorithm/user_maths/fifo)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/ACE_ECF_26/Algorithm/RLS)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/ACE_ECF_26/Algorithm/structure)

# ==============================
# 自己的文件加这里（TASK)
# ==============================
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Task)
USER_ADD_DIR(${CMAKE_SOURCE_DIR}/Task/app)

# 判断芯片型号目录是否存在
if(EXISTS "${CMAKE_SOURCE_DIR}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c")
    set(DSP_LIB_PATH "${CMAKE_SOURCE_DIR}/Middlewares/ST/ARM/DSP/Lib/libarm_cortexM7lfdp_math.a")
else()
    set(DSP_LIB_PATH "${CMAKE_SOURCE_DIR}/Middlewares/ST/ARM/DSP/Lib/libarm_cortexM4lf_math.a")
endif()

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
        stm32cubemx
        # Add user defined libraries
)

add_custom_command(
        TARGET ${PROJECT_NAME} POST_BUILD
        # COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex
        # COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin
        COMMENT "Building hex & bin file...\nEXCUTABLE SIZE:"
        COMMAND ${SIZE} ${PROJECT_NAME}.elf
)
