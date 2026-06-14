# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# === IMPORTANT: setam board-ul Pico 2 W ===
set(PICO_BOARD pico2_w CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(stewart_test C CXX ASM)

pico_sdk_init()

# ========================================================================
# === ALEGE AICI CARE TEST SE COMPILEAZA ===
# Decomenteaza o singura linie. Recompileaza dupa fiecare schimbare.
# ========================================================================
#set(ACTIVE_TEST "test_1_i2c_scan.c")
#set(ACTIVE_TEST "test_2_servo_single.c")
set(ACTIVE_TEST "test_3_servo_all.c")
# set(ACTIVE_TEST "test_4_stewart_home.c")
# ========================================================================

add_executable(stewart_test
    ${ACTIVE_TEST}
    pca9685.c
)

# Linkare cu bibliotecile Pico SDK necesare
target_link_libraries(stewart_test
    pico_stdlib
    hardware_i2c
)

# Activeaza stdout pe USB (pentru a vedea printf in serial monitor)
pico_enable_stdio_usb(stewart_test 1)
pico_enable_stdio_uart(stewart_test 0)

# Genereaza fisiere extra (UF2 pentru flash, hex, etc.)
pico_add_extra_outputs(stewart_test)

# Mesaj informativ la configurare
message(STATUS "Compilez testul: ${ACTIVE_TEST}")
message(STATUS "Board: ${PICO_BOARD}")