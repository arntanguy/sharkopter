cmake_minimum_required(VERSION 2.8)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/arduino-cmake/cmake/ArduinoToolchain.cmake)


set(PROJECT_NAME arduino)
project(PROJECT_NAME)


#set(${FIRMWARE_NAME}_PORT /path/to/device)

# ===============================
# Board config
# ===============================
# Board id (such as uno, mega2560, pro5v328...)
set(BOARD_ID pro5v328)
# Serial port, for upload and serial targets
set(SERIAL_PORT /dev/ttyACM0)
# Baudrate (9600, 115200...)
set(BAUD_RATE 9600)
# Serial command for serial target
set(SERIAL_COMMAND picocom ${SERIAL_PORT} -b ${BAUD_RATE} -l)
# Programmer ID, enables programmer burning
set(PROGRAMMER_ID arduinoisp)
# arvude flags for target
set(AFLGS)

# =============================
# Files to compile
# =============================
set(SOURCES src/blink.cpp)
set(HEADERS)
set(LIBRARIES)

# =============================
# Libraries
# =============================
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/libraries)
## If a library contains nested source, a special option must be set to enable recursion
## The option name should be ${LIBRARY_NAME}_RECURSE, where in this case LIBRARY_NAME is equal to Wire.
#set(Wire_RECURSE True)
#generate_arduino_library(name
#    ${BOARD_ID}
#     [SRCS  src1 src2 ... srcN]
#     [HDRS  hdr1 hdr2 ... hdrN]
#     [LIBS  lib1 lib2 ... libN]
#     [NO_AUTOLIBS])

# =============================
# Compile/Upload
# =============================
# Other options include:
# - ARDLIBS 	Manual list of Arduino type libraries, common use case is when the library header name does not match the librarie's directory name. ADVANCED OPTION! Can be used in conjuction with NO_AUTOLIBS.
# - SKETCH sketch path
generate_arduino_firmware(target_name
    BOARD ${BOARD_ID} 
    SRCS ${SOURCES}
    HDRS ${HEADERS}
    LIBS ${LIBRARIES}
    PORT ${SERIAL_PORT}
    SERIAL ${SERIAL_COMMAND}
    PROGRAMMER ${PROGRAMMER_ID}
    AFLAGS ${AFLAGS})
