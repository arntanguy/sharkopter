cmake_minimum_required(VERSION 2.8)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/arduino-cmake/cmake/ArduinoToolchain.cmake)


set(PROJECT_NAME nanocopter)
project(PROJECT_NAME)


#set(${FIRMWARE_NAME}_PORT /path/to/device)

# ===============================
# Board config
# ===============================
# Board id (such as uno, mega2560, pro5v328...)
set(BOARD_ID uno)
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
set(SOURCES src/nanocopter.cpp)
set(HEADERS)
set(LIBRARIES I2Cdev MPU6050 PID ESP8266 SoftwareSerial)

# =============================
# Libraries
# =============================

include_directories(/usr/share/arduino/libraries/Wire)
set(Wire_RECURSE True)

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/libraries)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libraries/i2cdevlib/Arduino/I2Cdev)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libraries/i2cdevlib/Arduino/MPU6050)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libraries/ITEADLIB_Arduino_WeeESP8266)
include_directories(/usr/share/arduino/libraries)

generate_arduino_library(I2Cdev
    BOARD ${BOARD_ID}
    SRCS libraries/i2cdevlib/Arduino/I2Cdev/I2Cdev.cpp)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libraries)
file(GLOB MPU6050_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/libraries/i2cdevlib/Arduino/MPU6050/*.cpp)
generate_arduino_library(MPU6050
    BOARD ${BOARD_ID}
    SRCS ${MPU6050_SRCS}
    LIBS I2Cdev
    )

generate_arduino_library(PID
    BOARD ${BOARD_ID}
    SRCS libraries/PID_v1/PID_v1.cpp 
    LIBS I2Cdev
    )

  generate_arduino_library(ESP8266
    BOARD ${BOARD_ID}
    SRCS libraries/ITEADLIB_Arduino_WeeESP8266/ESP8266.cpp)

  generate_arduino_library(SoftwareSerial
    BOARD ${BOARD_ID}
    SRCS /usr/share/arduino/libraries/SoftwareSerial/SoftwareSerial.cpp)

## If a library contains nested source, a special option must be set to enable recursion
## The option name should be ${LIBRARY_NAME}_RECURSE, where in this case LIBRARY_NAME is equal to Wire.
#set(Wire_RECURSE True)
#generate_arduino_library(name
#    BOARD ${BOARD_ID}
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
generate_arduino_firmware(nanocopter
    BOARD ${BOARD_ID} 
    SRCS ${SOURCES}
    HDRS ${HEADERS}
    LIBS ${LIBRARIES}
    PORT ${SERIAL_PORT}
    SERIAL ${SERIAL_COMMAND}
    PROGRAMMER ${PROGRAMMER_ID}
    AFLAGS ${AFLAGS})

generate_arduino_firmware(wifi
    BOARD ${BOARD_ID} 
    SRCS src/wifi_get.cpp 
    HDRS ${HEADERS}
    LIBS ${LIBRARIES}
    PORT ${SERIAL_PORT}
    SERIAL ${SERIAL_COMMAND}
    PROGRAMMER ${PROGRAMMER_ID}
    AFLAGS ${AFLAGS})

generate_arduino_firmware(mpu_calibration
    BOARD ${BOARD_ID} 
    SRCS src/mpu6050_raw_calib.cpp
    HDRS ${HEADERS}
    LIBS ${LIBRARIES}
    PORT ${SERIAL_PORT}
    SERIAL ${SERIAL_COMMAND}
    PROGRAMMER ${PROGRAMMER_ID}
    AFLAGS ${AFLAGS})
