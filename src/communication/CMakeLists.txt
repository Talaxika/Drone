# Create a library for the communication module
add_library(communication
    I2C.cpp
    I2CDevice.cpp)

# Include headers from the global include directory
target_include_directories(communication PUBLIC
    ../../include/communication
    ../../include/util)
