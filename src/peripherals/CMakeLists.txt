# Create a library for the peripherals module
add_library(peripherals DigitalPin.cpp )

# Include headers from the global include directory
target_include_directories(peripherals PUBLIC ../../include/peripherals)
