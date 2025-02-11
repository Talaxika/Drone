# Create a library for the peripherals module
add_library(motors
    ESC.cpp
    Motor.cpp
    MotorController.cpp)

# Include headers from the global include directory
target_include_directories(motors PUBLIC
    ../../include/motors
    ../../include/peripherals
    ../../include/power)

target_link_libraries(motors PRIVATE power)
