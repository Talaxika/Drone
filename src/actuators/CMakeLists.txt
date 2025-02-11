# Create a library for the sensors module
add_library(actuators
    LEDBoard.cpp)

# Include headers from the global include directory
target_include_directories(actuators PUBLIC
    ../../include/actuators
    ../../include/communication
    ../../include/power)

target_link_libraries(actuators PRIVATE power)