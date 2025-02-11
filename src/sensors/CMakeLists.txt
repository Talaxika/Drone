# Create a library for the sensors module
add_library(sensors
    IMUSensor.cpp
    TempHumiditySensor.cpp)

# Include headers from the global include directory
target_include_directories(sensors PUBLIC
    ../../include/sensors
    ../../include/communication
    ../../include/power)

target_link_libraries(sensors PRIVATE power)