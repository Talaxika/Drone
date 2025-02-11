cmake_minimum_required(VERSION 3.10)
project(AI_Fault_Detection_Simulation)

set(CMAKE_CXX_STANDARD 17)

# Add source directories
add_subdirectory(src)
# add_subdirectory(tests)

# Define executable
add_executable(SimulationApp src/main.cpp)

target_include_directories(SimulationApp PRIVATE include)

target_link_libraries(SimulationApp PRIVATE
    peripherals
    motors
    sensors
    communication
    filters
    kinematics
    power
    actuators)

# Enable testing
# enable_testing()
