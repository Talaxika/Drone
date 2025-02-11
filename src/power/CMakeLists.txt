# Create a library for the power module
add_library(power
    Battery.cpp
    EnergyConsumer.cpp
    PowerDistributionBoard.cpp )

# Include headers from the global include directory
target_include_directories(power PUBLIC
    ../../include/power
    ../../include/util)
