# Create a library for the kinematics module
add_library(kinematics FlightKinematics.cpp )

# Include headers from the global include directory
target_include_directories(kinematics PUBLIC
    ../../include/filters
    ../../include/kinematics)

# Because kinematics uses filters, it needs to be linked
target_link_libraries(kinematics PRIVATE filters)