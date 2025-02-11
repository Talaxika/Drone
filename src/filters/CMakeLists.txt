# Create a library for the filters module
add_library(filters KalmanFilter.cpp )

# Include headers from the global include directory
target_include_directories(filters PUBLIC ../../include/filters)
