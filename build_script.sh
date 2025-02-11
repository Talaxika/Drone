#!/bin/bash

# Define build directory
BUILD_DIR="build"

# Check if build directory exists, if not, create it
if [ ! -d "$BUILD_DIR" ]; then
    echo "Creating build directory..."
    mkdir -p "$BUILD_DIR"
fi

# Navigate to build directory
cd "$BUILD_DIR"

# Run CMake configuration
echo "Configuring project with CMake..."
cmake ..

# Check if CMake configuration was successful
if [ $? -ne 0 ]; then
    echo "CMake configuration failed!"
    exit 1
fi

# Build the project
echo "Building the project..."
make -j$(nproc)

# Check if build was successful
if [ $? -ne 0 ]; then
    echo "Build failed!"
    exit 1
fi

# Run the compiled application
echo "Running the application..."
./SimulationApp
