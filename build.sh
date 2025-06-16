#!/bin/bash

# WSN Deployment Build Script
# This script builds all components of the WSN deployment system

echo "Building WSN Deployment System..."
echo "=================================="

# Change to source directory
cd src/

echo "Cleaning previous builds..."
make clean

echo ""
echo "Building Base Station..."
make base-station TARGET=cooja
if [ $? -eq 0 ]; then
    echo "✓ Base Station built successfully"
else
    echo "✗ Base Station build failed"
    exit 1
fi

echo ""
echo "Building Mobile Robot..."
make mobile-robot TARGET=cooja
if [ $? -eq 0 ]; then
    echo "✓ Mobile Robot built successfully"
else
    echo "✗ Mobile Robot build failed"
    exit 1
fi

echo ""
echo "Building Sensor Node..."
make sensor-node TARGET=cooja
if [ $? -eq 0 ]; then
    echo "✓ Sensor Node built successfully"
else
    echo "✗ Sensor Node build failed"
    exit 1
fi

echo ""
echo "All components built successfully!"
echo ""
echo "To run the simulation:"
echo "1. Open Cooja simulator"
echo "2. Load the simulation file: src/wsn-deployment-simulation.csc"
echo "3. Start the simulation"
echo ""
echo "Build artifacts are located in src/build/"

