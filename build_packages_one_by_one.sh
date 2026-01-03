#!/bin/bash
# Build all packages one at a time to avoid Jetson crashes

cd /workspaces/isaac_ros-dev

# Get all package names
packages=$(colcon list --names-only)

# Build each package sequentially
for package in $packages; do
    echo "=========================================="
    echo "Building: $package"
    echo "=========================================="
    colcon build --packages-select "$package" --symlink-install --parallel-workers 1
    
    if [ $? -ne 0 ]; then
        echo "ERROR: Failed to build $package"
        echo "Continue? (y/n)"
        read -r response
        if [ "$response" != "y" ]; then
            exit 1
        fi
    fi
done

echo "All packages built successfully!"

