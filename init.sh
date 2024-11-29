#!/usr/bin/env bash

# Function to display help
usage() {
    echo "Usage: $0 [-s] [-h]"
    echo "  -s    Enable secure signing support (includes recursive submodule update)."
    echo "  -h    Show this help message."
    exit 1
}

# Default variables
SECURE_SIGNING=false

# Parse options
while getopts "sh" option; do
    case $option in
        s) SECURE_SIGNING=true ;;
        h) usage ;;
        *) usage ;;
    esac
done

# Adds target via rustup
rustup target add thumbv8m.main-none-eabihf

# Installs Clippy
rustup component add clippy

# Cloning and building picotool
echo "Cloning and building picotool..."

if [ "$SECURE_SIGNING" = true ]; then
    echo "Secure signing enabled"
    git submodule update --init --recursive
else
    echo "Secure signing not enabled"p
    git submodule update --init 
fi

# Build process
cd picotool || { echo "Failed to enter picotool directory"; exit 1; }
mkdir -p build
cd build || { echo "Failed to enter build directory"; exit 1; }

cmake .. -DPICO_SDK_PATH="../../pico-sdk"
make

echo "Picotool loader build complete!"