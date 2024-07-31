#!/bin/bash

# Run the Python script
python3 scripts/clang-tools/gen_compile_commands.py

# Check if the Python script ran successfully
if [ $? -ne 0 ]; then
    echo "Failed to generate compile commands."
    exit 1
fi

# Run the make command
make ARCH=arm64 O=build_4b LLVM=-18 rust-analyzer

# Check if the make command ran successfully
if [ $? -ne 0 ]; then
    echo "Make command failed."
    exit 1
fi

echo "Commands executed successfully."
