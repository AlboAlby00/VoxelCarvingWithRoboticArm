Voxel-Carving
=============

Straight forward implementation of a 3d reconstruction technique called voxel carving (or space carving)

Codebase Requirements

This codebase requires the following libraries and packages to be installed:

C++ Compiler (supporting C++11 or later)
OpenCV (version 2 or later)
yaml-cpp (YAML library for C++)
VTK (Visualization Toolkit)
Installation Instructions

1. C++ Compiler
Ensure that you have a C++ compiler installed on your system. This codebase requires a compiler that supports C++11 or later.

2. OpenCV
Install OpenCV on your system. You can follow the instructions specific to your operating system from the official OpenCV documentation: <https://opencv.org>

3. yaml-cpp
Install the yaml-cpp library for C++. Here are some general instructions:

Linux: You can install yaml-cpp using the package manager of your distribution. For example, on Ubuntu, you can run the following command:
`sudo apt-get install libyaml-cpp-dev`

macOS (Homebrew): If you're using Homebrew, you can install yaml-cpp by running the following command:

`brew install yaml-cpp`

Windows: For Windows, you can download the precompiled binaries of yaml-cpp from the official GitHub repository: <https://github.com/jbeder/yaml-cpp>

4. VTK
Install VTK (Visualization Toolkit) on your system. You can follow the instructions specific to your operating system from the official VTK documentation: <https://vtk.org>

Compiling and Running the Code

Once you have installed the required libraries, you can compile and run the codebase using your preferred build system or IDE.


```
mkdir build && cd build
cmake ..
make -j$(nproc)
./VoxelCarving <path/to/config>
```