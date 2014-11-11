# Example demostrating the working of a Particle Filter to localize a lost robot in a known environment

Apple Coding Test
Author: Ammar Husain

## Compile
To compile use CMake: http://www.cmake.org

Prerequisite
- OpenCV: http://www.opencv.org

## How to make
I have built and tested this package on Ubuntu 12.04 with OpenCV 2.4.9
On Linux
	mkdir build
	cd build
	cmake ..
	make 

## Use
 USAGE: ./ParticleFilter logname (for default: use -d) [optional] -w write-path (for writing images to disk) 

## Dataset
CMU Wean Hall: A wheeled robot driving around the corridors of Wean Hall 
Obtained from Statistical Techniques in Robotics course (16-831) at CMU

## References
Probabilistic Robotics: Wolfram Burgard, Dieter Fox, and Sebastian Thrun

## Troubleshooting
If you get linker errors to OpenCV - make sure you add the appropriate lib and include paths to CMakeLists
It currently assumes an OpenCV installation in /usr/local
