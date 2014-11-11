#  Copyright (c) 2014: Ammar Husain

cmake_minimum_required (VERSION 2.6)

project ("ParticleFilter")

set(PARTICLE_FILTER_BASE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# opencv is required to run this package
find_package(OpenCV REQUIRED)

# add the required classes
###include_directories(src)
add_subdirectory(src)


add_executable(ParticleFilter main.cpp)

target_link_libraries(ParticleFilter 
  ParticleFilterLibrary
  ${OpenCV_LIBS}
  )



