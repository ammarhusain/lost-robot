cmake_minimum_required(VERSION 2.6)

# Core functions of ParticleFilter library
file(GLOB PF_LIB_SOURCES	
	ParticleFilter.*
	MapData.*
	InputData.*
	)

add_library(ParticleFilterLibrary ${PF_LIB_SOURCES} )