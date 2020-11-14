cmake_minimum_required(VERSION 3.10)

#Set project name
project(cmGNC)

#cpp standard
set(CMAKE_CXX_STANDARD 17)

#add the executable
add_executable(cmGNC src/mainGNC.cc)

message("-- PROJECT_SOURCE_DIR : " ${PROJECT_SOURCE_DIR})

#set source files
set(SOURCE_FILES
	${PROJECT_SOURCE_DIR}/src/example/example-include.h
)

#set include directories
include_directories("/src/example")