cmake_minimum_required(VERSION 3.10)

#Set project name
project(cmGNC)
set(CMAKE_BUILD_TYPE Release)#Debug

#set(CMAKE_CXX_FLAGS "-g -O0 -std=c++17 ${CMAKE_CXX_FLAGS} -Wno-format-security -Wno-literal-suffix")

#add the executable
add_executable(cmGNC.run src/mainGNC.cc src/example/example-include.cc)

# Set the include directories
target_include_directories(cmGNC.run PUBLIC "${PROJECT_SOURCE_DIR}/src/example")

#cpp standard
set(CMAKE_CXX_STANDARD 17)

message("-- PROJECT_SOURCE_DIR : " ${PROJECT_SOURCE_DIR})

#target_link_libraries(cmGNC.run library1 library2)
#add_dependencies(cmGNC.run compiled_library_1 compiled_library_2) #Tells compiler if this executable relies on the output of another


#set include directories
include_directories("/src/example")
