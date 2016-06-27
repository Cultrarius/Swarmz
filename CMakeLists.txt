cmake_minimum_required(VERSION 3.5)
project(Swarmz)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES main.cpp swarmz.h)
add_executable(Swarmz ${SOURCE_FILES})