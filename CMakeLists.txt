cmake_minimum_required(VERSION 2.8)

project(kalman)

#添加Eigen头文件
include_directories("/usr/include/eigen3")

if(NOT WIN32)
ADD_DEFINITIONS("-std=c++0x -O3")
endif(NOT WIN32)

include_directories(src) 
FILE(GLOB_RECURSE sourcefiles "src/*.cpp")

add_executable(kalman ${sourcefiles} )
