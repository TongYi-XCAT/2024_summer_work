cmake_minimum_required(VERSION 3.10)

project(vid)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
# set(CMAKE_C_COMPILER gcc)
# set(CMAKE_CXX_COMPILER g++)
# set(COMPILE_FLAGS -g)

find_package(OpenCV REQUIRED)

INCLUDE_DIRECTORIES(./inc ${OpenCV_INCLUDE_DIRS} "/usr/include/eigen3")

set(STC_LIST ./src/main.cpp ./src/mark.cpp ./src/number_classifier.cpp ./src/kalman.cpp)
add_executable(vid ${STC_LIST})
target_link_libraries(vid ${OpenCV_LIBS})