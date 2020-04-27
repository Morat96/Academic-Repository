cmake_minimum_required(VERSION 2.8)

project(PROJECT)
find_package(OpenCV REQUIRED)

# PROJECT
set(PCV
    cpp_code/main.cpp
    cpp_code/PlateDetector.cpp
    cpp_code/PlateDetector.h
)

set (CMAKE_CXX_STANDARD 11)
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/include )
link_directories( ${CMAKE_BINARY_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
add_executable(Project ${PCV})
target_link_libraries(Project ${OpenCV_LIBS} )
set(CMAKE_SUPPRESS_REGENERATION true)
