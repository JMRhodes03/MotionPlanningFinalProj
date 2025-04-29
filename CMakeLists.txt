cmake_minimum_required(VERSION 3.10)

project(Project4)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ompl REQUIRED)
include_directories(${OMPL_INCLUDE_DIRS})

include_directories(/opt/homebrew/include)
include_directories(/opt/homebrew/include/eigen3)
link_directories(/opt/homebrew/lib)

add_executable(manipulator manipulator.cpp RG-RRT-Kang.cpp)
target_link_libraries(manipulator ${OMPL_LIBRARIES})
