cmake_minimum_required(VERSION 3.10)

project(Project4)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ompl REQUIRED)
include_directories(${OMPL_INCLUDE_DIRS})

# Include and link ODE (for OMPL's dependencies)
# include_directories(/opt/homebrew/Cellar/ode/0.16.6/include)
# link_directories(/opt/homebrew/Cellar/ode/0.16.6/lib)

include_directories(/opt/homebrew/include)
link_directories(/opt/homebrew/lib)

add_executable(manipulator manipulator.cpp RG-RRT-Kang.cpp)
target_link_libraries(manipulator ${OMPL_LIBRARIES})

# add_executable(car car.cpp RG-RRT.cpp CollisionChecking.cpp)
# target_link_libraries(car ${OMPL_LIBRARIES})

