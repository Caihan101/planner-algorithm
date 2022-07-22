cmake_minimum_required(VERSION 3.0.2)
project(my_rrt)

add_definitions(-std=c++11 -w)
set(CMAKE_CXX_FLAGS " -Wall -Wextra ${CMAKE_CXX_FLAGS}")
set(CMAKE_BUILD_TYPE "Release")

find_package(Eigen3 REQUIRED)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIR}
)


add_library(${PROJECT_NAME}
        src/Astar.cpp
        src/rrt.cpp
        src/rrtStar.cpp
        src/KDTree.cpp
        src/biRRTStar.cpp
        src/corridor.cpp
        src/smoothosqpproblem.cpp
        )

add_executable(${PROJECT_NAME}_demo src/main.cpp)

target_link_libraries(${PROJECT_NAME}_demo  ${PROJECT_NAME} /usr/local/lib/libosqp.so)
