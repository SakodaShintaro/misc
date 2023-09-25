cmake_minimum_required(VERSION 3.10)
project(project_template)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-march=native -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")

add_executable(a.out cpp/main.cpp)
