cmake_minimum_required(VERSION 3.10)
project(project_template)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-march=native -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")

# LibTorch
set(CAFFE2_USE_CUDNN 1)
list(APPEND CMAKE_PREFIX_PATH ../libtorch-2.1.1)
find_package(Torch REQUIRED)
include_directories(${TORCH_INCLUDE_DIRS})
link_libraries(${TORCH_LIBRARIES})

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_libraries(${OpenCV_LIBS})

add_executable(a.out cpp/main.cpp)
