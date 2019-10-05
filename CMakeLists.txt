cmake_minimum_required(VERSION 2.8.11)

project(image_warping_demo)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(CMAKE_AUTOMOC ON)
set(CMAKE_PREFIX_PATH "/usr/local/Cellar/qt/5.12.3/lib/cmake/Qt5Widgets/")
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Qt5Widgets REQUIRED)
find_package(OpenCV REQUIRED)

set(EIGEN_ROOT_DIR "/usr/local/include/eigen3/")

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN_ROOT_DIR})

add_subdirectory(image_warping)
add_subdirectory(image_warping_gui)
