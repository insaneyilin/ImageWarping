project(image_warping)

file(GLOB _SRCS *.cpp)

add_library(${PROJECT_NAME} ${_SRCS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
