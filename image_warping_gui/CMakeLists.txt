set(APP_NAME image_warping_demo)

qt5_wrap_cpp(MAIN_WINDOW_SRCS
    mainwindow.h
    image_widget.h)
qt5_wrap_ui(MAIN_WINDOW_UI
    mainwindow.ui)
qt5_add_resources(MAIN_WINDOW_RCCS mainwindow.qrc)

file(GLOB _SRCS *.cpp)

add_executable(${APP_NAME}
    ${_SRCS}
    ${MAIN_WINDOW_RCCS}
    ${MAIN_WINDOW_UI}
    ${MAIN_WINDOW_SRCS})

target_link_libraries(${APP_NAME}
    Qt5::Widgets
    ${OpenCV_LIBS}
    image_warping)
