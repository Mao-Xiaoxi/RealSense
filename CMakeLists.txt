cmake_minimum_required(VERSION 3.15)
project(MyTest LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(REALSENSE_ROOT "/Users/maoxiaoxi/Documents/Packages/librealsense")
set(REALSENSE_INCLUDE_DIR "${REALSENSE_ROOT}/include")
set(REALSENSE_BUILD_DIR "${REALSENSE_ROOT}/build/Release")

set(FLTK_DIR "/Users/maoxiaoxi/Documents/Packages/fltk/build")

find_package(OpenCV REQUIRED)
find_package(FLTK REQUIRED)

find_library(REALSENSE_LIB NAMES realsense2
    PATHS "${REALSENSE_BUILD_DIR}"
    NO_DEFAULT_PATH
)

add_library(auxiliary STATIC auxiliary.cpp)

target_include_directories(auxiliary PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${REALSENSE_INCLUDE_DIR}"
)

target_link_libraries(auxiliary PRIVATE
    "${REALSENSE_LIB}"
)

add_executable(myMeasure myMeasure.cpp)

target_include_directories(myMeasure PRIVATE
    "${REALSENSE_INCLUDE_DIR}"
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(myMeasure PRIVATE
    auxiliary
    "${REALSENSE_LIB}"
    ${OpenCV_LIBS}
    fltk::fltk
    fltk::gl
)

set_target_properties(myMeasure PROPERTIES
    BUILD_RPATH "${REALSENSE_BUILD_DIR}"
    INSTALL_RPATH "${REALSENSE_BUILD_DIR}"
)

add_executable(myTest myTest.cpp)

target_include_directories(myTest PRIVATE
    "${REALSENSE_INCLUDE_DIR}"
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(myTest PRIVATE
    "${REALSENSE_LIB}"
    ${OpenCV_LIBS}
)

set_target_properties(myTest PROPERTIES
    BUILD_RPATH "${REALSENSE_BUILD_DIR}"
    INSTALL_RPATH "${REALSENSE_BUILD_DIR}"
)

add_executable(myAlign myAlign.cpp)

target_include_directories(myAlign PRIVATE
    "${REALSENSE_INCLUDE_DIR}"
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(myAlign PRIVATE
    "${REALSENSE_LIB}"
    ${OpenCV_LIBS}
)

set_target_properties(myAlign PROPERTIES
    BUILD_RPATH "${REALSENSE_BUILD_DIR}"
    INSTALL_RPATH "${REALSENSE_BUILD_DIR}"
)