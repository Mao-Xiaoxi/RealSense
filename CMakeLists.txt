cmake_minimum_required(VERSION 3.15)
project(MyTest LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(REALSENSE_ROOT "/Users/maoxiaoxi/Documents/Packages/librealsense")
set(REALSENSE_INCLUDE_DIR "${REALSENSE_ROOT}/include")
set(REALSENSE_BUILD_DIR "${REALSENSE_ROOT}/build/Release")

find_package(OpenCV REQUIRED)

find_library(REALSENSE_LIB NAMES realsense2
    PATHS "${REALSENSE_BUILD_DIR}"
    NO_DEFAULT_PATH
)

file(GLOB SOURCES "*.cpp")

foreach(SOURCE_FILE ${SOURCES})
    get_filename_component(EXEC_NAME ${SOURCE_FILE} NAME_WE)

    add_executable(${EXEC_NAME} ${SOURCE_FILE})

    target_include_directories(${EXEC_NAME} PRIVATE
        "${REALSENSE_INCLUDE_DIR}"
        ${OpenCV_INCLUDE_DIRS}
    )

    target_link_libraries(${EXEC_NAME} PRIVATE
        "${REALSENSE_LIB}"
        ${OpenCV_LIBS}
    )

    set_target_properties(${EXEC_NAME} PROPERTIES
        BUILD_RPATH "${REALSENSE_BUILD_DIR}"
        INSTALL_RPATH "${REALSENSE_BUILD_DIR}"
    )
endforeach()