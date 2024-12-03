cmake_minimum_required(VERSION 3.20)

# 设置 depthai-core 路径变量，支持用户自定义路径
set(DEPTHAI_CORE_PATH "${CMAKE_CURRENT_LIST_DIR}/../depthai-core" CACHE PATH "Path to depthai-core directory")

# 验证路径是否有效
if(EXISTS "${DEPTHAI_CORE_PATH}/CMakeLists.txt")
    message(STATUS "Including depthai-core from ${DEPTHAI_CORE_PATH}")
    add_subdirectory("${DEPTHAI_CORE_PATH}" depthai-core EXCLUDE_FROM_ALL)
else()
    message(FATAL_ERROR "depthai-core path is invalid or CMakeLists.txt not found: ${DEPTHAI_CORE_PATH}")
endif()

project(quadcam-driver)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Set the source files for your project
set(SOURCES
    src/main.cpp
)

# Create an executable target
add_executable(quadcam-driver ${SOURCES})

# Link against OpenCV libraries
target_link_libraries(quadcam-driver PRIVATE depthai::opencv ${OpenCV_LIBS})

# Set include directories
include_directories(${OpenCV_INCLUDE_DIRS})
