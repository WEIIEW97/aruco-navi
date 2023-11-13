cmake_minimum_required(VERSION 3.26)
project(aruco_navi)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

file(GLOB sources ${CMAKE_CURRENT_LIST_DIR}/src/*.cpp)
file(GLOB headers ${CMAKE_CURRENT_LIST_DIR}/src/*.h)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(realsense2 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)


add_executable(${PROJECT_NAME} 
        main.cpp
        ${sources}
)

target_include_directories(
        ${PROJECT_NAME}
        PRIVATE
        ${realsense2_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${OPENGL_INCLUDE_DIRS}
)

target_link_libraries(
        ${PROJECT_NAME}
        ${realsense2_LIBRARY}
        ${OpenCV_LIBS}
        ${OPENGL_LIBRARIES}
        glfw
)
