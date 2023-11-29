cmake_minimum_required(VERSION 3.5.1)

project(pyaruconavi)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
MESSAGE(" *** OpenCV_INCLUDE_DIRS : " ${OpenCV_INCLUDE_DIRS})
MESSAGE(" *** OpenCV_LIB_DIRS : " ${OpenCV_LIB_DIRS})
MESSAGE(" *** OpenCV_LIBS : " ${OpenCV_LIBS})

# specify your pybind11 location
# set(pybind11_DIR "/home/william/anaconda3/lib/python3.11/site-packages/pybind11/share/cmake/pybind11")
execute_process(
        COMMAND conda activate base
        COMMAND python ${CMAKE_CURRENT_LIST_DIR}/get_pybind11_cmake.py
        OUTPUT_VARIABLE PYTHON_OUTPUT
)
message(STATUS "pybind11 cmake dir is ${PYTHON_OUTPUT}")
string(STRIP ${PYTHON_OUTPUT} PYTHON_OUTPUT)
set(pybind11_DIR ${PYTHON_OUTPUT})
message(STATUS "pybind11_DIR ${pybind11_DIR}")
find_package(pybind11 CONFIG REQUIRED)
MESSAGE(" *** PYTHON_INCLUDE_DIRS : " ${PYTHON_INCLUDE_DIRS})
MESSAGE(" *** PYTHON_LIBRARIES : " ${PYTHON_LIBRARIES})

set(NUMPY_INCLUDE_DIR "" CACHE FILEPATH "Path to numpy header if cmake can't find them.")
if (NOT ${NUMPY_INCLUDE_DIR} STREQUAL "")
    message(" *** NUMPY_INCLUDE_DIR : ${NUMPY_INCLUDE_DIR}")
    if (NOT EXISTS ${NUMPY_INCLUDE_DIR}/numpy/ndarrayobject.h)
        message(SEND_ERROR "Can't find numpy/ndarrayobject.h in ${NUMPY_INCLUDE_DIR}")
    endif ()
    include_directories(${NUMPY_INCLUDE_DIR})
endif ()

include_directories(${pybind11_INCLUDE_DIR})
include_directories(${PYTHON_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS}/opencv4)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

SET(SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/export_apis.cpp
)

if (CMAKE_SYSTEM_NAME STREQUAL "Windows")
    add_compile_options(/std:c++17 /Oy /utf-8)
else ()
    add_compile_options(-std=c++17 -fPIC -O3)
endif ()

LINK_DIRECTORIES(
        ${OpenCV_LIB_DIR}
)

pybind11_add_module(pyaruconavi MODULE ${SOURCES})
target_link_libraries(pyaruconavi PRIVATE ${OpenCV_LIBS})