cmake_minimum_required(VERSION 3.21)
project(lpcc)

find_package(OpenCV REQUIRED)
if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV not found.")
endif()

set(FLAGS "-g -Wall -Wextra -Weverything -Werror -O3 -std=c++20")
set(FLAGS "${FLAGS} -Wno-unused-function -Wno-padded -Wno-unused-member-function -Wno-missing-prototypes -Wno-unused-template")
set(FLAGS "${FLAGS} -Wno-c++98-compat -Wno-c++98-compat-extra-semi -Wno-pre-c++20-compat-pedantic -Wno-c++98-compat-pedantic")

set(CMAKE_C_COMPILER   clang)
set(CMAKE_CXX_COMPILER clang++)
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} ${FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${FLAGS}")
set(include_dirs ${OpenCV_INCLUDE_DIRS} include)
set(libraries ${OpenCV_LIBRARIES})


file(GLOB src src/*.cpp)
add_executable(${PROJECT_NAME} ${src})
target_include_directories(${PROJECT_NAME} PRIVATE ${include_dirs})
target_link_libraries(${PROJECT_NAME} PRIVATE ${libraries})
