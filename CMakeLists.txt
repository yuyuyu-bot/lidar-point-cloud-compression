cmake_minimum_required(VERSION 3.21)
project(clustering_based_pcc)

find_package(OpenCV REQUIRED)
if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV not found.")
endif()

set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -g -Wall -Wextra -Werror -O3 -std=c++20 -Wno-unused-function")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall -Wextra -Werror -O3 -std=c++20 -Wno-unused-function")
set(include_dirs ${OpenCV_INCLUDE_DIRS} include)
set(libraries ${OpenCV_LIBRARIES})


file(GLOB src src/*.cpp)
add_executable(${PROJECT_NAME} ${src})
target_include_directories(${PROJECT_NAME} PRIVATE ${include_dirs})
target_link_libraries(${PROJECT_NAME} PRIVATE ${libraries})
