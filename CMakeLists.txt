project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11  -g)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/main.cpp src/helpers.cpp src/cost_functions.cpp)
set(testptg src/testPTG.cpp src/helpers.cpp src/cost_functions.cpp)
include_directories(src/Eigen-3.3/Eigen)


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")


add_executable(path_planning ${sources})
add_executable(testptg ${testptg})

target_link_libraries(path_planning z ssl uv uWS)
