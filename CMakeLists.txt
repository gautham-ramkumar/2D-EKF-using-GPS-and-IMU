cmake_minimum_required(VERSION 3.16)
project(cpp_ekf LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)   # helps VS Code IntelliSense find Eigen

find_package(Eigen3 REQUIRED)

add_executable(ekf main.cpp src/data.cpp src/utm.cpp src/ekf2d.cpp src/output.cpp src/evaluation.cpp)
target_include_directories(ekf PRIVATE include)
target_link_libraries(ekf PRIVATE Eigen3::Eigen)