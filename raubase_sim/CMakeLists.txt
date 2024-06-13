cmake_minimum_required(VERSION 3.8)
project(raubase_sim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) 

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

message("========================= CMake ===========================")


ament_auto_package()
message("========================== Make ==========================")