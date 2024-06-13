cmake_minimum_required(VERSION 3.8)
project(dtu_robocup_description)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) 

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include("cmake/register_robot.cmake")

message("========================= CMake ===========================")

register_robot(Robobot-9)

ament_auto_package()
message("========================== Make ==========================")