cmake_minimum_required(VERSION 3.8)
project(dtu_robocup_description)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) 

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include("cmake/register_robot.cmake")

message("========================= CMake ===========================")

# Add robots
file(GLOB robot_list RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/robot/" "robot/**" )
foreach(robot ${robot_list})
  if (IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/robot/${robot}")
    make_robot_package(${robot} RESOURCES_DIR "robot/${robot}")
  endif()
endforeach()

install(PROGRAMS
      dummy_joint_state.py
      DESTINATION lib/${PROJECT_NAME}
    )

message("========================== Make ==========================")
ament_auto_package()