cmake_minimum_required(VERSION 3.8)
project(dtu_robocup_description)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) 

find_package(ament_cmake_auto REQUIRED)
find_package(arcana_cmake REQUIRED)
ament_auto_find_build_dependencies()

include("cmake/register_robot.cmake")

message("========================= CMake ===========================")

fetch_system_info(DISPLAY)

# Add robots
file(GLOB robot_list RELATIVE "${CMAKE_CURRENT_LIST_DIR}/robot/" "robot/**" )
foreach(robot ${robot_list})
  if (IS_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/robot/${robot}")
    make_robot_package(${robot} RESOURCES_DIR "robot/${robot}")
  endif()
endforeach()

# Add assets
install(
  DIRECTORY assets models
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  dummy_joint_state.py
  DESTINATION lib/${PROJECT_NAME}
)

setup_project_watcher(DISPLAY)
ament_auto_package()

# Add maps
install_xacro_files(FILES 
  "maps/robocup-25.sdf.xacro"
)

message("========================== Make ==========================")
