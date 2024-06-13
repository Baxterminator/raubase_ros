include("cmake/to_snake_case.cmake")
include("cmake/generate_robot_files.cmake")

macro(make_robot_package rname)
  cmake_parse_arguments(ARG "" "URDF" "RESOURCES_DIR" ${ARGN})
  to_snake_case(${rname} pkg_name)
  message("Generating robot package: ${rname} (${pkg_name})")

  # Get source generation folder
  generate_folders(${pkg_name} source_folder build_directory)
  message("   -> Generated sources folder: ${source_folder}")

  # Generate building files
  generate_cmakelist(${pkg_name} ${source_folder})
  generate_package(${pkg_name} ${source_folder})

  # Copy resources files
  if (ARG_RESOURCES_DIR)
    install_resources(${pkg_name} ${ARG_RESOURCES_DIR})
  endif()

  # Launch to the CMake stack
  add_subdirectory(${source_folder} ${build_directory})
endmacro()