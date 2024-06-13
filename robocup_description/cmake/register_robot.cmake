include("cmake/to_snake_case.cmake")
include("cmake/generate_robot_files.cmake")

macro(register_robot rname)
  to_snake_case(${rname} pkg_name)
  message("Generating robot: ${rname} (${pkg_name})")

  # Get source generation folder
  generate_folders(${pkg_name} source_folder build_directory)
  message("\t-> Generated sources folder: ${source_folder}")

  # Generate files
  generate_cmakelist(${pkg_name} ${source_folder})
  generate_package(${pkg_name} ${source_folder})

  add_subdirectory(${source_folder} ${build_directory})

endmacro()