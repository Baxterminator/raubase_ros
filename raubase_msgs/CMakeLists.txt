cmake_minimum_required(VERSION 3.8)
project(raubase_msgs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# =================================================================
#                  Raspberry PI Compilation arguments
# =================================================================
execute_process(COMMAND uname -m RESULT_VARIABLE IS_OK OUTPUT_VARIABLE CPU1)
string(STRIP ${CPU1} CPU)
# works for Raspberry 3 and 4
if (${CPU} MATCHES "armv7l" OR ${CPU} MATCHES "aarch64")
   message("# Is a RASPBERRY; CPU=${CPU} (Pi3=armv7l, pi4=aarch64)")
   #    set(EXTRA_CC_FLAGS " -mfpu=vfp -mfloat-abi=hard -march=armv6zk -mtune=arm1176jzf-s -DRASPBERRY_PI -D${CPU}")
   set(EXTRA_CC_FLAGS "-D${CPU} -O2 -g0 -DRASPBERRY_PI -I/home/local/git/CLI11/include")
   #set(EXTRA_CC_FLAGS "-D${CPU} -O0 -g2 -DRASPBERRY_PI")
else()
   message("# Not a RASPBERRY; CPU=${CPU}")
   set(EXTRA_CC_FLAGS "-D${CPU} -O0 -g2")
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic \
    -Wno-format-truncation -Wno-return-type \
    -std=c++20 ${EXTRA_CC_FLAGS}")
set(CMAKE_C_FLAGS ${CMAKE_C_FLAGS} "-pthread")

# =================================================================
#                           INTERFACE
# =================================================================
file(GLOB msg_files RELATIVE "${CMAKE_CURRENT_LIST_DIR}"
        "${CMAKE_CURRENT_LIST_DIR}/msg/*.msg"
        "${CMAKE_CURRENT_LIST_DIR}/srv/*.srv"
        "${CMAKE_CURRENT_LIST_DIR}/action/*.action"
)

if (msg_files)
  message("${msg_files}")
  rosidl_generate_interfaces(${PROJECT_NAME}
          ${msg_files}
          DEPENDENCIES std_msgs sensor_msgs)
  ament_export_dependencies(rosidl_default_runtime)

  if(NOT ${LEGACY_IDL})
      rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
  endif()
endif()

ament_auto_package()