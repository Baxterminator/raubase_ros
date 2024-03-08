cmake_minimum_required(VERSION 3.8)
project(raubase_ros)

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

macro(link_raspberry_libs node_name)
  if (${CPU} MATCHES "armv7l" OR ${CPU} MATCHES "aarch64")
    target_link_libraries(${node_name} ${CMAKE_THREAD_LIBS_INIT} ${OpenCV_LIBS} readline gpiod rt)
  elseif(${CPU} MATCHES "x86_64")
    target_link_libraries(${node_name} ${CMAKE_THREAD_LIBS_INIT} ${OpenCV_LIBS})
  else()
    target_link_libraries(${node_name} ${CMAKE_THREAD_LIBS_INIT} ${OpenCV_LIBS} readline gpiod)
  endif()
endmacro()

# =================================================================
#                             ROS Nodes
# =================================================================
########################## Common Lib #############################
file(GLOB common_src RELATIVE "${CMALE_CURRENT_LIST_DIR}" "${CMAKE_CURRENT_LIST_DIR}/src/common/*.cpp" "${CMAKE_CURRENT_LIST_DIR}/src/common/**/**.cpp")
if (common_src)
  ament_auto_add_library(common ${common_src})
  target_include_directories(common PRIVATE include)
  link_raspberry_libs(common)
endif()

############################ Camera ###############################
file(GLOB cam_src RELATIVE "${CMALE_CURRENT_LIST_DIR}" "${CMAKE_CURRENT_LIST_DIR}/src/camera/*.cpp" "${CMAKE_CURRENT_LIST_DIR}/src/camera/**/**.cpp")
if (cam_src)
  ament_auto_add_executable(camera ${cam_src})
  target_include_directories(camera PRIVATE include)
  link_raspberry_libs(camera)
endif()

########################## Controller #############################
file(GLOB controller_src RELATIVE "${CMALE_CURRENT_LIST_DIR}" "${CMAKE_CURRENT_LIST_DIR}/src/controller/*.cpp" "${CMAKE_CURRENT_LIST_DIR}/src/controller/**/**.cpp")
if (controller_src)
  ament_auto_add_executable(controller ${controller_src})
  target_include_directories(controller PRIVATE include)
  link_raspberry_libs(controller)
endif()

########################## Localization ###########################
file(GLOB localization_src RELATIVE "${CMALE_CURRENT_LIST_DIR}" "${CMAKE_CURRENT_LIST_DIR}/src/localization/*.cpp" "${CMAKE_CURRENT_LIST_DIR}/src/localization/**/**.cpp")
if (localization_src)
  ament_auto_add_executable(localization ${localization_src})
  target_include_directories(localization PRIVATE include)
  link_raspberry_libs(localization)
endif()

############################ Teensy ###############################
file(GLOB teensy_src RELATIVE "${CMALE_CURRENT_LIST_DIR}" "${CMAKE_CURRENT_LIST_DIR}/src/teensy/*.cpp" "${CMAKE_CURRENT_LIST_DIR}/src/teensy/**/**.cpp")
if (teensy_src)
  ament_auto_add_executable(teensy ${teensy_src})
  target_link_libraries(teensy common)
  target_include_directories(teensy PRIVATE include)
  link_raspberry_libs(teensy)
endif()

############################ Test #################################
file(GLOB test_src RELATIVE "${CMALE_CURRENT_LIST_DIR}" "${CMAKE_CURRENT_LIST_DIR}/src/test/*.cpp" "${CMAKE_CURRENT_LIST_DIR}/src/test/**/**.cpp")
if (test_src)
  ament_auto_add_executable(testing ${test_src})
  target_include_directories(testing PRIVATE include)
  link_raspberry_libs(testing)
endif()

############################ Python ###############################
ament_python_install_package(${PROJECT_NAME})
file(GLOB py_scripts RELATIVE "${CMALE_CURRENT_LIST_DIR}" "${CMAKE_CURRENT_LIST_DIR}/scripts/*")
if (py_scripts)
  message("${py_scripts}")
  install(PROGRAMS
    ${py_scripts}
    DESTINATION lib/${PROJECT_NAME}
    )
endif()

######################### Launch Files ############################
install(DIRECTORY 
        resources/default
        resources/launch
        DESTINATION share/${PROJECT_NAME})


ament_auto_package()