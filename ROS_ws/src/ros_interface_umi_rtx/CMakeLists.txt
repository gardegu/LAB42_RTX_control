cmake_minimum_required(VERSION 3.5)
project(ros_interface_umi_rtx)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

add_library(armcmds src/armlib/armcmds.cpp)
target_include_directories(armcmds PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


  ##################################### Node camera
 add_executable(nodeCamera
 src/node_camera.cpp
 )
 
 ament_target_dependencies(nodeCamera
 rclcpp
 sensor_msgs
 OpenCV
 cv_bridge
 )

 target_link_libraries(nodeCamera "${cpp_typesupport_target}")
 target_include_directories(nodeCamera PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
 

 ##################################### Node Arm
add_executable(nodeArm
 src/node_arm.cpp
 src/armlib/armcmds.cpp
 src/armlib/armraw.cpp
 src/armlib/armerror.c
 )
 
 ament_target_dependencies(nodeArm
 rclcpp
 std_msgs
 )
 
 target_link_libraries(nodeArm armcmds)
 target_link_libraries(nodeArm "${cpp_typesupport_target}" )
 target_include_directories(nodeArm PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
 
 #####################################



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS nodeCamera nodeArm
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
