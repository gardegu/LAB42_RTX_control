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
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(pinocchio REQUIRED)
find_package(ament_index_cpp REQUIRED)


add_library(armcmds src/drivers/armlib/armcmds.cpp)
target_include_directories(armcmds PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(armraw src/drivers/armlib/armraw.cpp)
target_include_directories(armraw PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

  add_library(armerror src/drivers/armlib/armerror.cpp)
target_include_directories(armerror PUBLIC
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
 geometry_msgs
 )

 target_link_libraries(nodeCamera "${cpp_typesupport_target}")
 target_include_directories(nodeCamera PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
 
##################################### Node Zed
add_executable(nodeZed
        src/ZED_M_drivers/usb_stereo_camera.cpp
        src/ZED_M_drivers/zed_capture_node.cpp
        )

ament_target_dependencies(nodeZed
        rclcpp
        cv_bridge
        image_transport
        camera_info_manager
        )

target_link_libraries(nodeZed "${cpp_typesupport_target}")
target_include_directories(nodeZed PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)

 ##################################### Node Arm
add_executable(nodeArm
 src/node_arm.cpp

 src/drivers/armlib/armcmds.cpp
 src/drivers/armlib/armraw.cpp
 src/drivers/armlib/armerror.cpp
 src/drivers/umi/umiinit.cpp
 src/drivers/umi/umicmds.cpp

 src/arm_parts/arm.cpp 
 src/arm_parts/forearm.cpp 
 src/arm_parts/joint.cpp 
 src/arm_parts/wrist.cpp
 )
 
 ament_target_dependencies(nodeArm
 rclcpp
 std_msgs
 sensor_msgs
 geometry_msgs

 )
 
 target_link_libraries(nodeArm armcmds armraw armerror)
 target_link_libraries(nodeArm "${cpp_typesupport_target}" )
 target_include_directories(nodeArm PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)

 ##################################### Node Inverse Kinematics
 add_executable(nodeInverseKinematics
 src/node_invkin.cpp
 )
 
 ament_target_dependencies(nodeInverseKinematics
 rclcpp
 geometry_msgs
 std_msgs
 sensor_msgs
 pinocchio
 ament_index_cpp
 )
 
 target_link_libraries(nodeInverseKinematics)
 target_link_libraries(nodeInverseKinematics "${cpp_typesupport_target}" )
 target_include_directories(nodeInverseKinematics PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)

 ##################################### Node Simulation
 add_executable(nodeSimu
 src/node_simu.cpp
 )
 
 ament_target_dependencies(nodeSimu
 rclcpp
 std_msgs
 sensor_msgs
 ament_index_cpp
 )
 
 target_link_libraries(nodeSimu)
 target_link_libraries(nodeSimu "${cpp_typesupport_target}" )
 target_include_directories(nodeSimu PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)




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

install(TARGETS nodeCamera nodeArm nodeInverseKinematics nodeZed nodeSimu
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include
  DESTINATION include)

install(DIRECTORY launch rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
