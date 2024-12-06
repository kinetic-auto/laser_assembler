cmake_minimum_required(VERSION 3.14.4)
project(laser_assembler)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(filters CONFIG REQUIRED)
find_package(pluginlib REQUIRED)

# For building service messages
find_package(rosidl_default_generators REQUIRED)

# Dependencies for python components
find_package(rclpy REQUIRED)
find_package(ament_cmake_python REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AssembleScans.srv"
  "srv/AssembleScans2.srv"
  DEPENDENCIES sensor_msgs
)

# add_executable(laser_scan_assembler_srv src/laser_scan_assembler_srv.cpp)
# ament_target_dependencies(laser_scan_assembler_srv 
#                           rclcpp 
#                           tf2 
#                           tf2_ros 
#                           sensor_msgs 
#                           laser_geometry 
#                           laser_assembler_interfaces 
#                           filters
#                           pluginlib)
# target_include_directories(laser_scan_assembler_srv PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_link_libraries(laser_scan_assembler_srv PUBLIC rclcpp::rclcpp)

# add_executable(laser_scan_assembler src/laser_scan_assembler.cpp)
# target_include_directories(laser_scan_assembler PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_link_libraries(laser_scan_assembler PUBLIC rclcpp::rclcpp)

# add_executable(merge_clouds src/merge_clouds.cpp)
# target_include_directories(merge_clouds PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_link_libraries(merge_clouds PUBLIC rclcpp::rclcpp)

# add_executable(point_cloud_assembler_srv src/point_cloud_assembler_srv.cpp)
# target_include_directories(point_cloud_assembler_srv PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_link_libraries(point_cloud_assembler_srv PUBLIC rclcpp::rclcpp)

# add_executable(point_cloud_assembler src/point_cloud_assembler.cpp)
# target_include_directories(point_cloud_assembler PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_link_libraries(point_cloud_assembler PUBLIC rclcpp::rclcpp)

add_executable(point_cloud2_assembler src/point_cloud2_assembler.cpp)
ament_target_dependencies(point_cloud2_assembler 
                          rclcpp 
                          tf2 
                          tf2_ros
                          tf2_sensor_msgs
                          sensor_msgs 
                          laser_geometry 
                          )
target_include_directories(point_cloud2_assembler PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(point_cloud2_assembler "${cpp_typesupport_target}")

# if (BUILD_TESTING)
#   find_package(rostest)
#   find_package(ament_cmake_gtest REQUIRED)

#   add_executable(periodic_snapshotter examples/periodic_snapshotter.cpp)
#   target_include_directories(periodic_snapshotter PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>)
#   target_link_libraries(periodic_snapshotter PUBLIC rclcpp::rclcpp)

#   add_executable(dummy_scan_producer test/dummy_scan_producer.cpp)
#   target_include_directories(dummy_scan_producer PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>)
#   target_link_libraries(dummy_scan_producer PUBLIC rclcpp::rclcpp)

#   add_executable(test_assembler test/test_assembler.cpp)
#   target_include_directories(test_assembler PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>)
#   target_link_libraries(test_assembler PUBLIC rclcpp::rclcpp)
# endif()

##############################################################################
# Install
##############################################################################

install(TARGETS
    # laser_scan_assembler_srv
    # laser_scan_assembler
    # merge_clouds
    # point_cloud_assembler_srv
    # point_cloud_assembler
    point_cloud2_assembler
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME}_py)
install(PROGRAMS
  scripts/client
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
