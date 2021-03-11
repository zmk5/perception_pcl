cmake_minimum_required(VERSION 3.5)
project(pcl_ros2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find system dependencies
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED QUIET COMPONENTS core features filters io segmentation surface)

## Find ROS package dependencies
find_package(ament_cmake_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(dependencies
  pcl_conversions
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  EIGEN3
  PCL
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

## Declare the pcl_ros_tf library
add_library(pcl_ros2_tf src/transforms.cpp)
ament_target_dependencies(pcl_ros2_tf
  ${dependencies}
)
target_link_libraries(pcl_ros2_tf ${PCL_LIBRARIES})

### Declare the pcl_ros_filters library
# add_library(pcl_ros2_filters
#  src/pcl_ros2/filters/extract_indices.cpp
#  src/filters/filter.cpp
#  src/pcl_ros2/filters/passthrough.cpp
#  src/pcl_ros2/filters/project_inliers.cpp
#  src/pcl_ros2/filters/radius_outlier_removal.cpp
#  src/pcl_ros2/filters/statistical_outlier_removal.cpp
#  src/pcl_ros2/filters/voxel_grid.cpp
#  src/pcl_ros2/filters/crop_box.cpp
# )
# target_link_libraries(pcl_ros2_filters pcl_ros2_tf ${catkin_LIBRARIES} ${PCL_LIBRARIES})
# add_dependencies(pcl_ros2_filters ${PROJECT_NAME}_gencfg)
# class_loader_hide_library_symbols(pcl_ros2_filters)

set(node_plugins "")

add_library(filter_component SHARED
  src/filters/filter.cpp)
target_compile_definitions(filter_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(filter_component
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "pcl_msgs"
  "tf2"
  "PCL"
  "tf2_ros"
  "EIGEN3"
  "message_filters")
rclcpp_components_register_nodes(filter_component "pcl_ros2::Filter")
set(node_plugins "${node_plugins}pcl_ros2::Filter;$<TARGET_FILE:filter_component>\n")



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

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    pcl_ros2_tf
#    pcl_ros_io
#    pcl_ros_features
#    pcl_ros_filters
#    pcl_ros_surface
#    pcl_ros_segmentation
#    pcd_to_pointcloud
#    pointcloud_to_pcd
#    bag_to_pcd
#    convert_pcd_to_image
#    convert_pointcloud_to_image
  EXPORT export_pcl_ros
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

# install(DIRECTORY plugins samples
#   DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(pcl_ros2_tf)
ament_export_dependencies(${dependencies})
ament_export_targets(export_pcl_ros HAS_LIBRARY_TARGET)
ament_package()
