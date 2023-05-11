cmake_minimum_required(VERSION 3.8)
project(tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# add_definitions("-DNDEBUG")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(dlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL 1.10 REQUIRED COMPONENTS common io filters segmentation)
find_package(rcpputils)
add_subdirectory(dbscan)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

include_directories(
  include 
  ${DLIB_INCLUDE_DIRECTORY}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${rviz_visual_tools_INCLUDE_DIRS}
  )

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(tracker 
  src/tracker.cpp
  src/kf.cpp
  src/utils.cpp
  src/vis.cpp
)

ament_target_dependencies(tracker
  rclcpp
  std_msgs 
  sensor_msgs
  geometry_msgs
  laser_geometry
  nav_msgs
  tf2_ros
  tf2_eigen
  visualization_msgs
  rviz_visual_tools
  pcl_conversions
  pcl_ros
)

target_link_libraries(tracker 
  ${EIGEN3_LIBRARIES}
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
  dlib
  dbscan  
)

install(
  TARGETS 
    tracker
  DESTINATION 
    lib/${PROJECT_NAME}
)

install(PROGRAMS
    scripts/scan2pc.py
  DESTINATION 
    lib/${PROJECT_NAME}
)

install(
  DIRECTORY 
    launch
  DESTINATION 
    share/${PROJECT_NAME}
)

ament_package()
