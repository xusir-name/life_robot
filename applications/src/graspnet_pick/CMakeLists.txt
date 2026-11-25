cmake_minimum_required(VERSION 3.8)
project(graspnet_pick)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_CXX_STANDARD_REQUIRED)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

add_library(graspnet_pick_core SHARED
  src/geometry.cpp
  src/transforms.cpp
)
target_include_directories(graspnet_pick_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(graspnet_pick_core
  Eigen3::Eigen
  ${OpenCV_LIBS}
  yaml-cpp
)
ament_target_dependencies(graspnet_pick_core geometry_msgs)

add_executable(grasp_estimator
  src/estimator_node.cpp
  src/grasp_estimator_main.cpp
)
target_link_libraries(grasp_estimator graspnet_pick_core)
ament_target_dependencies(grasp_estimator
  rclcpp
  interfaces
  sensor_msgs
  geometry_msgs
  tf2_ros
  cv_bridge
)
target_link_libraries(grasp_estimator ${OpenCV_LIBS})

add_executable(grasp_executor
  src/executor_node.cpp
  src/grasp_executor_main.cpp
)
target_link_libraries(grasp_executor graspnet_pick_core)
ament_target_dependencies(grasp_executor
  rclcpp
  interfaces
  sensor_msgs
  geometry_msgs
  std_msgs
  trajectory_msgs
  tf2
  tf2_ros
)

install(TARGETS
  grasp_estimator
  grasp_executor
  graspnet_pick_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_geometry test/test_geometry.cpp)
  target_link_libraries(test_geometry graspnet_pick_core)
  target_include_directories(test_geometry PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(test_geometry ${OpenCV_LIBS})
  ament_target_dependencies(test_geometry geometry_msgs)
endif()

ament_package()
