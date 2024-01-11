cmake_minimum_required(VERSION 3.1)

project(ai_robot_core)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(websocketpp REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  ai_robot_msgs
  geometry_msgs
  nav_msgs
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  laser_geometry
)

catkin_package(
  CATKIN_DEPENDS
    # ai_robot_msgs
    geometry_msgs
    nav_msgs
    pcl_ros
    roscpp
    sensor_msgs
    std_msgs
    tf2
    tf2_ros
        laser_geometry
  DEPENDS
    eigen
    opencv
    libwebsocketpp-dev
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  # ${OpenCV_INCLUDE_DIRS}
)

add_executable(test_ws
  src/test/test_websocket.cpp
  src/websocket/client.cpp
)
target_include_directories(test_ws PRIVATE)
target_link_libraries(test_ws pthread websocketpp::websocketpp)

add_executable(test_cloudplanner
  src/test/test_cloudplanner.cpp
  src/cloudplanner.cpp
  src/websocket/client.cpp
  src/bluetooth.cpp
)
# target_include_directories(test_cloudplanner PRIVATE)
target_link_libraries(test_cloudplanner pthread websocketpp::websocketpp ${catkin_LIBRARIES})

add_executable(test_merca2plane
  src/test/test_merca2plane.cpp
  src/bluetooth.cpp
)
target_include_directories(test_merca2plane PRIVATE ${EIGEN3_INCLUDE_DIR})
target_link_libraries(test_merca2plane ${EIGEN_LIBRARIES})

# add_executable(map2odom_tf_pub
#   src/map2odom_tf_pub
# )
# target_link_libraries(map2odom_tf_pub ${catkin_LIBRARIES})

add_executable(odom2map_tf_pub
  src/odom2map_tf_pub
)
target_link_libraries(odom2map_tf_pub ${catkin_LIBRARIES})

add_executable(movebase_node
  src/movebase_node.cpp
  src/movebase.cpp
  src/cloudplanner.cpp
  #src/simple_local_planner.cpp
  src/globals.cpp
  src/websocket/client.cpp
  src/util.cpp
  src/bluetooth.cpp
  #src/obstacle_avoid.cpp
        src/falco_local_planner.cpp
)
add_dependencies(movebase_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(movebase_node ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} ${OpenCV_LIBS})

# add_executable(movebase_client
#   src/movebase_client_node.cpp
#   src/movebase_client.cpp
#   src/globals.cpp
#   src/bluetooth.cpp
# )
# add_dependencies(movebase_client ${catkin_EXPORTED_TARGETS})
# target_link_libraries(movebase_client ${catkin_LIBRARIES})

#add_executable(brain_node
#  src/brain_node.cpp
#  src/brain.cpp
#  src/cloudplanner.cpp
#  src/movebase_client.cpp
#  src/man_control.cpp
#  src/base_control.cpp
#  src/globals.cpp
#  src/websocket/client.cpp
#  src/bluetooth.cpp
#  src/obstacle_avoid.cpp
#  src/util.cpp
#)

add_executable(brain_node
  src/brain_node.cpp
  src/brain.cpp
  src/cloudplanner.cpp
  src/movebase_client.cpp
  src/man_control.cpp
  src/base_control.cpp
  src/globals.cpp
  src/websocket/client.cpp
  src/bluetooth.cpp
  src/util.cpp
)
add_dependencies(brain_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(brain_node ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} ${OpenCV_LIBS})

catkin_python_setup()

catkin_install_python(PROGRAMS nodes/oa_sim_noros nodes/run_oa
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

execute_process(
  COMMAND bash "-c" "if command -v catkni-lsp &>/dev/null; then (cd ${CMAKE_SOURCE_DIR} && catkin-lsp); fi"
)
