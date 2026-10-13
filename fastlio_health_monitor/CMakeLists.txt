cmake_minimum_required(VERSION 3.0.2)
project(fastlio_health_monitor)

add_compile_options(-std=c++14 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  livox_ros_driver
  geometry_msgs
  message_generation
  nav_msgs
  pluginlib
  roscpp
  rviz
  sensor_msgs
  std_msgs
  std_srvs
)

add_message_files(FILES RuntimeState.msg TaskState.msg)
add_service_files(FILES ControlModule.srv)
generate_messages(DEPENDENCIES std_msgs)

find_package(Eigen3 REQUIRED)
catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES robot_health_core fastlio_health_monitor_panel
  CATKIN_DEPENDS diagnostic_msgs geometry_msgs message_runtime livox_ros_driver nav_msgs pluginlib roscpp rviz sensor_msgs std_msgs
)

find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(OpenSSL REQUIRED)
find_package(Threads REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
qt5_wrap_cpp(HEALTH_PANEL_MOC include/fastlio_health_monitor/health_panel.h)

add_library(robot_health_core src/health_core.cpp src/event_recorder.cpp)
target_link_libraries(robot_health_core yaml-cpp ${Boost_LIBRARIES} OpenSSL::Crypto ${CMAKE_THREAD_LIBS_INIT} ${CMAKE_DL_LIBS})
target_include_directories(robot_health_core PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
add_custom_target(health_source_manifest ALL
  BYPRODUCTS ${CMAKE_CURRENT_BINARY_DIR}/source_build_manifest.h ${CMAKE_CURRENT_BINARY_DIR}/source_build_manifest.json
  COMMAND /usr/bin/python3 ${CMAKE_CURRENT_SOURCE_DIR}/test/build_manifest.py
    ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR}/source_build_manifest.json)
add_dependencies(robot_health_core health_source_manifest)

add_executable(robot_health_monitor_node src/robot_health_monitor_node.cpp src/input_adapters.cpp)
add_dependencies(robot_health_monitor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_health_monitor_node robot_health_core ${catkin_LIBRARIES})
add_executable(robot_runtime_controller_node src/runtime_controller_node.cpp)
add_dependencies(robot_runtime_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_runtime_controller_node robot_health_core ${catkin_LIBRARIES})
add_executable(health_bag_playback_node src/bag_playback_node.cpp)
target_link_libraries(health_bag_playback_node ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(health_core_test test/health_core_test.cpp)
  if(TARGET health_core_test)
    target_link_libraries(health_core_test robot_health_core)
    target_compile_definitions(health_core_test PRIVATE HEALTH_TEST_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
  endif()
endif()

add_library(fastlio_health_monitor_panel src/health_panel.cpp ${HEALTH_PANEL_MOC})
add_dependencies(fastlio_health_monitor_panel ${catkin_EXPORTED_TARGETS})
target_link_libraries(fastlio_health_monitor_panel ${catkin_LIBRARIES} Qt5::Widgets)

install(TARGETS robot_health_monitor_node robot_runtime_controller_node health_bag_playback_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS fastlio_health_monitor_panel robot_health_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(FILES health_panel_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
