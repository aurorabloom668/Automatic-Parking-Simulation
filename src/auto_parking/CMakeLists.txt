cmake_minimum_required(VERSION 3.16)
project(auto_parking)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(prius_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ParkingStatus.msg"
  DEPENDENCIES std_msgs)

set(CMAKE_AUTOMOC ON)
add_library(parking_panel SHARED src/parking_panel.cpp include/auto_parking/parking_panel.hpp)
target_compile_features(parking_panel PUBLIC cxx_std_17)
target_include_directories(parking_panel PUBLIC include)
ament_target_dependencies(parking_panel rclcpp rviz_common pluginlib prius_msgs std_msgs)
rosidl_get_typesupport_target(status_typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(parking_panel Qt5::Widgets "${status_typesupport}")
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)
install(TARGETS parking_panel LIBRARY DESTINATION lib)

# rosidl 安装同名 Python 消息包；业务模块直接安装到其中，避免重复生成同名目标。
ament_get_python_install_dir(PYTHON_INSTALL_DIR)
install(DIRECTORY auto_parking/ DESTINATION "${PYTHON_INSTALL_DIR}/auto_parking"
  FILES_MATCHING PATTERN "*.py" PATTERN "__pycache__" EXCLUDE)
foreach(entry IN ITEMS gt_odom:gt_odom_node planned_park:auto_park_node
    parking_supervisor:hybrid_park_node control_mux:control_mux proximity_warn:proximity_node)
  string(REPLACE ":" ";" pair "${entry}")
  list(GET pair 0 executable)
  list(GET pair 1 module)
  file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${executable}"
    CONTENT "#!/usr/bin/env python3\nfrom auto_parking.${module} import main\nmain()\n")
  install(PROGRAMS "${CMAKE_CURRENT_BINARY_DIR}/${executable}" DESTINATION lib/${PROJECT_NAME})
endforeach()
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  set(test_env "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs"
    "ROS_DOMAIN_ID=121" "ROS_LOCALHOST_ONLY=1")
  ament_add_pytest_test(parking_geometry test/test_geometry.py ENV ${test_env})
  ament_add_pytest_test(local_parking test/test_local_planning.py TIMEOUT 120 ENV ${test_env})
  ament_add_pytest_test(supervisor test/test_supervisor.py TIMEOUT 120 ENV ${test_env})
  ament_add_pytest_test(control_mux test/test_control_mux.py ENV ${test_env})
  ament_add_pytest_test(tracking test/test_tracking.py TIMEOUT 120
    ENV ${test_env} "PYTEST_ADDOPTS=-o junit_logging=all")
  ament_add_pytest_test(ros_flow test/test_ros_flow.py TIMEOUT 60 ENV ${test_env})
  ament_add_pytest_test(perception test/test_perception.py ENV ${test_env})
  add_executable(panel_smoke test/panel_smoke.cpp)
  ament_target_dependencies(panel_smoke rclcpp rviz_common pluginlib prius_msgs std_msgs)
  target_link_libraries(panel_smoke Qt5::Widgets "${status_typesupport}")
  add_test(NAME panel_smoke COMMAND panel_smoke)
  set_tests_properties(panel_smoke PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen;${test_env}" TIMEOUT 30)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
