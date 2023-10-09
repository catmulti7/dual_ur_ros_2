cmake_minimum_required(VERSION 3.22)
project(dual_ur5_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)




install(DIRECTORY launch
 DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY srdf DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_python_install_package(ur_moveit_config)
ament_python_install_module(ur_moveit_config/launch_common.py)

ament_package()
