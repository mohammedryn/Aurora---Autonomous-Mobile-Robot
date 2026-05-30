cmake_minimum_required(VERSION 3.8)
project(amr_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(PROGRAMS scripts/twist_to_reference.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
