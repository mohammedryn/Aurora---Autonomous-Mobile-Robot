cmake_minimum_required(VERSION 3.8)
project(amr_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes worlds config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
