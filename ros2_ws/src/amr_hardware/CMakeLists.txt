cmake_minimum_required(VERSION 3.8)
project(amr_hardware)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(amr_hardware SHARED
    src/serial_driver.cpp
    src/amr_hardware_interface.cpp
)
target_include_directories(amr_hardware PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(amr_hardware
    hardware_interface pluginlib rclcpp rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface amr_hardware.xml)

install(TARGETS amr_hardware
    EXPORT export_amr_hardware
    LIBRARY  DESTINATION lib
    ARCHIVE  DESTINATION lib
    RUNTIME  DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(FILES amr_hardware.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(amr_hardware)
ament_export_targets(export_amr_hardware)
ament_package()
