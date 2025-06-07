cmake_minimum_required(VERSION 3.16)
project(umrt-arm-ros-firmware)

set(CMAKE_CXX_STANDARD 14)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
    add_compile_options(-Wall -Wextra -fPIC)
endif()

# Fixes intellisense not finding ros2 headers https://youtrack.jetbrains.com/issue/CPP-29747/Certain-ROS2-package-headers-missing-from-Intellisense-when-using-a-Docker-toolchain
include_directories(SYSTEM /opt/ros/$ENV{ROS_DISTRO}/include)
include(CMakeFindDependencyMacro)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
        hardware_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
        ament_cmake
        umrt-arm-firmware-lib
        )
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
    find_package(${Dependency} CONFIG REQUIRED)
endforeach()

# Setup ros2_control_demo_description
install(
        DIRECTORY hardware/include/ros2_control_demo_description/diffbot/urdf hardware/include/ros2_control_demo_description/diffbot/rviz
        DESTINATION share/${PROJECT_NAME}/ros2_control_demo_description/diffbot
)

add_library(
        umrt-arm-ros-firmware
        SHARED
        hardware/robotic_arm_control_system.cpp
        hardware/stepper_adapter.cpp
        hardware/arduino_stepper_adapter.cpp)
target_compile_features(umrt-arm-ros-firmware PUBLIC cxx_std_17)
target_include_directories(umrt-arm-ros-firmware PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
        $<INSTALL_INTERFACE:include/umrt-arm-ros-firmware>
        )
ament_target_dependencies(
        umrt-arm-ros-firmware PUBLIC
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
# ament_target_dependencies doesn't use namespaces when linking, so we need to link umrt-arm-firmware-lib ourself
target_link_libraries(umrt-arm-ros-firmware PRIVATE umrt-arm-firmware-lib::umrt-arm-firmware-lib)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "umrt-arm-ros-firmware_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface umrt-arm-ros-firmware.xml)

# Parse version number from the file
file(READ "version" version_input)
string(REGEX MATCH "([0-9]+)\.([0-9]+)\.([0-9]+)" _ ${version_input})

# Fill version number into package.xml
configure_file(
        package.in.xml
        ${CMAKE_CURRENT_SOURCE_DIR}/package.xml
        @ONLY
)

# INSTALL
install(
        DIRECTORY hardware/include/
        DESTINATION include/umrt-arm-ros-firmware
)
install(
        DIRECTORY description/launch description/ros2_control description/urdf
        DESTINATION share/umrt-arm-ros-firmware
)
install(
        DIRECTORY bringup/launch bringup/config
        DESTINATION share/umrt-arm-ros-firmware
)
install(TARGETS umrt-arm-ros-firmware
        EXPORT export_umrt-arm-ros-firmware
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )

if(BUILD_TESTING)
    find_package(ament_cmake_pytest REQUIRED)

    ament_add_pytest_test(example_2_urdf_xacro test/test_urdf_xacro.py)
    ament_add_pytest_test(view_example_2_launch test/test_view_robot_launch.py)
endif()

## EXPORTS
ament_export_targets(export_umrt-arm-ros-firmware HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
