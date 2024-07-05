cmake_minimum_required(VERSION 3.16)
project(arm_firmware)

set(CMAKE_CXX_STANDARD 14)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
    add_compile_options(-Wall -Wextra)
endif()

# Fixes intellisense not finding ros2 headers https://youtrack.jetbrains.com/issue/CPP-29747/Certain-ROS2-package-headers-missing-from-Intellisense-when-using-a-Docker-toolchain
include_directories(SYSTEM /opt/ros/$ENV{ROS_DISTRO}/include)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
        hardware_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
        )

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach()

add_library(
        arm_firmware
        SHARED
        hardware/diffbot_system.cpp
        hardware/include/StepperAdapter.cpp)
target_compile_features(arm_firmware PUBLIC cxx_std_17)
target_include_directories(arm_firmware PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
        $<INSTALL_INTERFACE:include/arm_firmware>
        )
ament_target_dependencies(
        arm_firmware PUBLIC
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "ROS2_CONTROL_DEMO_EXAMPLE_2_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface ros2_control_demo_example_2.xml)

# INSTALL
install(
        DIRECTORY hardware/include/
        DESTINATION include/arm_firmware
)
install(
        DIRECTORY description/launch description/ros2_control description/urdf
        DESTINATION share/arm_firmware
)
install(
        DIRECTORY bringup/launch bringup/config
        DESTINATION share/arm_firmware
)
install(TARGETS arm_firmware
        EXPORT export_arm_firmware
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
ament_export_targets(export_arm_firmware HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
