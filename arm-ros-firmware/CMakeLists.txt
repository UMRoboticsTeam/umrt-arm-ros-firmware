cmake_minimum_required(VERSION 3.16)
project(arm_ros_firmware)

set(CMAKE_CXX_STANDARD 14)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
    add_compile_options(-Wall -Wextra -fPIC)
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

# Setup Boost
find_package(Boost REQUIRED COMPONENTS log log_setup)
include_directories(${Boost_INCLUDE_DIRS})

# Setup arm-firmware-lib
add_subdirectory(hardware/include/arm-firmware-lib)

# Setup ros2_control_demo_description
install(
        DIRECTORY hardware/include/ros2_control_demo_description/diffbot/urdf hardware/include/ros2_control_demo_description/diffbot/rviz
        DESTINATION share/${PROJECT_NAME}/ros2_control_demo_description/diffbot
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach()

add_library(
        arm_ros_firmware
        SHARED
        hardware/diffbot_system.cpp
        hardware/StepperAdapter.cpp
)
target_compile_features(arm_ros_firmware PUBLIC cxx_std_17)
target_include_directories(arm_ros_firmware PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
        $<INSTALL_INTERFACE:include/arm_ros_firmware>
        )
ament_target_dependencies(
        arm_ros_firmware PUBLIC
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(arm_ros_firmware PRIVATE ${Boost_LIBRARIES})
target_link_libraries(arm_ros_firmware PRIVATE arm_firmware_lib)
target_include_directories(arm_ros_firmware PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/hardware/include/arm-firmware-lib/arm-firmware-lib>
        )

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "ARM_ROS_FIRMWARE_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface arm_ros_firmware.xml)

# INSTALL
install(
        DIRECTORY hardware/include/
        DESTINATION include/arm_ros_firmware
)
install(
        DIRECTORY description/launch description/ros2_control description/urdf
        DESTINATION share/arm_ros_firmware
)
install(
        DIRECTORY bringup/launch bringup/config
        DESTINATION share/arm_ros_firmware
)
install(TARGETS arm_ros_firmware
        EXPORT export_arm_ros_firmware
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
ament_export_targets(export_arm_ros_firmware HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
