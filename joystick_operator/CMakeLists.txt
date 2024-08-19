cmake_minimum_required(VERSION 3.16)
project(joystick_operator)

set(CMAKE_CXX_STANDARD 14)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -fPIC)
endif()

# Fixes intellisense not finding ros2 headers https://youtrack.jetbrains.com/issue/CPP-29747/Certain-ROS2-package-headers-missing-from-Intellisense-when-using-a-Docker-toolchain
include_directories(SYSTEM /opt/ros/$ENV{ROS_DISTRO}/include)

# Setup Boost
find_package(Boost REQUIRED COMPONENTS log log_setup)
include_directories(${Boost_INCLUDE_DIRS})

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
        rclcpp
        rclcpp_lifecycle
        std_msgs
        sensor_msgs
        )

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_executable(${PROJECT_NAME} src/${PROJECT_NAME}.cpp src/joystick_teleop_node.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
        )
target_link_libraries(${PROJECT_NAME} PRIVATE ${Boost_LIBRARIES})
ament_target_dependencies(
        ${PROJECT_NAME} PUBLIC
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# INSTALL
install(TARGETS
        ${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME}
        )

ament_package()
