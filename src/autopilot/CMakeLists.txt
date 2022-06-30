cmake_minimum_required(VERSION 3.5)
project(autopilot)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

## By adding the below flags, the compiler does not ignore warnings and enforce cleaner code
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(navigation2 REQUIRED)

###########
## Build ##
###########

# set(msg_files
#   "msg/ExampleType.msg")

# set(srv_files
#   "srv/ExampleService.srv")
  
# rosidl_generate_interfaces(${PROJECT_NAME} 
#   ${msg_files}
#   ${srv_files}
#   DEPENDENCIES std_msgs builtin_interfaces)

## Specify additional locations of header files
include_directories(include)

## Declare a CPP executable
# add_executable(${PROJECT_NAME}_pub src/${PROJECT_NAME}_pub.cpp)
# ament_target_dependencies(${PROJECT_NAME}_pub "rclcpp" "std_msgs")
# rosidl_target_interfaces(${PROJECT_NAME}_pub ${PROJECT_NAME} "rosidl_typesupport_cpp")

# add_executable(${PROJECT_NAME}_sub src/${PROJECT_NAME}_sub.cpp)
# ament_target_dependencies(${PROJECT_NAME}_sub "rclcpp" "std_msgs")
# rosidl_target_interfaces(${PROJECT_NAME}_sub ${PROJECT_NAME} "rosidl_typesupport_cpp")

# add_executable(${PROJECT_NAME}_service src/${PROJECT_NAME}_service.cpp)
# ament_target_dependencies(${PROJECT_NAME}_service "rclcpp" "std_msgs")
# rosidl_target_interfaces(${PROJECT_NAME}_service ${PROJECT_NAME} "rosidl_typesupport_cpp")

# add_executable(${PROJECT_NAME}_client src/${PROJECT_NAME}_client.cpp)
# ament_target_dependencies(${PROJECT_NAME}_client "rclcpp" "std_msgs")
# rosidl_target_interfaces(${PROJECT_NAME}_client ${PROJECT_NAME} "rosidl_typesupport_cpp")

#############
## Install ##
#############

## Install files to corresponding directories
install(
  DIRECTORY behavior_trees config launch models worlds 
  DESTINATION share/${PROJECT_NAME})

##########
## Test ##
##########

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
