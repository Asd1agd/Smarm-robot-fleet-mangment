cmake_minimum_required(VERSION 3.8)
project(micro_ros_custom_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MicrorosFeedback.msg"
  "msg/MicrorosLog.msg"
  "srv/MicrorosSendGoal.srv"
  "srv/MicrorosCancelGoal.srv"
 )

# Export for other packages
# ament_export_dependencies(rosidl_default_runtime) ##
ament_package()


