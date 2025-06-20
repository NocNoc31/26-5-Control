cmake_minimum_required(VERSION 3.5)
project(motor_driver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(can_interface REQUIRED)  # Thêm thư viện can_interface
find_package(std_msgs REQUIRED) 
find_package(rcl_interfaces REQUIRED)
find_package(std_srvs REQUIRED)

# Tạo node điều khiển motor

add_executable(ControlMotor src/ControlMotor.cpp)


# Liên kết với thư viện can_interface
ament_target_dependencies(ControlMotor rclcpp can_interface std_msgs rcl_interfaces std_srvs)



install(TARGETS
  ControlMotor

  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  config/exercise1_params.yaml
  config/exercise2_params.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()




