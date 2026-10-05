cmake_minimum_required(VERSION 3.10)
project(mobilerobot_toxic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)

install(
  DIRECTORY launch urdf rviz config meshes meshes_gz worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/fake_mobile_base.py
  scripts/mecanum_fake_controller.py
  scripts/fake_lidar_publisher.py
  scripts/ps5_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
