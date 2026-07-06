cmake_minimum_required(VERSION 3.8)
project(nsk_swarm)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch
)
install(
  DIRECTORY worlds/ DESTINATION share/${PROJECT_NAME}/worlds
)
install(
  DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config
)
install(
  DIRECTORY rviz/ DESTINATION share/${PROJECT_NAME}/rviz
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
