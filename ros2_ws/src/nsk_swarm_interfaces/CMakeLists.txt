cmake_minimum_required(VERSION 3.8)
project(nsk_swarm_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Compress.srv"
  "srv/Merge.srv"
  "srv/SimilarityQuery.srv"
)
ament_package()
