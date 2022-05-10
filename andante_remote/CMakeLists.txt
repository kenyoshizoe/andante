cmake_minimum_required(VERSION 3.5)
project(andante_remote)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  PATTERN ".svn" EXCLUDE
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  PATTERN ".svn" EXCLUDE
)

ament_package()
