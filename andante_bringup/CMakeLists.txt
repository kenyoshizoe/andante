cmake_minimum_required(VERSION 3.5)
project(andante_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  PATTERN ".svn" EXCLUDE
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  PATTERN ".svn" EXCLUDE
)
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
  PATTERN ".svn" EXCLUDE
)

ament_package()
