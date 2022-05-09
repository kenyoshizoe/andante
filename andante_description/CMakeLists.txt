cmake_minimum_required(VERSION 3.5)
project(andante_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY mesh/
  DESTINATION share/${PROJECT_NAME}/mesh
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
  PATTERN ".svn" EXCLUDE
)

ament_package()
