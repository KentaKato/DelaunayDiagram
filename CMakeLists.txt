cmake_minimum_required(VERSION 3.10)

project(DelaunayTriangulation CXX)

set(CMAKE_CXX_STANDARD 17)

find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${YAML_CPP_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/geometry_primitives.cpp
  src/delaunay_triangulation.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${YAML_CPP_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(delaunay_triangulation
  src/main.cpp
)

target_link_libraries(delaunay_triangulation
  ${PROJECT_NAME}
)

add_executable(test_geometry_primitives
  test/test_geometry_primitives.cpp
)

target_link_libraries(test_geometry_primitives
  ${PROJECT_NAME}
)