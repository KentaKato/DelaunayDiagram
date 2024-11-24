cmake_minimum_required(VERSION 3.10)

project(DelaunayVoronoiCpp CXX)

set(CMAKE_CXX_STANDARD 20)

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
  src/delaunay_triangulation_drawer.cpp
  src/voronoi_diagram.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${YAML_CPP_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(interactive_delaunay_voronoi
  src/interactive_delaunay_voronoi.cpp
)

target_link_libraries(interactive_delaunay_voronoi
  ${PROJECT_NAME}
)

add_executable(centroid_voronoi
  src/centroid_voronoi.cpp
)

target_link_libraries(centroid_voronoi
  ${PROJECT_NAME}
)