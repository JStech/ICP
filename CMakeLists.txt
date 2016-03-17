cmake_minimum_required(VERSION 2.8)
project(ICP)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")

find_package(Calibu REQUIRED)
find_package(HAL REQUIRED)
find_package(Pangolin REQUIRED)
find_package(SceneGraph REQUIRED)
find_package(PCL REQUIRED)

include_directories( ${Calibu_INCLUDE_DIRS} )
include_directories( ${HAL_INCLUDE_DIRS} )
include_directories( ${Pangolin_INCLUDE_DIRS} )
include_directories( ${SceneGraph_INCLUDE_DIRS} )
include_directories( ${PCL_INCLUDE_DIRS} )
include_directories( "include"
  ${CMAKE_BINARY_DIR}
  ${CMAKE_SOURCE_DIR}
  )

link_directories( ${CMAKE_BINARY_DIR} )
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

add_library( dualquat SHARED src/dualquat.cpp )
add_executable( dualquat_test tests/dualquat_test.cpp )
target_link_libraries( dualquat_test dualquat )

add_library( icp SHARED src/icp.cpp )
add_executable( icp_test tests/icp_test.cpp )
target_link_libraries( icp dualquat ${PCL_LIBRARIES} )
target_link_libraries( icp_test dualquat icp ${PCL_LIBRARIES} )

add_executable( main src/main.cpp )
target_link_libraries( main icp dualquat ${PCL_LIBRARIES} )

add_executable( make_pc src/make_pc.cpp )
add_executable( plot_pc src/plot_pc.cpp )

target_link_libraries( plot_pc
  ${HAL_LIBRARIES}
  ${Pangolin_LIBRARIES}
  ${SceneGraph_LIBRARIES}
  ${PCL_LIBRARIES}
  )

target_link_libraries( make_pc
  ${Calibu_LIBRARIES}
  ${HAL_LIBRARIES}
  ${Pangolin_LIBRARIES}
  ${SceneGraph_LIBRARIES}
  ${PCL_LIBRARIES}
  )
