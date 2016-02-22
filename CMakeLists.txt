cmake_minimum_required(VERSION 2.8)
project(ICP)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")

add_subdirectory(libelas)

add_executable( icp main.cpp )
add_executable( plot_pc plot_pc.cpp )

find_package(Calibu REQUIRED)
find_package(HAL REQUIRED)
find_package(Pangolin REQUIRED)
find_package(SceneGraph REQUIRED)

include_directories( ${Calibu_INCLUDE_DIRS} )
include_directories( ${HAL_INCLUDE_DIRS} )
include_directories( ${Pangolin_INCLUDE_DIRS} )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/libelas/include )

target_link_libraries( plot_pc
  ${Calibu_LIBRARIES}
  ${HAL_LIBRARIES}
  ${Pangolin_LIBRARIES}
  ${SceneGraph_LIBRARIES}
  libelas
  )

target_link_libraries( icp
  ${Calibu_LIBRARIES}
  ${HAL_LIBRARIES}
  ${Pangolin_LIBRARIES}
  ${SceneGraph_LIBRARIES}
  libelas
  )
