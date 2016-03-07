cmake_minimum_required(VERSION 2.8)
project(ICP)

list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")

#set(LibArchive_INCLUDE_DIR /usr/local/opt/libarchive/include)
#set(LibArchive_LIBRARY_DIR /usr/local/opt/libarchive/lib)

add_executable( icp main.cpp )
add_executable( plot_pc plot_pc.cpp )

find_package(Calibu REQUIRED)
find_package(HAL REQUIRED)
find_package(Pangolin REQUIRED)
find_package(SceneGraph REQUIRED)
find_package(PCL REQUIRED COMPONENTS io)
find_package(LibArchive REQUIRED)

include_directories( ${Calibu_INCLUDE_DIRS} )
include_directories( ${HAL_INCLUDE_DIRS} )
include_directories( ${Pangolin_INCLUDE_DIRS} )
include_directories( ${SceneGraph_INCLUDE_DIRS} )
include_directories( ${PCL_INCLUDE_DIRS} )
include_directories( ${LibArchive_INCLUDE_DIRS} )

target_link_libraries( plot_pc
  ${Calibu_LIBRARIES}
  ${HAL_LIBRARIES}
  ${Pangolin_LIBRARIES}
  ${SceneGraph_LIBRARIES}
  ${PCL_LIBRARIES}
  ${LibArchive_LIBRARIES}
  )

target_link_libraries( icp
  ${Calibu_LIBRARIES}
  ${HAL_LIBRARIES}
  ${Pangolin_LIBRARIES}
  ${SceneGraph_LIBRARIES}
  ${PCL_LIBRARIES}
  ${LibArchive_LIBRARIES}
  )
