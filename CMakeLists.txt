cmake_minimum_required(VERSION 2.8)
project(ICP)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
add_definitions(-DPROFILE)

option(BUILD_ICP "Build the ICP library, requires other packages" ON)
option(BUILD_ICP_TEST "Build the ICP library test suite, requires other packages" OFF)

if (BUILD_ICP)
   find_package(Sophus REQUIRED)
endif()

if(BUILD_ICP_TEST)
  find_package(Calibu REQUIRED)
  find_package(HAL REQUIRED)
  find_package(Pangolin REQUIRED)
  find_package(SceneGraph REQUIRED)
endif()

find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

if(BUILD_ICP_TEST)
  include_directories( ${Calibu_INCLUDE_DIRS} )
  include_directories( ${HAL_INCLUDE_DIRS} )
  include_directories( ${Pangolin_INCLUDE_DIRS} )
  include_directories( ${SceneGraph_INCLUDE_DIRS} )
endif()

if (BUILD_ICP)
   include_directories( ${Sophus_INCLUDE_DIRS})
endif()

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

add_library( kdtree SHARED src/kdtree.cpp )
add_executable( kdtree_test tests/kdtree_test.cpp )
target_link_libraries( kdtree_test kdtree ${PCL_LIBRARIES} )

if(BUILD_ICP)
  add_library( icp SHARED src/icp.cpp )
  target_link_libraries( icp dualquat ${PCL_LIBRARIES} )
  
  if (BUILD_ICP_TEST)
  add_executable( icp_test tests/icp_test.cpp )
  target_link_libraries( icp_test
  dualquat
  icp
  ${PCL_LIBRARIES}
  ${Pangolin_LIBRARIES}
  ${SceneGraph_LIBRARIES}
  )

  add_executable( main src/main.cpp )
  target_link_libraries( main icp dualquat ${PCL_LIBRARIES} )

  add_executable( aggregate_pc src/aggregate_pc.cpp )
  add_executable( make_pc src/make_pc.cpp )
  add_executable( plot_pc src/plot_pc.cpp )

  target_link_libraries( aggregate_pc
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

  target_link_libraries( plot_pc
    ${HAL_LIBRARIES}
    ${Pangolin_LIBRARIES}
    ${SceneGraph_LIBRARIES}
    ${PCL_LIBRARIES}
    )
  endif()
endif()
