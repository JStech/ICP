project(ICP)
cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
aux_source_directory(. SRC_LIST)
add_executable(${PROJECT_NAME} ${SRC_LIST})

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)

add_executable( icp main.cpp )

find_package(HAL REQUIRED)
find_package(Calibu REQUIRED)

include_directories( ${HAL_INCLUDE_DIRS} )
include_directories( ${Calibu_INCLUDE_DIRS} )

target_link_libraries( icp
  ${HAL_LIBRARIES}
  ${Calibu_LIBRARIES}
  )
