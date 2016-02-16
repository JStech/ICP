cmake_policy(SET CMP0042 OLD)

include(def_library)
include(export_package)

set(ELAS_VERSION_MAJOR 1)
set(ELAS_VERSION_MINOR 0)
set(ELAS_VERSION ${ELAS_VERSION_MAJOR}.${ELAS_VERSION_MINOR})

option(BUILD_SHARED_LIBS "Build Shared Library" ON)


#################################################
# Append all includes.
list(APPEND ELAS_INC_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/
   )

# Add include directories to library.
include_directories(${ELAS_INC_DIRS})


#################################################
# Append all library paths.
list(APPEND ELAS_LINK_DIRS
   )


#################################################
# Append all libraries.
list(APPEND ELAS_LIBS
   )


#################################################
# Library headers and sources.
set(ELAS_HDRS
    include/elas/descriptor.h
    include/elas/elas.h
    include/elas/filter.h
    include/elas/image.h
    include/elas/matrix.h
    include/elas/timer.h
    include/elas/triangle.h
   )

set(ELAS_SRCS
    src/descriptor.cpp
    src/elas.cpp
    src/filter.cpp
    src/matrix.cpp
    src/triangle.cpp
   )


#################################################
# Define library.
def_library(libelas
  SOURCES ${ELAS_HDRS} ${ELAS_SRCS}
  LINK_LIBS ${ELAS_LIBS}
  PACKAGE ELAS)


#################################################
# Export library to cmake.
export_package(ELAS
  TARGETS libelas
  VERSION ${ELAS_VERSION}
  LIBRARY libelas
  LIBRARIES libelas
  INCLUDE_DIRS ${ELAS_INC_DIRS}
  LINK_DIRS ${ELAS_LINK_DIRS}
  )
