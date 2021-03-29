cmake_minimum_required(VERSION 3.14)

#get options and compiling env
include(cmake/options.cmake)
include(cmake/linker.cmake)
include(cmake/ccache.cmake)

project(ShipRouter)
project(ShipRouter LANGUAGES CXX)

#set C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

############################
###get external projects
############################
include(cmake/fmt.cmake)
include(cmake/tbb.cmake)
include(cmake/span.cmake)
include(cmake/CLI11.cmake)
include(cmake/gtest.cmake)
include(cmake/progress.cmake)
include(cmake/pistache.cmake)
include(cmake/json.cmake)

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

if(CMAKE_BUILD_TYPE STREQUAL "Release")
  set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

# needed for multithreading
find_package(Threads REQUIRED)

include(cmake/flags.cmake)

#############################################
## library with all the logic which
## can later be easily linked against test
#############################################
add_library(ShipRouterSrc "")

target_sources(ShipRouterSrc
  PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/include/Graph.hpp
  ${CMAKE_CURRENT_LIST_DIR}/include/PBFExtractor.hpp
  ${CMAKE_CURRENT_LIST_DIR}/include/NodeLookup.hpp
  ${CMAKE_CURRENT_LIST_DIR}/include/OSMNode.hpp
  ${CMAKE_CURRENT_LIST_DIR}/include/Coastline.hpp
  ${CMAKE_CURRENT_LIST_DIR}/include/CoastlineLookup.hpp
  ${CMAKE_CURRENT_LIST_DIR}/include/Polygon.hpp
  ${CMAKE_CURRENT_LIST_DIR}/include/Dijkstra.hpp
  ${CMAKE_CURRENT_LIST_DIR}/include/LatLng.hpp
  ${CMAKE_CURRENT_LIST_DIR}/include/Range.hpp
  ${CMAKE_CURRENT_LIST_DIR}/include/SphericalGrid.hpp
  ${CMAKE_CURRENT_LIST_DIR}/include/ServiceManager.hpp
  ${CMAKE_CURRENT_LIST_DIR}/include/IndependentSetCalculator.hpp
  ${CMAKE_CURRENT_LIST_DIR}/include/GraphContractor.hpp

  PRIVATE
  src/Graph.cpp
  src/PBFExtractor.cpp
  src/NodeLookup.cpp
  src/OSMNode.cpp
  src/Coastline.cpp
  src/CoastlineLookup.cpp
  src/Polygon.cpp
  src/SphericalGrid.cpp
  src/Dijkstra.cpp
  src/ServiceManager.cpp
  src/IndependentSetCalculator.cpp
  src/GraphContractor.cpp
  )

# add the dependencies of the target to enforce
# the right order of compiling
add_dependencies(ShipRouterSrc fmt-project)
add_dependencies(ShipRouterSrc tbb-project)
add_dependencies(ShipRouterSrc span-lite-project)
add_dependencies(ShipRouterSrc CLI11-project)
add_dependencies(ShipRouterSrc progress-cpp-project)
add_dependencies(ShipRouterSrc pistache-project)
add_dependencies(ShipRouterSrc json-project)

# make headers available
target_include_directories(ShipRouterSrc PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/vendor/osmpbf/include
  ${CMAKE_CURRENT_SOURCE_DIR}/vendor/libosmpbfreader/include
  fmt
  tbb
  ${JSON_INCLUDE_DIR}
  pistache
  ${CLI11_INCLUDE_DIR}
  ${PROGRESS_CPP_INCLUDE_DIR}
  ${SPAN_LITE_INCLUDE_DIR}
  )

#link against libarys
target_link_libraries(ShipRouterSrc LINK_PUBLIC
  fmt
  pistache
  tbb)

target_link_libraries(ShipRouterSrc PRIVATE
  ${CMAKE_THREAD_LIBS_INIT})


###############################
## THE ACTUAL BINARY
###############################
add_executable(ShipRouter src/main.cpp)


# statically link protobuf
set(Protobuf_USE_STATIC_LIBS ON)

find_package(Protobuf REQUIRED)

set(CMAKE_FIND_LIBRARY_SUFFIXES ".a")
find_package(ZLIB REQUIRED)

# make headers available
target_include_directories(ShipRouter PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/vendor/osmpbf/include
  ${CMAKE_CURRENT_SOURCE_DIR}/vendor/libosmpbfreader/include
  fmt
  tbb
  pistache
  ${JSON_INCLUDE_DIR}
  ZLIB::ZLIB
  ${CLI11_INCLUDE_DIR}
  ${PROGRESS_CPP_INCLUDE_DIR}
  ${SPAN_LITE_INCLUDE_DIR}
  )

#link against libarys
target_link_libraries(ShipRouter LINK_PUBLIC
  ShipRouterSrc
  fmt
  tbb
  ZLIB::ZLIB
  ${CMAKE_CURRENT_SOURCE_DIR}/vendor/osmpbf/lib/libosmpbf.a
  ${Protobuf_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT})

# add the dependencies of the target to enforce
# the right order of compiling
add_dependencies(ShipRouter ShipRouterSrc)

if (BUILD_TESTS)
  enable_testing()
  add_subdirectory(test)
endif (BUILD_TESTS)
