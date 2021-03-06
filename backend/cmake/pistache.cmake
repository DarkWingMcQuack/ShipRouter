include(ExternalProject)
include(GNUInstallDirs)

set(CMAKE_ARGS
  -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
  -DCMAKE_BUILD_TYPE=Release
  -DPISTACHE_BUILD_EXAMPLES=FALSE
  -DPISTACHE_BUILD_TESTS=FALSE
  -DPISTACHE_BUILD_DOCS=FALSE
  -DPISTACHE_USE_SSL=FALSE)

ExternalProject_Add(pistache-project
  PREFIX deps/pistache
  GIT_REPOSITORY https://github.com/oktal/pistache.git
  GIT_TAG 06508ad02b14c9c5e3ac16e7a095e86114150a16
  DOWNLOAD_DIR ${CMAKE_BINARY_DIR}/downloads
  CMAKE_ARGS ${CMAKE_ARGS}
  PATCH_COMMAND cmake -E make_directory <SOURCE_DIR>/win32-deps/include
  BUILD_COMMAND cmake --build <BINARY_DIR> --config Release
  UPDATE_COMMAND ""
  INSTALL_COMMAND cmake --build <BINARY_DIR> --config Release --target install
  )

ExternalProject_Get_Property(pistache-project INSTALL_DIR)
add_library(pistache STATIC IMPORTED)
set(PISTACHE_LIBRARY ${INSTALL_DIR}/${CMAKE_INSTALL_LIBDIR}/${CMAKE_STATIC_LIBRARY_PREFIX}pistache${CMAKE_STATIC_LIBRARY_SUFFIX})
set(PISTACHE_INCLUDE_DIR ${INSTALL_DIR}/include)
file(MAKE_DIRECTORY ${PISTACHE_INCLUDE_DIR})  # Must exist.
set_property(TARGET pistache PROPERTY IMPORTED_LOCATION ${PISTACHE_LIBRARY})
set_property(TARGET pistache PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${PISTACHE_INCLUDE_DIR})

unset(INSTALL_DIR)
unset(CMAKE_ARGS)
