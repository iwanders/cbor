cmake_minimum_required(VERSION 3.5.0)

project(Cbor VERSION 0.0.1)

# Actual target export.
add_library(cbor INTERFACE)
target_include_directories(cbor
  INTERFACE
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/include/>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
add_library(Cbor::cbor ALIAS cbor)
target_compile_features(cbor INTERFACE cxx_relaxed_constexpr)

# testing
include(CTest)
if(BUILD_TESTING)
  enable_testing()
  add_subdirectory(test)
endif()

# clang format helper target.
file(GLOB_RECURSE FORMAT_SRC_FILES  "${PROJECT_SOURCE_DIR}/**.h"  "${PROJECT_SOURCE_DIR}/**.cpp")
add_custom_target(clang_format COMMAND clang-format-7 -i ${FORMAT_SRC_FILES})

# Exposing package information...
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion
)

set(CBOR_INSTALL_EXPORT_CMAKE_DIR  "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}/")
set(CBOR_EXPORT_CMAKE_DIR "${CMAKE_CURRENT_BINARY_DIR}/")

# install all the files.
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    DESTINATION ${CBOR_INSTALL_EXPORT_CMAKE_DIR}
)
export(
  TARGETS cbor
  NAMESPACE Cbor::
  FILE ${CBOR_EXPORT_CMAKE_DIR}/${PROJECT_NAME}Config.cmake
)
install(TARGETS cbor EXPORT ${PROJECT_NAME}Config
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  )
install(DIRECTORY include/cbor/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(
  EXPORT ${PROJECT_NAME}Config
  FILE ${PROJECT_NAME}Config.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CBOR_INSTALL_EXPORT_CMAKE_DIR}
)