cmake_minimum_required(VERSION 3.10.0)
project(kdl_utils VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_INSTALL_DIR share/${PROJECT_NAME}/cmake)

find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)

# includes
include_directories(
  include
  ${orocos_kdl_INCLUDE_DIRS}
)

# add library
add_library(math_utils SHARED src/math_utils.cpp)
target_include_directories(math_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${orocos_kdl_INCLUDE_DIRS}
)
target_link_libraries(math_utils ${orocos_kdl_LIBRARIES})

add_library(tf_utils SHARED src/tf_utils.cpp)
target_include_directories(tf_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${orocos_kdl_INCLUDE_DIRS}
)
target_link_libraries(tf_utils ${orocos_kdl_LIBRARIES})

add_library(mutils SHARED src/utils.cpp)
target_include_directories(mutils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${orocos_kdl_INCLUDE_DIRS}
  
)
target_link_libraries(mutils ${orocos_kdl_LIBRARIES})

add_library(solver_utils SHARED src/solver_utils.cpp)
target_include_directories(solver_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${orocos_kdl_INCLUDE_DIRS}
)
target_link_libraries(solver_utils ${orocos_kdl_LIBRARIES})

# install
install(
  TARGETS math_utils tf_utils mutils
  EXPORT ${PROJECT_NAME}-targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp"
)

# Make the targets accessible from this packages's install tree
install(
  EXPORT ${PROJECT_NAME}-targets
  FILE ${PROJECT_NAME}-targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CMAKE_INSTALL_DIR})

# Generate the CMake config file
include(CMakePackageConfigHelpers)
write_basic_package_version_file(${PROJECT_NAME}-config-version.cmake
                                 VERSION ${PROJECT_VERSION}
                                 COMPATIBILITY AnyNewerVersion)

# Generate the version file accompanying the project's config file
configure_package_config_file(${PROJECT_NAME}-config.cmake.in
                              ${PROJECT_NAME}-config.cmake
                              INSTALL_DESTINATION ${CMAKE_INSTALL_DIR})

# Install the config and configversion
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
        DESTINATION ${CMAKE_INSTALL_DIR})