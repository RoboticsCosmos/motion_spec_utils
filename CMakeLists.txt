cmake_minimum_required(VERSION 3.15.0)
project(motion_spec_utils VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_INSTALL_DIR share/${PROJECT_NAME}/cmake)

add_compile_definitions(_OS_UNIX)

# add path to CMAKE_PREFIX_PATH
list(APPEND CMAKE_PREFIX_PATH /home/batsy/rc/build/)
list(APPEND CMAKE_MODULE_PATH /home/batsy/rc/build/)

find_package(Eigen3 REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kinova_mediator REQUIRED)

# append orocos include dirs
list(APPEND local_INCLUDE_DIRS ${CMAKE_INSTALL_PREFIX}/include/)

# includes
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${local_INCLUDE_DIRS}

  # kinova
  ${CMAKE_INSTALL_PREFIX}/include/kinova_api/
  ${CMAKE_INSTALL_PREFIX}/include/kinova_api/client
  ${CMAKE_INSTALL_PREFIX}/include/kinova_api/common
  ${CMAKE_INSTALL_PREFIX}/include/kinova_api/messages
  ${CMAKE_INSTALL_PREFIX}/include/kinova_api/client_stubs
  ${CMAKE_INSTALL_PREFIX}/include/kinova_api/google
)

# link directories
link_directories(${CMAKE_INSTALL_PREFIX}/lib/)

# add library
add_library(math_utils SHARED src/math_utils.cpp)
target_include_directories(math_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(math_utils orocos-kdl)

add_library(mutils SHARED src/utils.cpp)
target_include_directories(mutils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mutils 
  kdl_parser 
  orocos-kdl 
  kinova_mediator
  KortexApiCpp
  pthread
)

add_library(solver_utils SHARED src/solver_utils.cpp)
target_include_directories(solver_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(solver_utils orocos-kdl)

# install
install(
  TARGETS math_utils mutils solver_utils
  EXPORT ${PROJECT_NAME}-targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# Generate the CMake config file
include(CMakePackageConfigHelpers)
# Generate the version file accompanying the project's config file
configure_package_config_file(${PROJECT_NAME}-config.cmake.in
                              ${PROJECT_NAME}-config.cmake
                              INSTALL_DESTINATION ${CMAKE_INSTALL_DIR})

write_basic_package_version_file(${PROJECT_NAME}-config-version.cmake
                                 VERSION ${PROJECT_VERSION}
                                 COMPATIBILITY AnyNewerVersion)

# Install the config and configversion
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
        DESTINATION ${CMAKE_INSTALL_DIR})

# Make the targets accessible from this packages's build tree
export(
  EXPORT ${PROJECT_NAME}-targets
  NAMESPACE ${PROJECT_NAME}::
  FILE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake"
)

# Make the targets accessible from this packages's install tree
install(
  EXPORT ${PROJECT_NAME}-targets
  FILE ${PROJECT_NAME}-targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CMAKE_INSTALL_DIR})

# Install all public header files
install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp"
)