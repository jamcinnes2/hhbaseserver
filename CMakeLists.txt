cmake_minimum_required(VERSION 3.18)

# make sure cross compiler sysroot is set
set(SYSROOT_DIR "" CACHE PATH "cross sysroot directory")
if(NOT SYSROOT_DIR)
  if(DEFINED ENV{SYSROOT_DIR})
    set(SYSROOT_DIR "$ENV{SYSROOT_DIR}" CACHE PATH "" FORCE)
  else()
    message(FATAL_ERROR "SYSROOT_DIR not defined.  Set it on the command line or as an env var.")
  endif()
endif()
message(STATUS "Using sysroot: ${SYSROOT_DIR}")

# create the project
project(rpi-sx1262 LANGUAGES CXX)

# when using debuggers such as gdb, the following line can be used
#set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_BUILD_TYPE Release)

# add these CMake based projects that it can build
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/libs/RadioLib" "${CMAKE_CURRENT_BINARY_DIR}/RadioLib")
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/libs/fmt-9.1.0" "${CMAKE_CURRENT_BINARY_DIR}/fmt-9.1.0")

# add these generic dependency projects
include_directories(
    "${CMAKE_CURRENT_SOURCE_DIR}/libs/Crow/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/libs/json/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/libs/cpp-httplib"
    "${CMAKE_CURRENT_SOURCE_DIR}/libs/lgpio"
    "${CMAKE_CURRENT_SOURCE_DIR}/libs/openssl-3.0.15/include"
)

# find cross compiled libs in the source dir
find_library(
    LGPIO_LIB
    NAMES lgpio gpio liblgpio liblgpio.so
    HINTS "${CMAKE_CURRENT_SOURCE_DIR}/libs/lgpio"
    NO_CMAKE_FIND_ROOT_PATH
)

find_library(
    SSL_LIB
    NAMES ssl
    HINTS "${CMAKE_CURRENT_SOURCE_DIR}/libs/openssl-3.0.15"
    NO_CMAKE_FIND_ROOT_PATH
)

find_library(
    CRYPTO_LIB
    NAMES crypto
    HINTS "${CMAKE_CURRENT_SOURCE_DIR}/libs/openssl-3.0.15"
    NO_CMAKE_FIND_ROOT_PATH
)

# add the executable
add_executable(${PROJECT_NAME} main.cpp lowhop.cpp loracomms.cpp webservice.cpp forwarddata.cpp)

# link libraries
target_link_libraries(${PROJECT_NAME} PRIVATE RadioLib fmt ${LGPIO_LIB} ${SSL_LIB} ${CRYPTO_LIB})
