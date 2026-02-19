# arm-toolchain.cmake
# For cross compiling on PC for target rpi, using a strict sysroot so -nostdinc -nostdinc++

# 1. Tell CMake we are targeting a different OS/arch
set(CMAKE_SYSTEM_NAME Linux)            # the target OS
set(CMAKE_SYSTEM_PROCESSOR aarch64)        # arm, arm64, etc.

#  Path to the sysroot that contains a full Debian/Ubuntu aarch64
#  runtime (headers + libc + libraries).
#set(SYSROOT_DIR "$ENV{HOME}/sandboxes/rpios_bookworm_sysroot")
set(CMAKE_SYSROOT "${SYSROOT_DIR}")

# 2. Pick the compiler
set(CMAKE_C_COMPILER   aarch64-linux-gnu-gcc-12)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++-12)

# Tell CMake where to look for libraries & headers
#set(CMAKE_FIND_ROOT_PATH /usr/arm-linux-gnueabihf)
set(CMAKE_FIND_ROOT_PATH "${SYSROOT_DIR}")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# we must explicitly define system include dirs for strict cross compiling
include_directories( SYSTEM PRIVATE
    "${SYSROOT_DIR}/usr/lib/gcc/aarch64-linux-gnu/12/include"
    "${SYSROOT_DIR}/usr/include/aarch64-linux-gnu"
    "${SYSROOT_DIR}/usr/include/aarch64-linux-gnu/c++/12"
    "${SYSROOT_DIR}/usr/include/c++/12"
    "${SYSROOT_DIR}/usr/include"
)

# compiler flags
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} --sysroot ${SYSROOT_DIR} -nostdinc -march=armv8-a")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --sysroot ${SYSROOT_DIR} -nostdinc -nostdinc++ -march=armv8-a")

# linker flags
set(CMAKE_EXE_LINKER_FLAGS    "${CMAKE_EXE_LINKER_FLAGS} -Wl,--sysroot=${SYSROOT_DIR}")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--sysroot=${SYSROOT_DIR}")
set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} -Wl,--sysroot=${SYSROOT_DIR}")
