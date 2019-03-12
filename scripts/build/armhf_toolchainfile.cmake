# from https://raw.githubusercontent.com/ros2-for-arm/ros2/master/aarch64_toolchainfile.cmake
# adapted for 32bit

# Copyright (c) 2018, ARM Limited.
# SPDX-License-Identifier: Apache-2.0

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_VERSION 1)
set(CMAKE_SYSTEM_PROCESSOR armhf)
# specify the cross compiler
set(CMAKE_C_COMPILER $ENV{CROSS_COMPILE}gcc)
set(CMAKE_CXX_COMPILER $ENV{CROSS_COMPILE}g++)
# where is the target environment
# CMAKE_FIND_ROOT_PATH should not be set to build a ros package
set(CMAKE_FIND_ROOT_PATH ${MAKE_CURRENT_SOURCE_DIR}/../../)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# This assumes that pthread will be available on the target system
# (this emulates that the return of the TRY_RUN is a return code "0"
set(THREADS_PTHREAD_ARG "0"
  CACHE STRING "Result from TRY_RUN" FORCE)
