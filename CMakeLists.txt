# Copyright (C) 2018 Ben Smith
#
# This software may be modified and distributed under the terms
# of the MIT license.  See the LICENSE file for details.
cmake_minimum_required(VERSION 2.8)
project(binjgb)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

option(WERROR "Build with warnings as errors" OFF)

if (MSVC)
  add_definitions(-W3 -D_CRT_SECURE_NO_WARNINGS)
  if (WERROR)
    add_definitions(-WX)
  endif ()
else ()
  add_definitions(
    -Wall -Wextra -Wpointer-arith -Wno-unused-parameter -g
    -Wno-unused-function -Wno-unused-variable
    -Wno-implicit-fallthrough
  )
  if (WERROR)
    add_definitions(-Werror)
  endif ()
endif ()

function (target_copy_to_bin name)
add_custom_target(${name}-copy-to-bin ALL
  COMMAND ${CMAKE_COMMAND} -E make_directory ${PROJECT_SOURCE_DIR}/bin
  COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${name}> ${PROJECT_SOURCE_DIR}/bin
  DEPENDS ${name}
)
endfunction ()

add_executable(bagbe
  src/bagbe.cc
)

install(TARGETS bagbe DESTINATION bin)
target_copy_to_bin(bagbe)
