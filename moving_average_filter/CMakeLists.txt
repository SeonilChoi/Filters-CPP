cmake_minimum_required(VERSION 3.5)
project(moving_average_filter)

if (NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Release")
  add_compile_options(-O3 -march=native -DNDEBUG)
elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_compile_options(-g)
endif()

add_library(moving_average_filter_library INTERFACE)

target_include_directories(moving_average_filter_library INTERFACE "${PROJECT_SOURCE_DIR}/include")

install(DIRECTORY include/ DESTINATION include)
