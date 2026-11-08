cmake_minimum_required(VERSION 3.16)
project(slam_practise CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Threads REQUIRED)

add_compile_options(-Wall -Wextra -Wpedantic)
set(SAN -fsanitize=address,undefined -fno-omit-frame-pointer)

# One executable per scratch/*.cpp (and interviewer/reference/*.cpp)
file(GLOB SOURCES CONFIGURE_DEPENDS
     ${CMAKE_SOURCE_DIR}/scratch/*.cpp
     ${CMAKE_SOURCE_DIR}/scratch/cpp/*.cpp
     ${CMAKE_SOURCE_DIR}/interviewer/reference/*.cpp)
foreach(src ${SOURCES})
  get_filename_component(name ${src} NAME_WE)
  add_executable(${name} ${src})
  target_include_directories(${name} PRIVATE ${CMAKE_SOURCE_DIR})
  target_link_libraries(${name} PRIVATE Eigen3::Eigen Threads::Threads)
  if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_options(${name} PRIVATE ${SAN})
    target_link_options(${name} PRIVATE ${SAN})
  endif()
endforeach()
