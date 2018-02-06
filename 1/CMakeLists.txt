cmake_minimum_required(VERSION 2.8.3)
project(sayhello)
set(CMAKE_BUILD_TYPE "Release")

add_library(libhello hello.cpp)
add_executable(hello useHello.cpp)

target_link_libraries(hello libhello)

install(TARGETS hello DESTINATION /usr/local/lib)
install(FILES hello.h DESTINATION /usr/local/include)

