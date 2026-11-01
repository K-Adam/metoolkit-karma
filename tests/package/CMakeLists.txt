# Builds a consumer against an installed karma, to check the exported package.
cmake_minimum_required(VERSION 3.20)
project(karma_consumer LANGUAGES C CXX)

find_package(karma CONFIG REQUIRED)
add_executable(consumer ../c_smoke.c)
target_link_libraries(consumer PRIVATE karma::karma)
