cmake_minimum_required(VERSION 3.20)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
    message(FATAL_ERROR "In-source builds are not allowed. Use: cmake -B build")
endif()

project(karma
    VERSION 1.2
    DESCRIPTION "Re-implementation of the MathEngine Karma 1.2 physics engine API"
    LANGUAGES C CXX)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_library(karma INTERFACE)
add_library(karma::karma ALIAS karma)
target_include_directories(karma INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
