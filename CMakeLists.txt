cmake_minimum_required(VERSION 3.20)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
    message(FATAL_ERROR "In-source builds are not allowed. Use: cmake -B build")
endif()

if(NOT DEFINED CMAKE_TOOLCHAIN_FILE AND DEFINED ENV{VCPKG_ROOT})
    set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" CACHE FILEPATH "")
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

find_package(Bullet CONFIG REQUIRED)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(KARMA_INCLUDE_DIR ${CMAKE_INSTALL_INCLUDEDIR}/karma)
add_library(karma_api INTERFACE)
target_include_directories(karma_api INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${KARMA_INCLUDE_DIR}>)
# The headers select MeReal and MEAPI from these platform macros, not from the compiler ones.
if(WIN32)
    target_compile_definitions(karma_api INTERFACE WIN32)
else()
    target_compile_definitions(karma_api INTERFACE LINUX)
endif()

add_library(karma INTERFACE)
add_library(karma::karma ALIAS karma)
install(TARGETS karma_api karma EXPORT karmaTargets)

function(karma_add_library name)
    cmake_parse_arguments(ARG "" "" "DEPS" ${ARGN})
    file(GLOB sources CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/${name}/*.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/${name}/*.c)
    add_library(${name} STATIC ${sources})
    target_include_directories(${name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/internal ${BULLET_INCLUDE_DIRS})
    target_link_libraries(${name} PUBLIC karma_api ${ARG_DEPS} PRIVATE ${BULLET_LIBRARIES})
    target_link_libraries(karma INTERFACE ${name})
    install(TARGETS ${name} EXPORT karmaTargets)
endfunction()

karma_add_library(MeGlobals)
karma_add_library(McdFrame DEPS MeGlobals)
karma_add_library(McdCommon DEPS McdFrame)
karma_add_library(McdPrimitives DEPS McdFrame)
karma_add_library(McdConvex DEPS McdPrimitives)
karma_add_library(Mdt DEPS MeGlobals)
karma_add_library(Mst DEPS McdCommon McdPrimitives Mdt)
karma_add_library(MeXML DEPS MeGlobals)
karma_add_library(MeAssetDB DEPS MeGlobals)
karma_add_library(MeAssetDBXMLIO DEPS MeAssetDB MeXML)
karma_add_library(MeAssetFactory DEPS Mst McdConvex MeAssetDB)
karma_add_library(MeViewer2)
karma_add_library(MeApp)
karma_add_library(MdtKea DEPS Mdt)
karma_add_library(MdtBcl DEPS MeGlobals)

set(KARMA_CMAKE_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/karma)
install(DIRECTORY include/ DESTINATION ${KARMA_INCLUDE_DIR})
install(EXPORT karmaTargets NAMESPACE karma:: DESTINATION ${KARMA_CMAKE_DIR})
configure_package_config_file(cmake/karmaConfig.cmake.in karmaConfig.cmake INSTALL_DESTINATION ${KARMA_CMAKE_DIR})
write_basic_package_version_file(karmaConfigVersion.cmake COMPATIBILITY SameMajorVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/karmaConfig.cmake ${CMAKE_CURRENT_BINARY_DIR}/karmaConfigVersion.cmake
    DESTINATION ${KARMA_CMAKE_DIR})

# Projects that embed this one with add_subdirectory do not build its tests by default.
string(COMPARE EQUAL "${CMAKE_SOURCE_DIR}" "${PROJECT_SOURCE_DIR}" KARMA_TOP_LEVEL)
option(KARMA_BUILD_TESTS "Build the tests" ${KARMA_TOP_LEVEL})
if(KARMA_BUILD_TESTS)
    enable_testing()
    set(KARMA_REFERENCE_DIR ${PROJECT_SOURCE_DIR}/reference/metoolkit-karma-v1.2 CACHE PATH "Karma 1.2 reference SDK")
    add_subdirectory(tests)
endif()
