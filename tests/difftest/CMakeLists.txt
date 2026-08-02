# Runs the same scenarios against our libraries and compares their traces with golden traces recorded from
# the original Karma 1.2 libraries. Configured on its own (x86 only), this directory builds the scenarios
# against the reference libraries and records the golden traces:
#   cmake -S tests/difftest -B build-ref -A Win32
#   cmake --build build-ref --config Release --target golden
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    cmake_minimum_required(VERSION 3.20)
    project(karma_reference C CXX)
    if(NOT MSVC OR NOT CMAKE_SIZEOF_VOID_P EQUAL 4)
        message(FATAL_ERROR "The reference libraries are 32-bit MSVC builds. Configure with -A Win32.")
    endif()
    set(CMAKE_MSVC_RUNTIME_LIBRARY MultiThreadedDLL)
    # A git worktree has no submodule checkout, so point this at the main checkout's copy.
    set(KARMA_REFERENCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../reference/metoolkit-karma-v1.2 CACHE PATH "Karma 1.2 reference SDK")
    set(ref ${KARMA_REFERENCE_DIR}/metoolkit)
    add_library(karma INTERFACE)
    target_include_directories(karma INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/../../include)
    target_compile_definitions(karma INTERFACE WIN32)
    target_link_directories(karma INTERFACE ${ref}/lib.rel/win32_single_msvcrt)
    target_link_libraries(karma INTERFACE Mst Mdt MdtKea MdtBcl McdPrimitives McdCommon McdFrame MeGlobals user32)
    target_sources(karma INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/msvcrt_shim.c)
    enable_testing()
    set(recording ON)
endif()

add_executable(tracecmp tracecmp.cpp)
file(GLOB scenarios CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/*.c)
list(FILTER scenarios EXCLUDE REGEX "msvcrt_shim\\.c$")
foreach(src ${scenarios})
    get_filename_component(scenario ${src} NAME_WE)
    set(golden ${CMAKE_CURRENT_SOURCE_DIR}/golden/${scenario}.txt)
    add_executable(diff_${scenario} ${src})
    target_link_libraries(diff_${scenario} PRIVATE karma)
    add_test(NAME diff_${scenario} COMMAND tracecmp $<TARGET_FILE:diff_${scenario}> ${golden})
    set_tests_properties(diff_${scenario} PROPERTIES SKIP_RETURN_CODE 77 LABELS difftest)
    list(APPEND record COMMAND tracecmp --write $<TARGET_FILE:diff_${scenario}> ${golden})
endforeach()

if(recording)
    add_custom_target(golden ${record} DEPENDS tracecmp)
endif()
