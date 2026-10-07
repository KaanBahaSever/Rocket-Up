cmake_minimum_required(VERSION 3.16)
project(RocketUp VERSION 0.1.0 DESCRIPTION "Modular rocket flight simulator" LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(ROCKETUP_BUILD_EXAMPLES "Build example programs" ON)
option(ROCKETUP_BUILD_APPS "Build the rocketup command line tool" ON)
option(ROCKETUP_BUILD_TESTS "Build unit tests" ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# ---------------------------------------------------------------- dependencies
# MathRix (https://github.com/KaanBahaSever/MathRix): git submodule, or fetched when missing.
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/libs/MathRix/CMakeLists.txt)
    add_subdirectory(libs/MathRix)
else()
    include(FetchContent)
    FetchContent_Declare(MathRix GIT_REPOSITORY https://github.com/KaanBahaSever/MathRix.git GIT_TAG v0.2.0)
    FetchContent_MakeAvailable(MathRix)
endif()

add_library(nlohmann_json_vendored INTERFACE)
target_include_directories(nlohmann_json_vendored SYSTEM INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/nlohmann/include>)

find_package(Threads REQUIRED)

# ---------------------------------------------------------------- core library
file(GLOB_RECURSE ROCKETUP_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB_RECURSE ROCKETUP_HEADERS CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/include/rocketup/*.hpp)

file(READ ${CMAKE_CURRENT_SOURCE_DIR}/docs/logo.svg ROCKETUP_LOGO_SVG)
configure_file(cmake/Logo.hpp.in ${CMAKE_CURRENT_BINARY_DIR}/generated/rocketup/Logo.hpp @ONLY)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/docs/logo.svg)

add_library(rocketup STATIC ${ROCKETUP_SOURCES} ${ROCKETUP_HEADERS})
add_library(rocketup::rocketup ALIAS rocketup)
target_include_directories(rocketup PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_include_directories(rocketup PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/generated)
target_link_libraries(rocketup PUBLIC MathRix::MathRix nlohmann_json_vendored Threads::Threads)
target_compile_definitions(rocketup PUBLIC ROCKETUP_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")

if(MSVC)
    target_compile_options(rocketup PRIVATE /W4 /permissive- /utf-8)
    target_compile_definitions(rocketup PUBLIC _CRT_SECURE_NO_WARNINGS NOMINMAX)
else()
    target_compile_options(rocketup PRIVATE -Wall -Wextra -Wpedantic -Wshadow)
endif()

# ---------------------------------------------------------------- tools, examples, tests
if(ROCKETUP_BUILD_APPS)
    add_subdirectory(apps)
endif()
if(ROCKETUP_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()
if(ROCKETUP_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

install(TARGETS rocketup ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY data/ DESTINATION share/rocketup/data)
