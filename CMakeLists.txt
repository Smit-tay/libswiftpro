cmake_minimum_required(VERSION 3.22)
project(libswiftpro VERSION 0.2.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

find_package(Threads REQUIRED)

set(LIBUARM_SOURCES
    src/swiftpro.cc
    src/commander.cc
    src/receiver.cc
    src/utils/utils.cc
    src/serial/serial.cc
    src/serial/impl/unix.cc
    src/serial/impl/list_ports/list_ports_linux.cc
)

add_library(uarm        SHARED ${LIBUARM_SOURCES})
add_library(uarm_static STATIC ${LIBUARM_SOURCES})
set_target_properties(uarm_static PROPERTIES OUTPUT_NAME uarm)

foreach(_target uarm uarm_static)
    target_compile_options(${_target} PRIVATE -Wall -Wextra -Wpedantic)
    target_include_directories(${_target}
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    )
    target_link_libraries(${_target} PUBLIC Threads::Threads)
endforeach()

option(BUILD_EXAMPLES "Build example programs" ON)
if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

install(
    TARGETS uarm uarm_static
    EXPORT libswiftproTargets
    LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(
    DIRECTORY include/uarm include/serial
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
    EXPORT libswiftproTargets
    NAMESPACE libswiftpro::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/libuarm
)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/libswiftproConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/libswiftproConfig.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/libuarm
)

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/libswiftproConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(
    FILES
        "${CMAKE_CURRENT_BINARY_DIR}/libswiftproConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/libswiftproConfigVersion.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/libuarm
)
