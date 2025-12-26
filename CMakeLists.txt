cmake_minimum_required(VERSION 3.16)
project(xmass_df_driver VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Options
option(BUILD_SHARED_LIBS "Build shared libraries" ON)
option(BUILD_TESTS "Build test programs" ON)
option(BUILD_EXAMPLES "Build example programs" ON)
option(WITH_SOAPYSDR "Build with SoapySDR support" ON)
option(WITH_USDR "Build with usdr-lib support" ON)

# Find required packages
find_package(Threads REQUIRED)

# Optional: SoapySDR
if(WITH_SOAPYSDR)
    find_package(SoapySDR CONFIG)
    if(SoapySDR_FOUND)
        message(STATUS "Found SoapySDR: ${SoapySDR_VERSION}")
        add_definitions(-DHAVE_SOAPYSDR)
    else()
        message(WARNING "SoapySDR not found, building without SoapySDR support")
    endif()
endif()

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/include)

# LMK05318B driver library
add_library(lmk05318b
    src/lmk05318b.c
)
target_include_directories(lmk05318b PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# xMASS carrier board library
add_library(xmass_carrier
    src/xmass_carrier.c
)
target_link_libraries(xmass_carrier
    lmk05318b
    Threads::Threads
)
target_include_directories(xmass_carrier PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# xMASS synchronization library
add_library(xmass_sync
    src/xmass_sync.c
)
target_link_libraries(xmass_sync
    xmass_carrier
    m
)
target_include_directories(xmass_sync PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Combined xMASS DF library
add_library(xmass_df INTERFACE)
target_link_libraries(xmass_df INTERFACE
    xmass_sync
    xmass_carrier
    lmk05318b
)

# Test programs
if(BUILD_TESTS)
    # LMK05318B test
    add_executable(test_lmk05318b tests/test_lmk05318b.c)
    target_link_libraries(test_lmk05318b lmk05318b)
    
    # Carrier board test
    add_executable(test_carrier tests/test_carrier.c)
    target_link_libraries(test_carrier xmass_carrier)
    
    # Sync test
    add_executable(test_sync tests/test_sync.c)
    target_link_libraries(test_sync xmass_sync)
endif()

# Example programs
if(BUILD_EXAMPLES)
    # Direction finding example
    add_executable(xmass_df_example examples/df_example.c)
    target_link_libraries(xmass_df_example xmass_sync)
    
    # Calibration tool
    add_executable(xmass_calibrate examples/calibrate.c)
    target_link_libraries(xmass_calibrate xmass_sync)
endif()

# Installation
install(TARGETS lmk05318b xmass_carrier xmass_sync
    EXPORT xmass_df_targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.h"
)

install(EXPORT xmass_df_targets
    FILE xmass_df_targets.cmake
    NAMESPACE xmass::
    DESTINATION lib/cmake/xmass_df
)

# Package configuration
include(CMakePackageConfigHelpers)

configure_package_config_file(
    ${CMAKE_SOURCE_DIR}/cmake/xmass_df_config.cmake.in
    ${CMAKE_BINARY_DIR}/xmass_df_config.cmake
    INSTALL_DESTINATION lib/cmake/xmass_df
)

write_basic_package_version_file(
    ${CMAKE_BINARY_DIR}/xmass_df_config_version.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(FILES
    ${CMAKE_BINARY_DIR}/xmass_df_config.cmake
    ${CMAKE_BINARY_DIR}/xmass_df_config_version.cmake
    DESTINATION lib/cmake/xmass_df
)

# Python module installation
install(FILES
    src/xmass_df.py
    src/xmass_soapy.py
    DESTINATION share/xmass_df/python
)

# Print configuration summary
message(STATUS "")
message(STATUS "xMASS Direction Finding Driver Configuration:")
message(STATUS "  Version:        ${PROJECT_VERSION}")
message(STATUS "  Build type:     ${CMAKE_BUILD_TYPE}")
message(STATUS "  Shared libs:    ${BUILD_SHARED_LIBS}")
message(STATUS "  Tests:          ${BUILD_TESTS}")
message(STATUS "  Examples:       ${BUILD_EXAMPLES}")
message(STATUS "  SoapySDR:       ${SoapySDR_FOUND}")
message(STATUS "")
