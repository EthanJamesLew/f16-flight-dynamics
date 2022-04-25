include(GNUInstallDirs)

# install headers so we can reference them differently in the cpp files
install(DIRECTORY f16_flight_dynamics/ DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}"
        COMPONENT ${PROJECT_NAME}
        FILES_MATCHING
        PATTERN "*.h"
        REGEX "/src$" EXCLUDE
        REGEX "/doc$" EXCLUDE
        REGEX "/tests$" EXCLUDE)


# include directories
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# collect source code and add it to the library
file(GLOB_RECURSE F16_SOURCE_CODE f16_flight_dynamics/*.cpp)
add_library(${PROJECT_NAME} SHARED ${F16_SOURCE_CODE})

# make interfaces so we can install the headers
target_include_directories(
        ${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

# install headers
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# install project and export targets
install(
        TARGETS ${PROJECT_NAME}
        EXPORT f16Targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

set(F16_CMAKE_PACKAGE_INSTALL_SUBDIR "share/${PROJECT_NAME}/cmake")

install(
        EXPORT f16Targets
        NAMESPACE ${PROJECT_NAME}::
        DESTINATION ${F16_CMAKE_PACKAGE_INSTALL_SUBDIR})

# library verisoning stuff
include(CMakePackageConfigHelpers)

write_basic_package_version_file(
        f16ConfigVersion.cmake
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY SameMinorVersion)

configure_package_config_file(
        "${PROJECT_SOURCE_DIR}/cmake/f16Config.cmake.in" f16Config.cmake
        INSTALL_DESTINATION ${F16_CMAKE_PACKAGE_INSTALL_SUBDIR})

install(FILES "${PROJECT_BINARY_DIR}/src/f16Config.cmake"
        "${PROJECT_BINARY_DIR}/src/f16ConfigVersion.cmake"
        DESTINATION ${F16_CMAKE_PACKAGE_INSTALL_SUBDIR})