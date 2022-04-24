# install headers so we can reference them differently in the cpp files
install(DIRECTORY f16_flight_dynamics/ DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}"
        COMPONENT ${PROJECT_NAME}
        FILES_MATCHING
        PATTERN "*.h"
        REGEX "/src$" EXCLUDE
        REGEX "/doc$" EXCLUDE
        REGEX "/tests$" EXCLUDE)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# collect source code and add it to the library
file(GLOB_RECURSE F16_SOURCE_CODE f16_flight_dynamics/*.cpp)
add_library(${PROJECT_NAME} SHARED ${F16_SOURCE_CODE})

# expose the headers to the other pieces
target_include_directories (${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})