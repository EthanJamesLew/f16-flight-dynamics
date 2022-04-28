# set the python project module
# you'll need to setup the boost macro for this too
set(PYPROJECTNAME f16dynpy)

# install headers so we can reference them differently in the cpp files
install(DIRECTORY ../python-bindings DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${PYPROJECTNAME}"
        COMPONENT python-bindings
        FILES_MATCHING
        PATTERN "*.h"
        REGEX "/src$" EXCLUDE
        REGEX "/doc$" EXCLUDE
        REGEX "/tests$" EXCLUDE)

# stop cmake from prefixing with the name lib
set(CMAKE_SHARED_LIBRARY_PREFIX "")

# python binding packages
find_package(Boost COMPONENTS python REQUIRED)
find_package(PythonLibs REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/..)
include_directories(${PYTHON_INCLUDE_DIRS})

# collect source code and add it to the library
file(GLOB_RECURSE PB_SOURCE_CODE ./*.cpp)


if(DEFINED SKBUILD)
    # prevent an unused variable warning
    set(ignoreMe "${SKBUILD}")

    # call pybind11-config to obtain the root of the cmake package
    execute_process(COMMAND ${PYTHON_EXECUTABLE} -m pybind11 --cmakedir
            OUTPUT_VARIABLE pybind11_ROOT_RAW)
    string(STRIP ${pybind11_ROOT_RAW} pybind11_ROOT)
    find_package(pybind11)

    # make this into a python module
    pybind11_add_module(${PYPROJECTNAME} MODULE
            ${PB_SOURCE_CODE})

    # link to the shared objects
    target_link_libraries(${PYPROJECTNAME} PUBLIC ${PROJECT_NAME} Boost::python Boost::numpy)

    # Installing the extension module to the root of the package
    install(TARGETS ${PYPROJECTNAME} DESTINATION .)

    #configure_file("${PROJECT_SOURCE_DIR}/src/hello/__main__.py.in"
    #        "${PROJECT_BINARY_DIR}/src/hello/__main__.py")

    install(FILES "${PROJECT_SOURCE_DIR}/python-bindings/f16dynamics/__init__.py" DESTINATION .)

    # The extension module must load the hello library as a dependency when the
    # extension module is loaded. The easiest way to locate the hello library is
    # via RPATH. Absolute RPATHs are possible, but they make the resulting
    # binaries not redistributable to other Python installations (conda is broke,
    # wheel reuse is broke, and more!).
    #
    # Placing the hello library in the package and using relative RPATHs that
    # doesn't point outside of the package means that the built package is
    # relocatable. This allows for safe binary redistribution.
    if(APPLE)
        set_target_properties(
                ${PYPROJECTNAME} PROPERTIES INSTALL_RPATH "@loader_path/${CMAKE_INSTALL_LIBDIR}")
    else()
        set_target_properties(${PYPROJECTNAME} PROPERTIES INSTALL_RPATH
                "$ORIGIN/${CMAKE_INSTALL_LIBDIR}")
    endif()
else()
    # build and link
    add_library(${PYPROJECTNAME} SHARED ${PB_SOURCE_CODE})
    target_link_libraries(${PYPROJECTNAME} ${PROJECT_NAME} ${PYTHON_LIBRARIES} Boost::python Boost::numpy)
endif()
