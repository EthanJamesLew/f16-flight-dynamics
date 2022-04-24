# set the python project module
# you'll need to setup the boost macro for this too
set(PYPROJECTNAME f16dynpy)
# stop cmake from prefixing with the name lib
set(CMAKE_SHARED_LIBRARY_PREFIX "")

# python binding packages
find_package(Boost COMPONENTS python REQUIRED)
find_package(PythonLibs REQUIRED)

include_directories(${PYTHON_INCLUDE_DIRS})

# build and link
add_library(${PYPROJECTNAME} SHARED PyBinding.cpp)
target_link_libraries(${PYPROJECTNAME} ${PROJECT_NAME} ${PYTHON_LIBRARIES} Boost::python Boost::numpy)
