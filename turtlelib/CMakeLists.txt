cmake_minimum_required(VERSION 3.22)
project(turtlelib) # TODO

find_package(Doxygen)

# Building documentation should be optional.
# To build documentation pass -DBUILD_DOCS=ON when generating the build system
option(BUILD_DOCS "Build the documentation" OFF)

if(NOT CMAKE_BUILD_TYPE)        # If no build type is set, set one
  set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Type of build." FORCE)
endif()

# build the documentation
if(${DOXYGEN_FOUND} AND ${BUILD_DOCS})
    # Turn the README.md into the homepage of the doxygen docs
    set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md)

    # Tell Doxygen where to find the documentation
    doxygen_add_docs(doxygen include/ src/ README.md ALL)

    # The documentation will be in the build/html directory
    # The main page is build/html/index.html
endif()


# Create a library.  Can specify if it is shared or static but usually
# you don't need or want to.
# name is the name of the library without the extension or lib prefix
# name creates a cmake "target" (in this case the target is called "libname"

add_library(turtlelib src/geometry2d.cpp src/se2d.cpp src/wheels.cpp src/diff_drive.cpp)

# Use target_include_directories so that #include"mylibrary/header.hpp" works
# The use of the <BUILD_INTERFACE> and <INSTALL_INTERFACE> is because when
# using the library from the build directory or after installation the header
# files are actually in different locations.
# During the build, the headers are read from the source code directory
# When used from the installed location, headers are in the
# system include/ directory
target_include_directories(turtlelib
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
    $<INSTALL_INTERFACE:include/>)

include(CTest)

# CTest sets BUILD_TESTING to on. To disable tests add -DBUILD_TESTING=OFF when invoking cmake
if(BUILD_TESTING)
    # Find the Unit testing framework. In this example, Catch2
    find_package(Catch2 3 REQUIRED)

    # A test is just an executable that is linked against the unit testing library
    add_executable(test_turtlelib
                   tests/test_angle.cpp
                   tests/test_wheels.cpp
                   tests/test_geometry2d.cpp
                   tests/test_se2d.cpp)


    target_link_libraries(test_turtlelib Catch2::Catch2WithMain turtlelib)
    # register the test with CTest, telling it what executable to run
    add_test(NAME check_turtlelib COMMAND test_turtlelib)

endif()

target_compile_options(turtlelib PUBLIC -Wall -Wextra -pedantic)


target_compile_features(turtlelib PUBLIC cxx_std_23)

# Create an executable from the following source code files
# The Name of the executable creates a cmake "target"
add_executable(Converter src/converter.cpp)
add_executable(Manual_tester src/tester.cpp)


target_link_libraries(Converter turtlelib)
target_link_libraries(Manual_tester turtlelib)


# install the include files by copying the whole include directory
install(DIRECTORY include/ DESTINATION include)

install(TARGETS Converter Manual_tester turtlelib EXPORT turtlelib-targets)


install(EXPORT turtlelib-targets
        FILE turtlelib-config.cmake
        NAMESPACE turtlelib::
        DESTINATION lib/cmake/${PROJECT_NAME})