cmake_minimum_required(VERSION 3.18)
project(route_frame_regression LANGUAGES CXX)

find_path(Boost_INCLUDE_DIR boost/version.hpp HINTS ${BOOST_ROOT} ENV BOOST_ROOT REQUIRED)
enable_testing()
add_executable(route_frame_regression
    route_frame_regression.cpp
    ../src/core/frame/frame.cpp
    ../src/core/frame/geometry.cpp
    ../src/core/frame/draw_frame.cpp
    ../src/core/frame/frame_transform.cpp
    ../src/common/utf.cpp
    ../src/common/tweener.cpp)
target_compile_features(route_frame_regression PRIVATE cxx_std_17)
target_compile_definitions(route_frame_regression PRIVATE BOOST_ALL_NO_LIB)
target_include_directories(route_frame_regression PRIVATE ../src ${Boost_INCLUDE_DIR})
if(APPLE)
    target_compile_definitions(route_frame_regression PRIVATE BOOST_STACKTRACE_GNU_SOURCE_NOT_REQUIRED)
endif()
# The server normally supplies common headers through StdAfx.h.
if(MSVC)
    target_compile_options(route_frame_regression PRIVATE /FIboost/algorithm/string.hpp)
else()
    target_compile_options(route_frame_regression PRIVATE -include boost/algorithm/string.hpp)
endif()
add_test(NAME route_frame_regression COMMAND route_frame_regression)
