cmake_minimum_required(VERSION 3.28)
project(aja)

set(SOURCES
    aja.cpp
    aja.h

    consumer/aja_consumer.cpp
    consumer/aja_consumer.h

    producer/aja_producer.cpp
    producer/aja_producer.h
)

casparcg_add_module_project(aja
    SOURCES ${SOURCES}
    INIT_FUNCTION "aja::init"
)

target_include_directories(aja PRIVATE
    ..
    ${FFMPEG_INCLUDE_PATH}
)

target_link_libraries(aja PRIVATE
    ajantv2
    ${FFMPEG_LIBRARIES}
)

# libajantv2 publishes its own compiler warning/optimization flags as PUBLIC
# usage requirements. Keep those flags for building libajantv2 itself, but do
# not propagate them into CasparCG targets that link against it.
set_property(TARGET ajantv2 PROPERTY INTERFACE_COMPILE_OPTIONS "")

set_target_properties(aja PROPERTIES FOLDER modules)
source_group(sources ./*)
source_group(sources\\consumer consumer/*)
source_group(sources\\producer producer/*)
