cmake_minimum_required(VERSION 3.28)
project(aja)

set(AJA_ROOT /opt/src/libajantv2)
set(AJA_BUILD ${AJA_ROOT}/build)

set(SOURCES
    aja.cpp
    aja.h

    consumer/aja_consumer.cpp
    consumer/aja_consumer.h
)

casparcg_add_module_project(aja
    SOURCES ${SOURCES}
    INIT_FUNCTION "aja::init"
)

target_compile_definitions(aja PRIVATE
    AJALinux
    AJA_LINUX
    AJASTATIC
    AJA_USE_CPLUSPLUS11
    NTV2_USE_CPLUSPLUS11
)

target_include_directories(aja PRIVATE
    ..
    ${AJA_ROOT}
    ${AJA_ROOT}/ajaanc/includes
    ${AJA_ROOT}/ajabase
    ${AJA_ROOT}/ajantv2/includes
    ${AJA_BUILD}/ajantv2
    ${AJA_BUILD}/ajantv2/mbedtls-install/include
    ${AJA_ROOT}/ajantv2/src/lin
    ${AJA_ROOT}/ajabase/pnp/linux
    ${AJA_ROOT}/ajabase/system/linux
    ${AJA_ROOT}/thirdparty/include
)

target_link_libraries(aja PRIVATE
    ${AJA_BUILD}/ajantv2/libajantv2.a

    udev
    dl
    pthread
    rt

    ${AJA_BUILD}/ajantv2/mbedtls-install/lib/libmbedtls.a
    ${AJA_BUILD}/ajantv2/mbedtls-install/lib/libmbedcrypto.a
    ${AJA_BUILD}/ajantv2/mbedtls-install/lib/libmbedx509.a

    ${AJA_BUILD}/ajantv2/mbedtls-install/lib/libmbedtls.a
    ${AJA_BUILD}/ajantv2/mbedtls-install/lib/libmbedcrypto.a
    ${AJA_BUILD}/ajantv2/mbedtls-install/lib/libmbedx509.a
)

set_target_properties(aja PROPERTIES FOLDER modules)
source_group(sources ./*)
