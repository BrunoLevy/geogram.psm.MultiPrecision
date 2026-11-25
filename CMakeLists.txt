cmake_minimum_required(VERSION 3.16)

project(geogram_multiprecision LANGUAGES CXX)

find_package(Threads REQUIRED)

# Public inline arithmetic uses globals that are not exported by Windows DLLs.
add_library(geogram_multiprecision STATIC MultiPrecision_psm.cpp MultiPrecision_psm.h)
add_library(geogram::multiprecision ALIAS geogram_multiprecision)

target_include_directories(geogram_multiprecision PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(geogram_multiprecision PUBLIC Threads::Threads)
target_compile_features(geogram_multiprecision PUBLIC cxx_std_17)
target_compile_definitions(geogram_multiprecision PUBLIC GEO_STATIC_LIBS)

# Exact predicates, including inline code in the public header, must not
# contract multiply-add operations into FMAs.
if(MSVC)
    target_compile_options(geogram_multiprecision PUBLIC /fp:strict)
elseif(CMAKE_CXX_COMPILER_ID MATCHES "^(GNU|Clang|AppleClang)$")
    target_compile_options(geogram_multiprecision PUBLIC -ffp-contract=off)
endif()

add_executable(geogram_multiprecision_example MultiPrecision_example.cpp)
target_link_libraries(geogram_multiprecision_example PRIVATE geogram::multiprecision)
