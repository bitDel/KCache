cmake_minimum_required(VERSION 3.10)

project(CacheSystem)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

file(GLOB SOURCES "*.cpp")

add_executable(main ${SOURCES})

set_target_properties(main PROPERTIES CLEAN_DIRECT_OUTPUT 1)

# target_compile_options(main PRIVATE -Wall -Wextra -O2)