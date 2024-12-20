# Require CMake 3.16
cmake_minimum_required(VERSION 3.16)

# Require C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall -Wextra)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

project(nema CXX)

# Set include directories
include_directories(${PROJECT_SOURCE_DIR}/include)

# Set source files
set(SOURCES
    src/main.cpp
    src/common/bswap.cpp
    src/common/file.cpp
    src/hw/cpu.cpp
    src/hw/dma.cpp
    src/sys/emulator.cpp
    src/sys/memory.cpp
)

# Set header files
set(HEADERS
    include/common/bswap.hpp
    include/common/file.hpp
    include/common/types.hpp
    include/hw/cpu.hpp
    include/hw/dma.hpp
    include/sys/config.hpp
    include/sys/emulator.hpp
    include/sys/memory.hpp
)

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})
