cmake_minimum_required(VERSION 3.10)
project(serial)

#C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

if(APPLE)
    find_library(IOKIT_LIBRARY IOKit)
    find_library(FOUNDATION_LIBRARY Foundation)
endif()

# Sources
set(PROJECT_SOURCES
    src/serial.cpp
    include/serial/serial.hpp)

if(APPLE)
    list(APPEND PROJECT_SOURCES src/impl/unix.cpp)
    list(APPEND PROJECT_SOURCES src/impl/list_ports/list_ports_osx.cpp)
elseif(UNIX)
    list(APPEND PROJECT_SOURCES src/impl/unix.cpp)
    list(APPEND PROJECT_SOURCES src/impl/list_ports/list_ports_linux.cpp)
else()
    list(APPEND PROJECT_SOURCES src/impl/win.cpp)
    list(APPEND PROJECT_SOURCES src/impl/list_ports/list_ports_win.cpp)
endif()

# Add serial library
add_library(${PROJECT_NAME} ${PROJECT_SOURCES})
if(APPLE)
    target_link_libraries(${PROJECT_NAME} ${FOUNDATION_LIBRARY} ${IOKIT_LIBRARY})
elseif(UNIX)
    target_link_libraries(${PROJECT_NAME} pthread)
else()
    target_link_libraries(${PROJECT_NAME} setupapi)
endif()

# Uncomment for example
add_executable(${PROJECT_NAME}_example examples/serial_example.cpp)
add_dependencies(${PROJECT_NAME}_example ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}_example ${PROJECT_NAME})

# Include headers
include_directories(include)

# Install executable
install(TARGETS ${PROJECT_NAME})
