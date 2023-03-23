# Serial Communication Library

This is a modified version of https://github.com/wjwwood/serial by
William Woodall, John Harrison. The goal is essentially to provide a cleaner
version and remove extra unnecessary dependencies like Python and Catkin.

This is a cross-platform library for interfacing with rs-232 serial like ports written in C++.
It provides a modern C++ interface with a workflow designed to look and feel like PySerial,
but with the speed and control provided by C++.

Serial is a class that provides the basic interface common to serial libraries (open, close, read, write, etc..)
and requires no extra dependencies. It also provides tight control over timeouts and control over handshaking lines. 

### Documentation

Website: WIP

API Documentation: WIP

### Dependencies

Required:
* [cmake](http://www.cmake.org) - buildsystem

Optional (for documentation):
* [Doxygen](http://www.doxygen.org/) - Documentation generation tool
* [graphviz](http://www.graphviz.org/) - Graph visualization software

### Install

Get the code:

    git clone https://github.com/JonathSpirit/serial.git

Prepare build:

    mkdir build && cd build

Build:

    cmake .. && cmake --build .

Build and run the tests:

    ctest

### License

[The MIT License](LICENSE)

### Authors

Original from :
- William Woodal
- John Harrison

Modified by :
- Guillaume Guillet
