cmake_minimum_required(VERSION 3.16)
project(Manipulator)

set(CMAKE_CXX_STANDARD 14)

find_package(GSL REQUIRED)
add_executable(Manipulator Manipulator.cpp
        projects/Discretization.cpp
        projects/Spline.cpp
        projects/SerialCommunication.cpp
        projects/TrajectoryPlanning.cpp
        projects/SendOnTime.cpp
        projects/SerialCommunication.cpp
        projects/PackageData.cpp
        headfiles/PackageData.h
        headfiles/CommunicatinoBetween.h projects/CommunicatinoBetween.cpp)
target_link_libraries(Manipulator GSL::gsl)
