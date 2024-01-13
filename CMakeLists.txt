cmake_minimum_required(VERSION 3.0)
project(ekf)

set(CMAKE_CXX_STANDARD 17)

find_package(Gnuplot REQUIRED)
include_directories(${GNUPLOT_INCLUDE_DIRS})

find_package(Boost REQUIRED
COMPONENTS iostreams)


add_subdirectory(gnuplot-iostream)


add_executable(ekf src/main.cpp
        src/KalmanFilter.cpp
        include/KalmanFilter.h
        src/ExtendedKarmanFilter.cpp
        include/ExtendedKarmanFilter.h)

target_link_libraries(ekf Boost::iostreams)