cmake_minimum_required(VERSION 2.6)
project(conc_pl)

find_package(PCL 1.7 REQUIRED)
SET( CMAKE_CXX_FLAGS "-std=c++11 -O3")

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(conc_pl main.cpp)

target_link_libraries (conc_pl ${PCL_LIBRARIES})

install(TARGETS conc_pl RUNTIME DESTINATION bin)
