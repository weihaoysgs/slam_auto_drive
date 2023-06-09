cmake_minimum_required(VERSION 3.10)
project(InertialNavigation)

add_executable(run_eskf RunESKF.cpp)
target_link_libraries(run_eskf glog)