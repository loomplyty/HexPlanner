cmake_minimum_required (VERSION 2.8)
project (HexPlanner)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
 ########## include_directories ###############
include_directories("/usr/Aris_Dependent" )
 
#include_directories("/home/hex/Desktop/eigen" )

######### build project ###############
add_executable(main main.cpp Dynamics.cpp DynamicsBase.cpp DynamicsModelBase.cpp  solver.cpp matrix_support.cpp util.cpp ldl.cpp Planner.cpp)
