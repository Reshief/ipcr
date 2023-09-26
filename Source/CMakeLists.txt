cmake_minimum_required(VERSION 2.8)# FATAL_ERROR)

project(iterative_closest_point)

###################################################################################################
# PREPROCESSOR DEFINITIONS
###################################################################################################
if (CMAKE_CL_64)
 add_definitions(-DWIN64)
 set(PLATFORM  x64)
else (CMAKE_CL_64) 
 set(PLATFORM  x86)
endif (CMAKE_CL_64) 

if(WIN32)
  ADD_DEFINITIONS(-D_WIN32_WINNT=0x0501)
endif(WIN32)

set(CMAKE_AUTOMOC ON)


find_package(PCL 1.6 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

###################################################################################################
# HEADER FILES
###################################################################################################
set(HEADERS
	src/cmaes.h
	src/parameters.h
	src/timings.h
	src/utils.h
	src/random.h
)

###################################################################################################
# SOURCE FILES
###################################################################################################
set(SOURCES
	src/main.cpp
)

add_executable (iterative_closest_point ${HEADERS} ${SOURCES})
target_link_libraries (iterative_closest_point ${PCL_LIBRARIES} ${OpenCV_LIBS})