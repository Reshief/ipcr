cmake_minimum_required(VERSION 2.8)# FATAL_ERROR)

if (POLICY CMP0048)
    cmake_policy(SET CMP0048 NEW)
endif (POLICY CMP0048)

project(iterative_closest_point VERSION 0.1.1)

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

# add_subdirectory(extern/eigen)
# add_subdirectory(extern/pcl)

find_package(PCL 1.13.1 REQUIRED)
# find_package(OpenCV REQUIRED)
find_package(OpenCV 4 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

###################################################################################################
# HEADER FILES
###################################################################################################
set(HEADERS
	include/cmaes.h
	include/parameters.h
	include/timings.h
	include/utils.h
	include/random.h
)

###################################################################################################
# SOURCE FILES
###################################################################################################
set(SOURCES
	src/main.cpp
)

add_executable (iterative_closest_point ${SOURCES})
target_include_directories(iterative_closest_point include/)
target_link_libraries (iterative_closest_point ${PCL_LIBRARIES} ${OpenCV_LIBS})