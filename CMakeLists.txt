cmake_minimum_required(VERSION 3.20 FATAL_ERROR)

if (POLICY CMP0048)
    cmake_policy(SET CMP0048 NEW)
endif (POLICY CMP0048)

if(WIN32)
#	set(CMAKE_TOOLCHAIN_FILE ${VCPKG_PATH}/scripts/buildsystems/vcpkg.cmake)
	message("VCPKG buildsystem file: ${CMAKE_TOOLCHAIN_FILE}")
endif(WIN32)


project(iterative_closest_point VERSION 0.1.1)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin/)

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

find_package(PCL COMPONENTS REQUIRED)
#find_package(PCL 1.13.1 REQUIRED)
# find_package(OpenCV REQUIRED)
find_package(OpenCV COMPONENTS REQUIRED)
find_package(Qt5 COMPONENTS Core)
add_subdirectory(extern/cxxopts/)

include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# add automatic git version tracking
# based on: https://github.com/andrew-hardin/cmake-git-version-tracking
include(FetchContent)
FetchContent_Declare(cmake_git_version_tracking                   
  GIT_REPOSITORY https://github.com/andrew-hardin/cmake-git-version-tracking.git
  GIT_TAG 904dbda1336ba4b9a1415a68d5f203f576b696bb
)
FetchContent_MakeAvailable(cmake_git_version_tracking)


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

include_directories(include/)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (iterative_closest_point ${SOURCES})
target_include_directories(iterative_closest_point PUBLIC ${OpenCV_INCLUDE_DIRS} PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries (iterative_closest_point PRIVATE ${OpenCV_LIBS} cxxopts::cxxopts)
target_link_libraries (iterative_closest_point PRIVATE ${PCL_LIBRARIES} )
target_link_libraries(iterative_closest_point PRIVATE cmake_git_version_tracking)
# target_link_libraries(iterative_closest_point PRIVATE ${PCL_LIBRARIES} opencv_ml opencv_dnn opencv_core opencv_flann cxxopts::cxxopts)