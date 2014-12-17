PROJECT(outlier_generator)
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

SET( CMAKE_VERBOSE_MAKEFILE ON)
SET( CMAKE_BUILD_TYPE Debug )

SET( BUILD_TESTS OFF )

add_subdirectory(Sophus)
add_subdirectory(SophusDistributions)

FIND_PACKAGE( SophusDistributions REQUIRED )

include_directories( ${SophusDistributions_INCLUDE_DIRS} )

set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 
find_package(Boost 1.46.1 COMPONENTS date_time program_options REQUIRED) 
include_directories(${Boost_INCLUDE_DIRS}) 


add_executable(outlier_generator src/outlier_generator.cpp)
target_link_libraries(outlier_generator ${Boost_LIBRARIES})
