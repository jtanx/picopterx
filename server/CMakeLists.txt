#picopter cmake configuration file

cmake_minimum_required (VERSION 2.6)
project (picopter)

#Includes
include(CheckCSourceCompiles)

#Versioning
execute_process (COMMAND git rev-parse HEAD OUTPUT_VARIABLE PICOPTER_VERSION)
execute_process (COMMAND git  show -s --format=%ci HEAD
				 OUTPUT_STRIP_TRAILING_WHITESPACE
				 OUTPUT_VARIABLE PICOPTER_DATE
)

#Find prerequisite packages
#find_package (OpenCV REQUIRED)

#Set src and include folders
set (PI_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src)
set (PI_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/include)
set (PI_EMULATE ${CMAKE_CURRENT_SOURCE_DIR}/emulation)
#Set the output folder
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

#Set the included files
set (SOURCE
	 ${PI_SRC}/log.cpp
	 ${PI_SRC}/picopter.cpp
	 ${PI_SRC}/gpio.cpp
)
set (HEADERS
	 ${PI_INCLUDE}/common.h
	 ${PI_INCLUDE}/log.h 
	 ${PI_INCLUDE}/gpio.h
	 ${PI_INCLUDE}/picopter.h
)

#Check if we're building on the pi
CHECK_C_SOURCE_COMPILES ("
	#include <stdio.h>
	#ifndef __arm__
	#error Not building on the pi
	#endif
	int main(){return 0;}
" IS_ON_PI)

if (NOT IS_ON_PI)
	include_directories (${PI_EMULATE})
endif()

#Generate the config file
if (UNIX AND NOT CYGWIN)
	set (USE_SYSLOG 1)
endif()
configure_file (${PI_INCLUDE}/config.h.in ${PI_INCLUDE}/config.h)

#Set the include folders for the compiler
include_directories (${PI_INCLUDE})

#Set compiler flags
set (GCC_COMPILE_FLAGS "-std=c++11 -Wall -pedantic -g")
set (GCC_LINK_FLAGS "")
set (CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COMPILE_FLAGS}")
set (CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_LINK_FLAGS}")

#Pretty filenames (see log.h)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__FILENAME__='\"$(subst src/,,$<)\"'")

#Make the executable
add_executable(picopter ${HEADERS} ${SOURCE})