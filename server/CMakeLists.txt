#picopter cmake configuration file

cmake_minimum_required (VERSION 2.6)
project (picopter)

#Find prerequisite packages
#find_package (OpenCV REQUIRED)

#Set src and include folders
set (PI_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src)
set (PI_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/include)
#Set the output folder
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

#Set the included files
set (SOURCE
	 ${PI_SRC}/log.cpp
	 ${PI_SRC}/picopter.cpp
)
set (HEADERS
	 ${PI_INCLUDE}/common.h
	 ${PI_INCLUDE}/log.h 
)

#Generate the configure file
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