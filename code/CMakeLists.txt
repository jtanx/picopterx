#picopter cmake configuration file

cmake_minimum_required (VERSION 2.6)
project (picopter)

#Text colours
string (ASCII 27 Esc)
set (ColourReset "${Esc}[m")
set (ColourBold  "${Esc}[1m")
set (Red         "${Esc}[31m")
set(Green       "${Esc}[32m")
set(Yellow      "${Esc}[33m")

#Includes
include (CheckCSourceCompiles)
include (FindPkgConfig)

#Versioning
execute_process (COMMAND git rev-parse HEAD 
				 OUTPUT_STRIP_TRAILING_WHITESPACE
				 OUTPUT_VARIABLE PICOPTER_VERSION
)
execute_process (COMMAND git  show -s --format=%ci HEAD
				 OUTPUT_STRIP_TRAILING_WHITESPACE
				 OUTPUT_VARIABLE PICOPTER_DATE
)

#Find prerequisite packages
#find_package (OpenCV REQUIRED)

#Check for gpsd
pkg_check_modules (LIBGPS libgps)
include_directories (${LIBGPS_INCLUDE_DIRS})
link_directories (${LIBGPS_LIBRARY_DIRS})
set (IS_REAL_LIBGPS ${LIBGPS_FOUND})

#Set src and include folders
set (PI_SRC ${CMAKE_BINARY_DIR}/src CACHE INTERNAL "PI_SRC")
set (PI_INCLUDE ${CMAKE_BINARY_DIR}/include)
set (PI_EMULATE ${CMAKE_BINARY_DIR}/emulation)
set (PI_DOC ${CMAKE_BINARY_DIR}/doc)

#Set the output folder
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

#Documentation (if Doxygen is installed)
set_directory_properties (PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES ${PI_DOC})
find_package (Doxygen QUIET)
find_package (latex QUIET)
if (DOXYGEN_FOUND)
	if (LATEX_COMPILER)
	add_custom_target (doc
		${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
		WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
		COMMAND make -C ${CMAKE_CURRENT_BINARY_DIR}/doc/latex
		COMMENT "Generating code documentation..." VERBATIM
	)
	else(LATEX_COMPILER)
	add_custom_target (doc
		${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
		WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
		COMMENT "Generating code documentation..." VERBATIM
	)
	endif(LATEX_COMPILER)
endif(DOXYGEN_FOUND)

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
	message (WARNING "${Yellow}Using the emulation files - ${ColourBold}remember to check on the RPi!${ColourReset}")
elseif (NOT LIBGPS_FOUND)
	message (FATAL_ERROR "${Red}Building on the RPi without gpsd${ColourReset}")
endif()

if (NOT LIBGPS_FOUND)
	message (WARNING "${Yellow}Emulating libgps - beware bogus values!${ColourReset}")
	include_directories(${PI_EMULATE}/gps-emu)
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
set (GCC_LINK_FLAGS "-pthread")
set (CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COMPILE_FLAGS}")
set (CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_LINK_FLAGS}")

#Pretty filenames (see log.h)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__FILENAME__='\"$(subst src/,,$<)\"'")

#Setup the testing framework (gtest)
if (test)
	add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/third_party/googletest)
	enable_testing()
	include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})
	add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/test)
endif()

#Add the base module
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/base)

#Add in the server executable
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/server)