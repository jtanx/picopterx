#picopter cmake configuration file

cmake_minimum_required (VERSION 2.8)
project (picopter)

#Text colours
string (ASCII 27 Esc)
set (ColourReset "${Esc}[m")
set (ColourBold  "${Esc}[1m")
set (Red         "${Esc}[31m")
set (Green       "${Esc}[32m")
set (Yellow      "${Esc}[33m")

#Includes
include (CheckCSourceCompiles)
include (FindPkgConfig)
include (ExternalProject)

#Versioning
execute_process (COMMAND git rev-parse HEAD 
				 OUTPUT_STRIP_TRAILING_WHITESPACE
				 OUTPUT_VARIABLE PICOPTER_VERSION
)
execute_process (COMMAND git  show -s --format=%ci HEAD
				 OUTPUT_STRIP_TRAILING_WHITESPACE
				 OUTPUT_VARIABLE PICOPTER_DATE
)

#Set src and include folders
set (PI_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src)
set (PI_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/include)
set (PI_EMULATE ${CMAKE_CURRENT_SOURCE_DIR}/emulation)
set (PI_DOC ${CMAKE_CURRENT_SOURCE_DIR}/doc)
set (PI_THIRD_PARTY ${CMAKE_CURRENT_SOURCE_DIR}/third_party)

#Set the output folder
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

#Find prerequisite packages
find_package (OpenCV REQUIRED)
include_directories (${OpenCV_INCLUDE_DIRS})
link_directories (${OpenCV_LIBRARY_DIRS})

#Check for gpsd
pkg_check_modules (LIBGPS libgps)
include_directories (${LIBGPS_INCLUDE_DIRS})
link_directories (${LIBGPS_LIBRARY_DIRS})
set (IS_REAL_LIBGPS ${LIBGPS_FOUND})

#Set XSens stuff
if (NOT DISABLE_NONFREE)
	set (XSENS_FOLDER "${CMAKE_BINARY_DIR}/third_party/Xsens")
	set (XSENS_REPO "https://github.com/jtanx/Xsens")
	
	if (NOT EXISTS "${XSENS_FOLDER}")
		execute_process (COMMAND git clone ${XSENS_REPO} "${XSENS_FOLDER}"
						 RESULT_VARIABLE retval)
		if (NOT "${retval}" STREQUAL "0")
			message (FATAL_ERROR "${Red}Could not clone the Xsens repository.${ColourReset}")
		endif ()
	endif ()
	add_subdirectory (${XSENS_FOLDER})
	include_directories (${XSENS_FOLDER})
endif (NOT DISABLE_NONFREE)

#Include rapidjson
include_directories (${PI_THIRD_PARTY}/rapidjson)

#Documentation (if Doxygen is installed)
set_directory_properties (PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES ${PI_DOC})
find_package (Doxygen QUIET)
find_package (latex QUIET)
if (DOXYGEN_FOUND)
	if (LATEX_COMPILER)
	add_custom_target (doc
		${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		COMMAND make -C "${PI_DOC}/latex"
		COMMENT "Generating code documentation..." VERBATIM
	)
	else(LATEX_COMPILER)
	add_custom_target (doc
		${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
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

if (NOT IS_ON_PI OR FORCE_EMULATION)
	include_directories (${PI_EMULATE})
	message (WARNING "${Yellow}Using the emulation files - ${ColourBold}remember to check on the RPi!${ColourReset}")
else ()
	if (NOT LIBGPS_FOUND)
		message (FATAL_ERROR "${Red}Building on the RPi without gpsd${ColourReset}")
	endif (NOT LIBGPS_FOUND)

	CHECK_LIBRARY_EXISTS (wiringPi digitalWrite "" WIRINGPI_FOUND)
	if (NOT WIRINGPI_FOUND)
		message (FATAL_ERROR "${Red}Building on the RPi without wiringPi${ColourReset}")
	endif (NOT WIRINGPI_FOUND)
	
	message (STATUS "Checking for ServoBlaster...")
	execute_process (COMMAND /usr/local/sbin/servod --help
					 OUTPUT_QUIET
					 RESULT_VARIABLE SERVOBLASTER_ERROR)
	if (SERVOBLASTER_ERROR)
		message (FATAL_ERROR "${Red}ServoBlaster check failed! Ensure it is installed.${ColourReset}")
	else (SERVOBLASTER_ERROR)
		message (STATUS "ServoBlaster found!")
	endif (SERVOBLASTER_ERROR)
endif ()

if (NOT LIBGPS_FOUND)
	message (WARNING "${Yellow}Emulating libgps - beware bogus values!${ColourReset}")
	include_directories(${PI_EMULATE}/gps-emu)
endif()

#Australian tuning parameters
#set (IS_AUSTRALIAN_TUNED ON CACHE BOOL "Enable Australia-specific tuning (default: on)")

#Set the include folders for the compiler
include_directories (${PI_INCLUDE})

#Set compiler flags
set (GCC_COMPILE_FLAGS "-std=c++11 -Wall -pedantic -g")
set (GCC_LINK_FLAGS "-pthread")
set (CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COMPILE_FLAGS}")
set (CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_LINK_FLAGS}")

#Pretty filenames (see log.h) and log filepath
if (NOT MSYS AND NOT CYGWIN)
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__FILENAME__='\"$(subst ${CMAKE_SOURCE_DIR}/src/,,$(abspath $<))\"'")
	set (PICOPTER_LOG_LOCATION $ENV{HOME})
else ()
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__FILENAME__='\"$(subst ${CMAKE_SOURCE_DIR}/src/,,$(shell cygpath -m $(abspath $<)))\"'")
	execute_process (COMMAND cygpath -m ~
					 OUTPUT_STRIP_TRAILING_WHITESPACE
					 OUTPUT_VARIABLE PICOPTER_LOG_LOCATION
	)
endif()

if (NOT PICOPTER_LOG_LOCATION)
	set (PICOPTER_LOG_LOCATION "/home/pi/logs")
else ()
	set (PICOPTER_LOG_LOCATION "${PICOPTER_LOG_LOCATION}/logs")
endif()

#Generate the config file
if (UNIX AND NOT CYGWIN)
	set (USE_SYSLOG 1)
endif()
configure_file (${PI_INCLUDE}/config.h.in ${PI_INCLUDE}/config.h)

#Setup the testing framework (gtest)
if (test)
	add_subdirectory (${PI_THIRD_PARTY}/googletest)
	enable_testing ()
	include_directories (${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})
	add_subdirectory (${CMAKE_CURRENT_SOURCE_DIR}/test)
endif()

#Add the base module
add_subdirectory (${PI_SRC}/base)

#Add the modules folder
add_subdirectory (${PI_SRC}/modules)

#Add the testing folder
add_subdirectory (${PI_SRC}/apps)

#Add in the server executable
if (NOT DISABLE_SERVER)
	add_subdirectory (${PI_SRC}/server)
endif (NOT DISABLE_SERVER)
