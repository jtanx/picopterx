# The actual server executable

# Find out where Thrift is
find_program (THRIFT_COMPILER thrift DOC "Thrift compiler executable")
if (NOT THRIFT_COMPILER)
	message (FATAL_ERROR "${Red}Thrift was not found. Please install it.${ColourReset}")
endif()

# Set the list of Thrift generated files
set (GENSOURCE
	 gen-cpp/picopter_constants.h
	 gen-cpp/picopter_constants.cpp
	 gen-cpp/picopter_types.h
	 gen-cpp/picopter_types.cpp
	 gen-cpp/webInterface.h
	 gen-cpp/webInterface.cpp
	 gen-php/picopter/Types.php
	 gen-php/picopter/webInterface.php)
	 
# Set the Thrift source files (thrift definition files)
set (THRIFTSOURCE ${CMAKE_CURRENT_SOURCE_DIR}/picopter.thrift)

# Build the generated files
add_custom_command (OUTPUT ${GENSOURCE}
					COMMAND ${THRIFT_COMPILER} --gen cpp ${THRIFTSOURCE}
					COMMAND ${THRIFT_COMPILER} --gen php ${THRIFTSOURCE}
					DEPENDS ${THRIFTSOURCE}
					COMMENT "Generating Thrift interface files...")

# Make it run when make is called					
add_custom_target(thriftgen ALL DEPENDS ${GENSOURCE})					
# Link in the generated header files
include_directories (${CMAKE_CURRENT_BINARY_DIR}/gen-cpp)

# Set the included files
set (SOURCE
	 #gen-cpp/picopter_constants.cpp
	 #gen-cpp/picopter_types.cpp
	 #gen-cpp/webInterface.cpp
	 picopter.cpp
)
set (HEADERS

)

# Add the target
add_executable (picopter ${HEADERS} ${SOURCE})

# Depends on the Thrift generated files
add_dependencies (picopter thriftgen)

# Link it with the base module
target_link_libraries (picopter LINK_PUBLIC picopter_base)
# Link it with the control modules
target_link_libraries (picopter LINK_PUBLIC picopter_modules)