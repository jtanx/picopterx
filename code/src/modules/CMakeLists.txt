#Module components

#Set the included files
set (SOURCE
	 waypoints.cpp
)
set (HEADERS
	 ${PI_INCLUDE}/waypoints.h
)

#Compile as a static library
add_library (picopter_modules STATIC ${HEADERS} ${SOURCE})

#Link to base module
target_link_libraries(picopter_modules LINK_PRIVATE picopter_base)