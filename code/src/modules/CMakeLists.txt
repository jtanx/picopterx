#Module components

#Set the included files
set (SOURCE
	 waypoints.cpp
	 object_tracker.cpp
	 user_tracker.cpp
	 observations.cpp
	 env_mapping.cpp
	 utility.cpp
	 pathplan.cpp
	 gridspace.cpp
)
set (HEADERS
	 ${PI_INCLUDE}/waypoints.h
	 ${PI_INCLUDE}/object_tracker.h
	 ${PI_INCLUDE}/user_tracker.h
	 ${PI_INCLUDE}/observations.h
	 ${PI_INCLUDE}/env_mapping.h
	 ${PI_INCLUDE}/utility.h
	 ${PI_INCLUDE}/pathplan.h
	 ${PI_INCLUDE}/gridspace.h
)

#Compile as a static library
add_library (picopter_modules STATIC ${HEADERS} ${SOURCE})

#Link to base module
target_link_libraries(picopter_modules LINK_PRIVATE picopter_base)
