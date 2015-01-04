#Base module (common components)

#Check for gpsd
pkg_check_modules (LIBGPS libgps)
include_directories (${LIBGPS_INCLUDE_DIRS})
link_directories (${LIBGPS_LIBRARY_DIRS})

#Set the source path
set (PI_BASE ${PI_SRC}/base)

#Set the included files
set (SOURCE
	 ${PI_BASE}/log.cpp
	 ${PI_BASE}/gpio.cpp
	 ${PI_BASE}/buzzer.cpp
	 ${PI_BASE}/flightboard.cpp
	 ${PI_BASE}/gps.cpp
)
set (HEADERS
	 ${PI_INCLUDE}/common.h
	 ${PI_INCLUDE}/log.h 
	 ${PI_INCLUDE}/gpio.h
	 ${PI_INCLUDE}/buzzer.h
	 ${PI_INCLUDE}/picopter.h
	 ${PI_INCLUDE}/flightboard.h
	 ${PI_INCLUDE}/gps_feed.h
	 ${PI_BASE}/flightboard-private.h
)

#Compile as a static library
add_library (picopter_base STATIC ${HEADERS} ${SOURCE})

#Link to the GCC atomic library
target_link_libraries(picopter_base LINK_PUBLIC atomic)

#Link to gpsd, if present
target_link_libraries(picopter_base LINK_PRIVATE ${LIBGPS_LIBRARIES})