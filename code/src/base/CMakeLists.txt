#Base module (common components)

#Set the source path
set (PI_BASE ${PI_SRC}/base)

#Set the included files
set (SOURCE
	 ${PI_BASE}/log.cpp
	 ${PI_BASE}/gpio.cpp
)
set (HEADERS
	 ${PI_INCLUDE}/common.h
	 ${PI_INCLUDE}/log.h 
	 ${PI_INCLUDE}/gpio.h
	 ${PI_INCLUDE}/picopter.h
)

#Compile as a static library
add_library (picopter_base STATIC ${HEADERS} ${SOURCE})