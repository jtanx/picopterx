#Base module (common components)

#Set the included files
set (SOURCE
	 log.cpp
	 datalog.cpp
	 opts.cpp
	 gpio.cpp
	 buzzer.cpp
	 flightboard.cpp
	 gps.cpp
	 gps_gpsd.cpp
	 gps_naza.cpp
	 imu.cpp
	 flightcontroller.cpp
	 PID.cpp
	 NazaDecoderLib.cpp
	 camera_stream.cpp
	 image_gpu.cpp
)
set (HEADERS
	 flightboard-private.h
	 ${PI_INCLUDE}/common.h
	 ${PI_INCLUDE}/log.h
	 ${PI_INCLUDE}/datalog.h
	 ${PI_INCLUDE}/opts.h
	 ${PI_INCLUDE}/gpio.h
	 ${PI_INCLUDE}/buzzer.h
	 ${PI_INCLUDE}/picopter.h
	 ${PI_INCLUDE}/flightboard.h
	 ${PI_INCLUDE}/navigation.h
	 ${PI_INCLUDE}/gps_feed.h
	 ${PI_INCLUDE}/gps_gpsd.h
	 ${PI_INCLUDE}/gps_naza.h
	 ${PI_INCLUDE}/imu_feed.h
	 ${PI_INCLUDE}/flightcontroller.h
	 ${PI_INCLUDE}/PID.h
	 ${PI_INCLUDE}/NazaDecoderLib.h
	 ${PI_INCLUDE}/camera_stream.h
	 ${PI_INCLUDE}/image_gpu.h
)

#Compile as a static library
add_library (picopter_base STATIC ${HEADERS} ${SOURCE})

#Link to the GCC atomic library
target_link_libraries(picopter_base LINK_PUBLIC atomic)
#Link to gpsd, if present
target_link_libraries(picopter_base LINK_PRIVATE ${LIBGPS_LIBRARIES})
#Link to OpenCV bloat
target_link_libraries(picopter_base LINK_PRIVATE GLESv2 EGL ${OpenCV_LIBS})

#Link to XSens, if present
if (NOT DISABLE_NONFREE)
	target_link_libraries(picopter_base LINK_PRIVATE cmt)
endif (NOT DISABLE_NONFREE)
#Link to wiringPi, if present
if (WIRINGPI_FOUND)
	target_link_libraries(picopter_base LINK_PRIVATE wiringPi)
endif (WIRINGPI_FOUND)
