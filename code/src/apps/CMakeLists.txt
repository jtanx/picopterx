# Individual applications
if (BUILD_OPTIONALS)
	add_executable (nazadecoder naza_decoder.cpp)
	add_executable (nazamagcal naza_mag_calibration.cpp)
endif()

# Link it with the base module
if (BUILD_OPTIONALS)
	target_link_libraries (nazadecoder LINK_PUBLIC picopter_base)
	target_link_libraries (nazamagcal LINK_PUBLIC picopter_base)
endif()