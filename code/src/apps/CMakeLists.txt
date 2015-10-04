# Individual applications
if (BUILD_OPTIONALS)
	add_executable (fbtest fbtest.cpp)
	add_executable (pathtest pathtest.cpp)
	add_executable (gridtest gridtest.cpp)
	add_executable (trackertest trackertest.cpp)
	add_executable (camtest camtest.cpp)
	#add_executable (nazadecoder naza_decoder.cpp)
endif()

# Link it with the base module
if (BUILD_OPTIONALS)
	target_link_libraries (fbtest LINK_PUBLIC picopter_base)
	target_link_libraries (pathtest LINK_PUBLIC picopter_modules)
	target_link_libraries (gridtest LINK_PUBLIC picopter_modules)
	target_link_libraries (trackertest LINK_PUBLIC picopter_modules)
	target_link_libraries (camtest LINK_PUBLIC picopter_base)
	#target_link_libraries (nazadecoder LINK_PUBLIC picopter_base)
endif()
