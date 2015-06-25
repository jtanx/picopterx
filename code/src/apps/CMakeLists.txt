# Individual applications
if (BUILD_OPTIONALS)
	#add_executable (nazadecoder naza_decoder.cpp)
endif()

# Link it with the base module
if (BUILD_OPTIONALS)
	#target_link_libraries (nazadecoder LINK_PUBLIC picopter_base)
endif()