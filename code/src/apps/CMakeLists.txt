# The actual server executable

# Set the included files
set (SOURCE
	NazaDecoderExample.cpp
)
set (HEADERS

)

# Add the target
add_executable (nazadecoder ${HEADERS} ${SOURCE})

add_executable (nazamagcal ${HEADERS} naza_mag_calibration.cpp)

# Link it with the base module
target_link_libraries (nazadecoder LINK_PUBLIC picopter_base)
target_link_libraries (nazamagcal LINK_PUBLIC picopter_base)
