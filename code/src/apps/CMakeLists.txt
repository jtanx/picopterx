# Individual applications

add_executable (nazadecoder naza_decoder.cpp)
add_executable (nazamagcal naza_mag_calibration.cpp)

# Link it with the base module
target_link_libraries (nazadecoder LINK_PUBLIC picopter_base)
target_link_libraries (nazamagcal LINK_PUBLIC picopter_base)
