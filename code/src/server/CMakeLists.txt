#The actual server executable

#Set the included files
set (SOURCE
	 picopter.cpp
)
set (HEADERS

)

#Add the target
add_executable (picopter ${HEADERS} ${SOURCE})

#Link it with the base module
target_link_libraries (picopter LINK_PUBLIC picopter_base)