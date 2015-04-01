#The actual server executable

#Set the included files
set (SOURCE
	 picopter.cpp
)
set (HEADERS

)

#Add the target
add_executable (picopter ${HEADERS} ${SOURCE})

#Depends on the Thrift generated files
add_dependencies (picopter thriftgen)

#Link it with the base module
target_link_libraries (picopter LINK_PUBLIC picopter_base)
#Link it with the control modules
target_link_libraries (picopter LINK_PUBLIC picopter_modules)