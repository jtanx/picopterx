#Testing module (gtest)

#Set the included files
set (SOURCE
	 test_buzzer.cpp
	 test_opts.cpp
	 test_navigation.cpp
)
set (HEADERS
	 
)

#Add the target
add_executable (run-tests ${HEADERS} ${SOURCE})

#Link it to gtest
target_link_libraries(run-tests LINK_PRIVATE gtest gtest_main)

#Link it with the base module
target_link_libraries (run-tests LINK_PUBLIC picopter_base)

#For `make test`
add_test(NAME picopter-testing 
		 COMMAND ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/run-tests --gtest_color=yes
		 WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})