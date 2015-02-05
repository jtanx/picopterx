FIND_PROGRAM(THRIFT_COMPILER thrift DOC "Path to thrift compiler executable")

SET( SOURCES gen-cpp/picopter_constants.h gen-cpp/picopter_constants.cpp gen-cpp/picopter_types.h gen-cpp/picopter_types.cpp  gen-cpp/webInterface.h gen-cpp/webInterface.cpp)

ADD_CUSTOM_COMMAND(OUTPUT ${SOURCES} COMMAND ${THRIFT_COMPILER} --gen cpp picopter.thrift)

EXECUTE_PROCESS(COMMAND ${THRIFT_COMPILER} --gen php picopter.thrift WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})




INCLUDE_DIRECTORIES(/usr/local/thrift)


LINK_DIRECTORIES(/usr/local/lib)

ADD_LIBRARY(webInterface ${SOURCES})

TARGET_LINK_LIBRARIES(webInterface thrift)















