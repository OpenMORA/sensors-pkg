# File: CMakeListst.txt for module 
PROJECT(Claraquino)

# Use MRPT:
FIND_PACKAGE( MRPT REQUIRED hwdrivers ) # hmtslam hwdrivers ... any other sub-library

INCLUDE_DIRECTORIES( ${MOOS_INCLUDE_DIRS}  )
LINK_DIRECTORIES( ${MOOS_LIBRARY_DIRS} )

INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/../../EsquemasElectricos/Esquema_DriverMotorVolante/firmware/public_include/")

# Add executable
ADD_EXECUTABLE( ${PROJECT_NAME}
	main.cpp
	CClaraquino.h CClaraquino.cpp
	)

# Link the executable
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${MOOS_LIBRARIES} ${MRPT_LIBS} )

