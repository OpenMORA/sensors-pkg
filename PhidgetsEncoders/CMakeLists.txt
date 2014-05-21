# File: CMakeListst.txt for module 
PROJECT(PhidgetsEncoders)

# Search for Phidgets libs/headers:
FIND_PACKAGE(PHIDGETS)

IF(PHIDGETS_FOUND)

	# Use MRPT:
	FIND_PACKAGE( MRPT REQUIRED base ) # hmtslam hwdrivers ... any other sub-library

	INCLUDE_DIRECTORIES( ${MOOS_INCLUDE_DIRS}  )
	LINK_DIRECTORIES( ${MOOS_LIBRARY_DIRS} )

	INCLUDE_DIRECTORIES( ${PHIDGETS_INCLUDE_DIR}  )

	# Add executable
	ADD_EXECUTABLE( ${PROJECT_NAME}
		main.cpp
		CPhidgetsEncoders.h CPhidgetsEncoders.cpp
		)

	# Link the executable
	TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${MOOS_LIBRARIES} ${MRPT_LIBS} ${PHIDGETS_LIBRARIES} )

ELSE(PHIDGETS_FOUND)
	MESSAGE("[${PROJECT_NAME}] **Warning**: PHIDGETS libs not found, this module won't be build!")

ENDIF(PHIDGETS_FOUND)