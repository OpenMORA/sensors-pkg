## FindPHIDGETS: find the Phidget library
## Copyright (C) 2014 Jose-Luis Blanco-Claraco

# Look for libs & headers:
find_library(PHIDGETS_LIBRARIES
	NAMES phidget21 
	PATHS 
		"/usr/local/"
		"$ENV{ProgramFiles}/Phidgets/lib"
	DOC "Full path of library file 'phidget21.so' or 'phidget21.lib'" )

find_path(PHIDGETS_INCLUDE_DIR
	NAMES phidget21.h
	PATHS 
		"/usr/local/"
		"/usr/"
		"$ENV{ProgramFiles}/Phidgets/include"
	DOC "Path to [PATH]/phidget21.h"
)

set(PHIDGETS_FOUND "NO")
if(PHIDGETS_LIBRARIES AND PHIDGETS_INCLUDE_DIR)
	set(PHIDGETS_FOUND "Yes")
	MESSAGE(STATUS "PHIDGETS_LIBRARIES: ${PHIDGETS_LIBRARIES}")
	MESSAGE(STATUS "PHIDGETS_INCLUDE_DIR: ${PHIDGETS_INCLUDE_DIR}")
	
	MARK_AS_ADVANCED(PHIDGETS_INCLUDE_DIR)
	MARK_AS_ADVANCED(PHIDGETS_LIBRARIES)
endif(PHIDGETS_LIBRARIES AND PHIDGETS_INCLUDE_DIR)
