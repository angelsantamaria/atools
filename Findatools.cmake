#edit the following line to add the librarie's header files
FIND_PATH(atools_INCLUDE_DIR
	NAMES alg_fc.h cv_fc.h debug_fc.h math_fc.h rot_fc.h
	PATHS /usr/include /usr/local/include /usr/local/include/atools)

FIND_LIBRARY(atools_LIBRARY
    NAMES atools
    PATHS /usr/lib /usr/local/lib /usr/local/lib/atools) 

IF (atools_INCLUDE_DIR AND atools_LIBRARY)
   SET(atools_FOUND TRUE)
ENDIF (atools_INCLUDE_DIR AND atools_LIBRARY)

IF (atools_FOUND)
   IF (NOT atools_FIND_QUIETLY)
      MESSAGE(STATUS "Found atools: ${atools_LIBRARY}")
   ENDIF (NOT atools_FIND_QUIETLY)
ELSE (atools_FOUND)
   IF (atools_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find atools")
   ENDIF (atools_FIND_REQUIRED)
ENDIF (atools_FOUND)

