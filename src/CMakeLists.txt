SET(USING_CV true)

# driver source files ________________________________
IF(USING_CV)
	SET(sources math_fc/math_fc.cpp alg_fc/alg_fc.cpp debug_fc/debug_fc.cpp rot_fc/rot_fc.cpp cv_fc/cv_fc.cpp)
ELSE(USING_CV)
	SET(sources math_fc/math_fc.cpp alg_fc/alg_fc.cpp debug_fc/debug_fc.cpp rot_fc/rot_fc.cpp)
ENDIF(USING_CV)

# application header files ________________________________
IF(USING_CV)
	SET(headers math_fc/math_fc.h alg_fc/alg_fc.h debug_fc/debug_fc.h rot_fc/rot_fc.h cv_fc/cv_fc.h)
ELSE(USING_CV)
	SET(headers math_fc/math_fc.h alg_fc/alg_fc.h debug_fc/debug_fc.h rot_fc/rot_fc.h)
ENDIF(USING_CV)

set(CMAKE_BUILD_TYPE Release) 

# locate the necessary dependencies
FIND_PACKAGE(Eigen3 REQUIRED)

IF(USING_CV)
	FIND_PACKAGE(OpenCV REQUIRED)
ENDIF(USING_CV)

# add the necessary include directories
INCLUDE_DIRECTORIES(./math_fc ./alg_fc ./debug_fc ./rot_fc ${EIGEN3_INCLUDE_DIR})
IF(USING_CV)
	INCLUDE_DIRECTORIES(./cv_fc ${OpenCV_INCLUDE_DIRS})
ENDIF(USING_CV)

# create the shared library
ADD_LIBRARY(atools SHARED ${sources})

# link necessary libraries
TARGET_LINK_LIBRARIES(atools ${Boost_LIBRARIES} rt) 
IF(USING_CV)
	TARGET_LINK_LIBRARIES(atools ${OpenCV_LIBS}) 
ENDIF(USING_CV)	

# install
INSTALL(TARGETS atools
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib/atools
        ARCHIVE DESTINATION lib/atools)
INSTALL(FILES ${headers} DESTINATION include/atools)
INSTALL(FILES ../Findatools.cmake DESTINATION ${CMAKE_ROOT}/Modules/)
ADD_SUBDIRECTORY(examples)
