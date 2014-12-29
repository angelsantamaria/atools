# driver source files
SET(sources math_fc/math_fc.cpp alg_fc/alg_fc.cpp debug_fc/debug_fc.cpp cv_fc/cv_fc.cpp rot_fc/rot_fc.cpp)

# application header files
SET(headers math_fc/math_fc.h alg_fc/alg_fc.h debug_fc/debug_fc.h cv_fc/cv_fc.h rot_fc/rot_fc.h)

set(CMAKE_BUILD_TYPE Release) 

# locate the necessary dependencies
FIND_PACKAGE(Eigen REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)

# add the necessary include directories
INCLUDE_DIRECTORIES(./math_fc ./alg_fc ./debug_fc ./cv_fc ./rot_fc ${EIGEN_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})

# create the shared library
ADD_LIBRARY(atools SHARED ${sources})

# link necessary libraries
TARGET_LINK_LIBRARIES(atools ${EIGEN_LIBRARY} ${Boost_LIBRARIES} ${OpenCV_LIBS} rt) 

# install
INSTALL(TARGETS atools
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib/atools
        ARCHIVE DESTINATION lib/atools)
INSTALL(FILES ${headers} DESTINATION include/atools)
INSTALL(FILES ../Findatools.cmake DESTINATION ${CMAKE_ROOT}/Modules/)
ADD_SUBDIRECTORY(examples)
