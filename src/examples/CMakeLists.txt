# create an example application
ADD_EXECUTABLE(atools_tests atools_tests.cpp)
# link necessary libraries
TARGET_LINK_LIBRARIES(atools_tests atools)

IF(USING_CV)
  # create an example application
	ADD_EXECUTABLE(cv_tests cv_tests.cpp)
  # link necessary libraries
  TARGET_LINK_LIBRARIES(cv_tests atools)
ENDIF(USING_CV)