# create an example application
ADD_EXECUTABLE(atools_function_tests atools_function_tests.cpp)

# link necessary libraries
TARGET_LINK_LIBRARIES(atools_function_tests atools)
