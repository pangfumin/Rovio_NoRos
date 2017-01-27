# CMake generated Testfile for 
# Source directory: /home/pang/catkin_ws/src/rovio/lightweight_filtering
# Build directory: /home/pang/catkin_ws/src/rovio/lightweight_filtering/build
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
ADD_TEST(testState "testState")
ADD_TEST(testSigmaPoints "testSigmaPoints")
ADD_TEST(testPrediction "testPrediction")
ADD_TEST(testUpdate "testUpdate")
ADD_TEST(testModelBase "testModelBase")
ADD_TEST(testFilterBase "testFilterBase")
ADD_TEST(testGIFPrediction "testGIFPrediction")
SUBDIRS(gtest)
