// Bring in my package's API, which is what I'm testing
#include "../src/point.h"
// Bring in gtest
#include <gtest/gtest.h>
//
#include "ros/ros.h"


// Declare a test
TEST(TestSuite, testCase1)
{
  Point A, B, C;
  double roundX, roundY;
  A.setX(0.0);
  A.setY(0.0);
  B.setX(3.555);
  B.setY(3.556);
  C = A.milieu(B);
  roundX = round(C.getX()*1000)/1000;
  roundY = round(C.getY()*1000)/1000;
  ASSERT_EQ(roundX, 1.778);
  ASSERT_EQ(roundY, 1.778);
}

// Declare another test
TEST(TestSuite, testCase2)
{
  Point A, B;
  double d;
  A.setX(0.0);
  A.setY(0.0);
  B.setX(2.0);
  B.setY(2.0);
  d = round(A.distance(B)*1000)/1000;
  ASSERT_EQ(d, 2.828);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}