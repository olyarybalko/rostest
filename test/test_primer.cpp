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
  A.setX(0.0);
  A.setY(0.0);
  B.setX(1.0);
  B.setY(1.0);
  C = A.milieu(B);
  ASSERT_EQ(C.getX(), 0.5);
  ASSERT_EQ(C.getY(), 0.5);
}

// Declare another test
TEST(TestSuite, testCase2)
{
  Point A, B, C;
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