// Bring in my package's API, which is what I'm testing
#include "../src/point.h"
#include "../src/systeminfo.h"
// Bring in gtest
#include <gtest/gtest.h>
#include "ros/ros.h"
#include <string>
#include <ctime>

// Declare a test
TEST(TestSuite, testConstDefault)
{
  Point A;
  ASSERT_EQ(A.getX(), 0.0);
  ASSERT_EQ(A.getY(), 0.0);
}

TEST(TestSuite, testConstCopy)
{
  Point A, B;
  A.setX(3.0);
  A.setY(7.8);
  B = A;
  ASSERT_EQ(B.getX(), 3.0);
  ASSERT_EQ(B.getY(), 7.8);
}

TEST(TestSuite, testMilieu)
{
  Point A, B, C;
  double roundX, roundY;
  A.setX(0.0);
  A.setY(0.0);
  B.setX(1.0);
  B.setY(1.0);
  C = A.milieu(B);
  roundX = round(C.getX()*10000)/10000;
  roundY = round(C.getY()*10000)/10000;
  ASSERT_DOUBLE_EQ(roundX, 0.5);
  ASSERT_DOUBLE_EQ(roundY, 0.5);
}

TEST(TestSuite, testDistance)
{
  Point A, B;
  double d;
  A.setX(0.0);
  A.setY(0.0);
  B.setX(2.0);
  B.setY(2.0);
  d = round(A.distance(B)*10000)/10000;
  ASSERT_DOUBLE_EQ(d, 2.8284);
}

//test Systeminfo: type string, longueur de retour <=128, time de reponce <= 6 ms 
TEST(TestSuite, testSysteminfo)
{
  struct timespec now;
  double start, end;

  SystemInfo infotime;
  std::string info;
  EXPECT_TRUE(typeid(info) == typeid(infotime.getUpTime()));

  ASSERT_LE(infotime.getUpTime().size(), 128);

  clock_gettime(CLOCK_MONOTONIC, &now);
  start=now.tv_sec+1.e-9*now.tv_nsec;
  infotime.getUpTime();
  clock_gettime(CLOCK_MONOTONIC, &now);
  end=now.tv_sec+1.e-9*now.tv_nsec;
  ASSERT_LE((end-start), .1);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}