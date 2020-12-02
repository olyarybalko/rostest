// Bring in my package's API, which is what I'm testing
#include "../src/point.h"
#include "../src/systeminfo.h"
// Bring in gtest
#include <gtest/gtest.h>
#include "ros/ros.h"
#include <string>
#include <ctime>
#include <sstream>
#include "std_msgs/String.h"

// These variables for IoT
const int forfait = 1e+8;
const int year = 2;
const int month = 20;
const int days = 22;
const int timesperday = 2;

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

//test getSerialNumber: type string, longueur de retour 000000000000000d == 16, time de reponce <= 5 ms 
//test getDateTime: type string, longueur de retour 2020-11-30.21:56:51 == 19, time de reponce <= 2 ms 

//test getFreeRam type string, longueur de retour <=32, time de reponce <= 2 ms 
//test getUptimeSys type string, longueur de retour <=32, time de reponce <= 2 ms 
//test getLoadAverage type string, longueur de retour <=5, time de reponce <= 2 ms 

//22*12*2*2 = forfait 100 MB = 1e+8 Bytes

TEST(TestSuite, testSysteminfo)
{
  struct timespec now;
  double start, end;

  SystemInfo infotime;
  EXPECT_TRUE(typeid(std::string) == typeid(infotime.getUpTime()));

  ASSERT_LE(infotime.getUpTime().size(), 128);
  
  // test time de  reponce
  clock_gettime(CLOCK_MONOTONIC, &now);
  start=now.tv_sec+1.e-9*now.tv_nsec;
  infotime.getUpTime();
  clock_gettime(CLOCK_MONOTONIC, &now);
  end=now.tv_sec+1.e-9*now.tv_nsec;
  ASSERT_LE((end-start), .06);
} 

TEST(TestSuite, testFreeRam)
{
  struct timespec now;
  double start, end;

  SystemInfo infoRam;
  EXPECT_TRUE(typeid(std::string) == typeid(infoRam.getFreeRam()));

  ASSERT_LE(infoRam.getFreeRam().size(), 32);

  // test time de  reponce
  clock_gettime(CLOCK_MONOTONIC, &now);
  start=now.tv_sec+1.e-9*now.tv_nsec;
  infoRam.getFreeRam();
  clock_gettime(CLOCK_MONOTONIC, &now);
  end=now.tv_sec+1.e-9*now.tv_nsec;
  ASSERT_LE((end-start), .02);
}

TEST(TestSuite, testUptimeSys)
{
  struct timespec now;
  double start, end;

  SystemInfo infoUpTime;
  EXPECT_TRUE(typeid(std::string) == typeid(infoUpTime.getUpTime()));

  ASSERT_LE(infoUpTime.getUpTime().size(), 32);

  // test time de  reponce
  clock_gettime(CLOCK_MONOTONIC, &now);
  start=now.tv_sec+1.e-9*now.tv_nsec;
  infoUpTime.getUpTime();
  clock_gettime(CLOCK_MONOTONIC, &now);
  end=now.tv_sec+1.e-9*now.tv_nsec;
  ASSERT_LE((end-start), .02);
}

TEST(TestSuite, testLoadAverage)
{
  struct timespec now;
  double start, end;

  SystemInfo infoLoadAverage;

  // test switch case 1
  EXPECT_TRUE(typeid(std::string) == typeid(infoLoadAverage.getLoadAverage(1)));

  ASSERT_LE(infoLoadAverage.getLoadAverage(1).size(), 5);

  long double LoadDouble_0 = std::stold(infoLoadAverage.getLoadAverage(1));
  ASSERT_LE(0.0, LoadDouble_0);

  // test switch case 0
  EXPECT_TRUE(typeid(std::string) == typeid(infoLoadAverage.getLoadAverage(0)));

  ASSERT_LE(infoLoadAverage.getLoadAverage(0).size(), 5);

  long double LoadDouble_1 = std::stold(infoLoadAverage.getLoadAverage(0));
  ASSERT_LE(0.0, LoadDouble_1);

  // test time de  reponce
  clock_gettime(CLOCK_MONOTONIC, &now);
  start=now.tv_sec+1.e-9*now.tv_nsec;
  infoLoadAverage.getLoadAverage(1);
  clock_gettime(CLOCK_MONOTONIC, &now);
  end=now.tv_sec+1.e-9*now.tv_nsec;
  ASSERT_LE((end-start), .02);
}

TEST(TestSuite, testMsg)
{
  Point A, B;
  double d;
  A.setX(0.0);
  A.setY(0.0);
  B.setX(2.0);
  B.setY(2.0);
  d = round(A.distance(B)*10000)/10000;

  SystemInfo thisSystem;
  std_msgs::String msg;
  std::stringstream ss;

  ss <<  thisSystem.getDateTime() << " " << thisSystem.getSerialNumber() << " " <<  d << " " 
  << thisSystem.getUpTime() << " " << thisSystem.getLoadAverage(1);

  msg.data = ss.str();

  float forfait_days = 0.0;
  forfait_days = forfait / (days*month*days*timesperday);

  ASSERT_LE(msg.data.size(), forfait_days);
}


/*TEST(TestSuite, testRam)
{
  struct timespec now;
  double start, end;

  SystemInfo infoRam;
  EXPECT_TRUE(typeid(std::string) == typeid(infoRam.getRam()));

  ASSERT_LE(infoRam.getRam().size(), 32);

  clock_gettime(CLOCK_MONOTONIC, &now);
  start=now.tv_sec+1.e-9*now.tv_nsec;
  infoRam.getRam();
  clock_gettime(CLOCK_MONOTONIC, &now);
  end=now.tv_sec+1.e-9*now.tv_nsec;
  ASSERT_LE((end-start), .02);
}*/

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}