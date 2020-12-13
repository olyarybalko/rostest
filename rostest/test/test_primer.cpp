#include "../src/point.h"
#include "../src/systeminfo.h"
#include <gtest/gtest.h>
#include "ros/ros.h"
#include <string>
#include <ctime>
#include <sstream>
#include "std_msgs/String.h"

// These variables for IoT
const int forfait = 1e+8;
const int year = 2;
const int month = 12;
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

//test getUpTime: type string, longueur de retour <=128, time de reponce <= 6 ms 
TEST(TestSuite, getUpTime)
{
  struct timespec now;
  double start, end;

  SystemInfo info;
  EXPECT_TRUE(typeid(std::string) == typeid(info.getUpTime()));

  ASSERT_LE(info.getUpTime().size(), 128);
  
  // test time de  reponse
  clock_gettime(CLOCK_MONOTONIC, &now);
  start=now.tv_sec+1.e-9*now.tv_nsec;
  info.getUpTime();
  clock_gettime(CLOCK_MONOTONIC, &now);
  end=now.tv_sec+1.e-9*now.tv_nsec;
  ASSERT_LE((end-start), .065);
} 

//test getFreeRam type string, longueur de retour <=32, time de reponce <= 2 ms 
TEST(TestSuite, getFreeRam)
{
  struct timespec now;
  double start, end;

  SystemInfo info;
  EXPECT_TRUE(typeid(std::string) == typeid(info.getFreeRam()));

  ASSERT_LE(info.getFreeRam().size(), 32);

  // test time de  reponse
  clock_gettime(CLOCK_MONOTONIC, &now);
  start=now.tv_sec+1.e-9*now.tv_nsec;
  info.getFreeRam();
  clock_gettime(CLOCK_MONOTONIC, &now);
  end=now.tv_sec+1.e-9*now.tv_nsec;
  ASSERT_LE((end-start), .02);
}

//test getUpTimeFile type string, longueur de retour <=70, time de reponce <= 2 ms 
TEST(TestSuite, getUpTimeFile)
{
  struct timespec now;
  double start, end;

  SystemInfo info;
  EXPECT_TRUE(typeid(std::string) == typeid(info.getUpTimeFile()));

  ASSERT_LE(info.getUpTimeFile().size(), 70);

  // test time de  reponse
  clock_gettime(CLOCK_MONOTONIC, &now);
  start=now.tv_sec+1.e-9*now.tv_nsec;
  info.getUpTimeFile();
  clock_gettime(CLOCK_MONOTONIC, &now);
  end=now.tv_sec+1.e-9*now.tv_nsec;
  ASSERT_LE((end-start), .08);
}

//test getLoadAverage type string, longueur de retour <=5, time de reponce <= 2 ms 
TEST(TestSuite, getLoadAverage)
{
  struct timespec now;
  double start, end;

  SystemInfo info;

  // test switch case 1
  EXPECT_TRUE(typeid(std::string) == typeid(info.getLoadAverage(1)));

  ASSERT_LE(info.getLoadAverage(1).size(), 5);

  long double LoadDouble_0 = std::stold(info.getLoadAverage(1));
  ASSERT_LE(0.0, LoadDouble_0);

  // test switch case 0
  EXPECT_TRUE(typeid(std::string) == typeid(info.getLoadAverage(0)));

  ASSERT_LE(info.getLoadAverage(0).size(), 5);

  long double LoadDouble_1 = std::stold(info.getLoadAverage(0));
  ASSERT_LE(0.0, LoadDouble_1);

  // test time de  reponse
  clock_gettime(CLOCK_MONOTONIC, &now);
  start=now.tv_sec+1.e-9*now.tv_nsec;
  info.getLoadAverage(1);
  clock_gettime(CLOCK_MONOTONIC, &now);
  end=now.tv_sec+1.e-9*now.tv_nsec;
  ASSERT_LE((end-start), .02);
}

//test MsgForfait 22days*12month*2year*2fois = forfait 100 MB = 1e+8 Bytes
TEST(TestSuite, testMsgForfait)
{
  Point A, B;
  double d;
  A.setX(0.0);
  A.setY(0.0);
  B.setX(2.0);
  B.setY(2.0);
  d = round(A.distance(B)*10000)/10000;

  SystemInfo info;
  std_msgs::String msg;
  std::stringstream ss;

  ss <<  info.getDateTime() << " " << info.getSerialNumber() << " " <<  d << " " 
  << info.getUpTimeFile() << " " << info.getLoadAverage(1);

  msg.data = ss.str();

  float forfait_days = 0.0;
  forfait_days = forfait / (year*month*days*timesperday);

  ASSERT_LE(msg.data.size(), forfait_days);
}

//test getSerialNumber: type string, longueur de retour 000000000000000d == 16, time de reponce <= 5 ms 
TEST(TestSuite, getSerialNumber)
{
  struct timespec now;
  double start, end;

  SystemInfo info;
  EXPECT_TRUE(typeid(std::string) == typeid(info.getSerialNumber()));

  ASSERT_LE(info.getSerialNumber().size(), 16);

  clock_gettime(CLOCK_MONOTONIC, &now);
  start=now.tv_sec+1.e-9*now.tv_nsec;
  info.getSerialNumber();
  clock_gettime(CLOCK_MONOTONIC, &now);
  end=now.tv_sec+1.e-9*now.tv_nsec;
  ASSERT_LE((end-start), .15);
}

//test getDateTime: type string, longueur de retour 2020-11-30.21:56:51 == 19, time de reponce <= 2 ms
TEST(TestSuite, getDateTime)
{
  struct timespec now;
  double start, end;

  SystemInfo info;
  EXPECT_TRUE(typeid(std::string) == typeid(info.getDateTime()));

  ASSERT_TRUE(info.getDateTime().size() == 19);

  clock_gettime(CLOCK_MONOTONIC, &now);
  start=now.tv_sec+1.e-9*now.tv_nsec;
  info.getDateTime();
  clock_gettime(CLOCK_MONOTONIC, &now);
  end=now.tv_sec+1.e-9*now.tv_nsec;
  ASSERT_LE((end-start), .02);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}