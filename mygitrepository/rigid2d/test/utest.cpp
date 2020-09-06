#include<iostream>
#include<iosfwd> // contains forward definitions for iostream objects
#include <ros/ros.h>
#include <gtest/gtest.h>
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include "rigid2d/waypoints.hpp"


TEST(rigid2dTest,rigid2dHpp)
{
  ASSERT_DOUBLE_EQ(rigid2d::PI, 3.14159265358979323846) << "PI definition is wrong.";
  ASSERT_FALSE(rigid2d::almost_equal(0.0,1.1e-12)) << "almost_equal failed";  
  ASSERT_TRUE(rigid2d::almost_equal(rigid2d::deg2rad(0.0),0.0)) << "deg2rad failed";
  ASSERT_TRUE(rigid2d::almost_equal(rigid2d::rad2deg(0.0),0.0)) << "rad2deg failed";
  ASSERT_TRUE(rigid2d::almost_equal(rigid2d::deg2rad(rigid2d::rad2deg(30)),30)) << "rad2deg/deg2rad failed";

  //Vector2D test
  rigid2d::Vector2D v, nv;
  v.x=2.0;
  v.y=3.0;
  nv=rigid2d::normalize(v);
  double l;
  l=sqrt(2*2+3*3);
  ASSERT_DOUBLE_EQ(nv.x*l,v.x) << "normalize failed";
  ASSERT_DOUBLE_EQ(nv.y*l,v.y) << "normalize failed";

  std::stringstream sst;
  sst<<v;
  int flag = strcmp(sst.str().c_str() ,"[2 3]");
  ASSERT_TRUE(flag == 0) << "operator<<(std::ostream & os, const Vector2D & v) fail. Correct: " << v;
  sst.str("");
  sst.clear();

  std::string temp;
  temp="2 3";
  sst<<temp;
  rigid2d::Vector2D tv;
  sst>>tv;
  ASSERT_DOUBLE_EQ(tv.x,v.x) << "operator>>(std::istream & is, const Vector2D & v) fail";
  ASSERT_DOUBLE_EQ(tv.y,v.y) << "operator>>(std::istream & is, const Vector2D & v) fail";
  sst.str("");
  sst.clear();

  //Twist2D test
  rigid2d::Twist2D t, tt;
  t.Vx=2;
  t.Vy=3;
  t.Wz=4;

  sst<<t;
  flag = strcmp(sst.str().c_str() ,"[4 2 3]");
  ASSERT_TRUE(flag == 0) << "operator<<(std::ostream & os, const Twist2D & v) fail. Correct: " << t;
  sst.str("");
  sst.clear();

  temp="4 2 3";
  sst<<temp;
  sst>>tt;
  ASSERT_DOUBLE_EQ(tt.Vx,t.Vx) << "operator>>(std::istream & is, const Twist2D & v) fail";
  ASSERT_DOUBLE_EQ(tt.Vy,t.Vy) << "operator>>(std::istream & is, const Twist2D & v) fail";
  ASSERT_DOUBLE_EQ(tt.Wz,t.Wz) << "operator>>(std::istream & is, const Twist2D & v) fail";
  sst.str("");
  sst.clear();
  
  rigid2d::Transform2D T1;
  t=T1.displacement();
  ASSERT_DOUBLE_EQ(t.Vx,0) << "Transform2D()/displacement fail";
  ASSERT_DOUBLE_EQ(t.Vy,0) << "Transform2D()/displacement fail";
  ASSERT_DOUBLE_EQ(t.Wz,0) << "Transform2D()/displacement fail";

  double radius = 2.1;
  rigid2d::Transform2D T2(radius);
  t=T2.displacement();
  ASSERT_DOUBLE_EQ(t.Vx,0) << "Transform2D(double radians) fail";
  ASSERT_DOUBLE_EQ(t.Vy,0) << "Transform2D(double radians) fail";
  ASSERT_DOUBLE_EQ(t.Wz,radius) << "Transform2D(double radians) fail" ;

  rigid2d::Transform2D T3(v);
  t=T3.displacement();
  ASSERT_DOUBLE_EQ(t.Vx,v.x) << "Transform2D(const Vector2D & trans) fail";
  ASSERT_DOUBLE_EQ(t.Vy,v.y) << "Transform2D(const Vector2D & trans) fail";
  ASSERT_DOUBLE_EQ(t.Wz,0) << "Transform2D(const Vector2D & trans) fail";

  rigid2d::Transform2D T4(v,radius);
  t=T4.displacement();
  ASSERT_DOUBLE_EQ(t.Vx,v.x) << "Transform2D(const Vector2D & trans, double radians) fail";
  ASSERT_DOUBLE_EQ(t.Vy,v.y) << "Transform2D(const Vector2D & trans, double radians) fail";
  ASSERT_DOUBLE_EQ(t.Wz,radius) << "Transform2D(const Vector2D & trans, double radians) fail";
  
  tv=T4(v);
  ASSERT_NEAR(tv.x,-1.59932,1.0e-4) << "Vector2D Transform2D::operator()(Vector2D v) const fail";
  ASSERT_NEAR(tv.y,3.21188,1.0e-4) << "Vector2D Transform2D::operator()(Vector2D v) const fail";
  
  t.Vx=2;
  t.Vy=3;
  t.Wz=4;
  tt=T4(t);
  ASSERT_NEAR(tt.Vx,8.4006797,1.0e-4) << "Twist2D Transform2D::operator()(Twist2D v) const fail";
  ASSERT_NEAR(tt.Vy,-7.7881196,1.0e-4) << "Twist2D Transform2D::operator()(Twist2D v) const fail";
  ASSERT_NEAR(tt.Wz,4.0,1.0e-4) << "Twist2D Transform2D::operator()(Twist2D v) const fail";

  rigid2d::Transform2D T5;
  T5=T4.inv();
  t=T5.displacement();
  ASSERT_NEAR(t.Vx,-1.579936,1.0e-4) << "Transform2D inv() const fail";
  ASSERT_NEAR(t.Vy,3.240957,1.0e-4) << "Transform2D inv() const fail";
  ASSERT_NEAR(t.Wz,-radius,1.0e-4) << "Transform2D inv() const fail";

  T5*=T4;
  t=T5.displacement();
  ASSERT_NEAR(t.Vx,0,1.0e-4) << "Transform2D & Transform2D::operator*=(const Transform2D & rhs) fail";
  ASSERT_NEAR(t.Vy,0,1.0e-4) << "Transform2D & Transform2D::operator*=(const Transform2D & rhs) fail";
  ASSERT_NEAR(t.Wz,0,1.0e-4) << "Transform2D & Transform2D::operator*=(const Transform2D & rhs) fail";

  
  sst<<T4;
  flag = strcmp(sst.str().c_str() ,"dtheta (degrees): 120.321 dx: 2 dy: 3\n");
  ASSERT_TRUE(flag == 0) << "operator<<(std::ostream & os, const Transform2D & tf) fail. Correct: " << sst.str();
  sst.str("");
  sst.clear();

  rigid2d::Transform2D T6;
  temp="35 5 8";
  sst<<temp;
  sst>>T6;
  t=T6.displacement();
  ASSERT_NEAR(t.Vx,5,1.0e-4) << "operator>>(std::istream & is, Transform2D & tf) fail.";
  ASSERT_NEAR(t.Vy,8,1.0e-4) << "operator>>(std::istream & is, Transform2D & tf) fail.";
  ASSERT_NEAR(t.Wz,rigid2d::deg2rad(35),1.0e-4) << "operator>>(std::istream & is, Transform2D & tf) fail.";

  rigid2d::Transform2D T7;
  T5=T4.inv();
  T7=T4*T5;
  t=T7.displacement();
  ASSERT_NEAR(t.Vx,0,1.0e-4) << "Transform2D & operator*(Transform2D lhs, const Transform2D & rhs) fail";
  ASSERT_NEAR(t.Vy,0,1.0e-4) << "Transform2D & operator*(Transform2D lhs, const Transform2D & rhs) fail";
  ASSERT_NEAR(t.Wz,0,1.0e-4) << "Transform2D & operator*(Transform2D lhs, const Transform2D & rhs) fail";

  t.Vx=2;
  t.Vy=3;
  t.Wz=4;
  rigid2d::Transform2D T8;
  T8=rigid2d::integrateTwist(t);
  t=T8.displacement();
  ASSERT_NEAR(t.Wz,-2.2831853,1.0e-4) << "integrateTwist fail";
  ASSERT_NEAR(t.Vx,-1.6186339,1.0e-4) << "integrateTwist fail";
  ASSERT_NEAR(t.Vy,0.25921994,1.0e-4) << "integrateTwist fail";
  
  v.x=1;
  v.y=2;
  tv.x=3;
  tv.y=3;
  v+=tv;
  ASSERT_EQ(v.x,4) << "Vector2D & operator+=(const Vector2D & v) fail";
  ASSERT_EQ(v.y,5) << "Vector2D & operator+=(const Vector2D & v) fail";
  rigid2d::Vector2D v1;
  v1=v+tv;
  ASSERT_EQ(v1.x,7) << "Vector2D operator+(const Vector2D & v) fail";
  ASSERT_EQ(v1.y,8) << "Vector2D operator+(const Vector2D & v) fail";

  v-=tv;
  ASSERT_EQ(v.x,1) << "Vector2D & operator-=(const Vector2D & v) fail";
  ASSERT_EQ(v.y,2) << "Vector2D & operator-=(const Vector2D & v) fail";
  v1=v-tv;
  ASSERT_EQ(v1.x,-2) << "Vector2D operator+(const Vector2D & v) fail";
  ASSERT_EQ(v1.y,-1) << "Vector2D operator+(const Vector2D & v) fail";
  
  v*=4;
  ASSERT_EQ(v.x,4) << "Vector2D & operator*=(double a) fail";
  ASSERT_EQ(v.y,8) << "Vector2D & operator*=(double a) fail";

  v1=v*2;
  ASSERT_EQ(v1.x,8) << "Vector2D operator*(double a) fail";
  ASSERT_EQ(v1.y,16) << "Vector2D operator*(double a) fail";
  
  v1=3.0*v;
  ASSERT_EQ(v1.x,12) << "Vector2D operator*(double a) fail";
  ASSERT_EQ(v1.y,24) << "Vector2D operator*(double a) fail";

  ASSERT_NEAR(rigid2d::length(v),8.9443,1.0e-3) << "double length(const Vector2D &v) fail";
  ASSERT_NEAR(rigid2d::distance(v,tv),5.0990,1.0e-3) << "double distance(const Vector2D &v1, const Vector2D &v2) fail";
  ASSERT_NEAR(rigid2d::angle(v),1.10715,1.0e-3) << "double angle(const Vector2D &v) fail";

  rigid2d::Vector2D v2(3.2,4.4);
  ASSERT_NEAR(v2.x,3.2,1.0e-4) << "Vector2D constructor fail";
  ASSERT_NEAR(v2.y,4.4,1.0e-4) << "Vector2D constructor fail";
  rigid2d::Vector2D v3;
  ASSERT_NEAR(v3.x,0,1.0e-4) << "Vector2D constructor fail";
  ASSERT_NEAR(v3.y,0,1.0e-4) << "Vector2D constructor fail";

  double a=4.5;
  ASSERT_NEAR(rigid2d::normalize_angle(a),-1.7831853,1.0e-3) << "normalize_angle(double rad) fail";

}

TEST(rigid2dTest,diffdriveHpp)
{
  rigid2d::Twist2D t;
  t.Wz=1;
  t.Vx=2;
  t.Vy=0;
  rigid2d::DiffDrive dd1;
  rigid2d::WheelVelocities WV;
  WV=dd1.twistToWheels(t);
  ASSERT_EQ(WV.w_left,7.5) << "WheelVelocities twistToWheels(const Twist2D & t)/default constructor fail";
  ASSERT_EQ(WV.w_right,12.5) << "WheelVelocities twistToWheels(const Twist2D & t)/default constructor fail";
  
  t=dd1.wheelsToTwist(WV);
  ASSERT_EQ(t.Wz,1) << "Twist2D wheelsToTwist(const WheelVelocities wv) fail";
  ASSERT_EQ(t.Vx,2) << "Twist2D wheelsToTwist(const WheelVelocities wv) fail";
  ASSERT_EQ(t.Vy,0) << "Twist2D wheelsToTwist(const WheelVelocities wv) fail";


  /**moving straight：feedforward and updateOdometry updated properly*/
  t.Wz=0;
  t.Vx=2;
  t.Vy=1;
  rigid2d::DiffDrive dd2(t,2,0.4);
  rigid2d::DiffDrive dd3(t,2,0.4);
  rigid2d::Twist2D tt;
  tt.Wz=0;
  tt.Vx=1;
  tt.Vy=0;
  rigid2d::WheelVelocities jointstates;
  for (int i=0;i<10;i++)
  {
    dd2.feedforward(tt);
    jointstates=dd2.getJointStateFeedforward();
    dd3.updateOdometry(jointstates.w_left,jointstates.w_right);
  }
  rigid2d::Twist2D PoseNow;
  PoseNow=dd2.pose();
  ASSERT_EQ(PoseNow.Wz,0) << "moving straight:feedforward update fail";
  ASSERT_EQ(PoseNow.Vx,12) << "moving straight:feedforward update fail";
  ASSERT_EQ(PoseNow.Vy,1) << "moving straight:feedforward update fail";
  PoseNow=dd3.pose();
  ASSERT_EQ(PoseNow.Wz,0) << "moving straight:updateOdometry update fail";
  ASSERT_EQ(PoseNow.Vx,12) << "moving straight:updateOdometry update fail";
  ASSERT_EQ(PoseNow.Vy,1) << "moving straight:updateOdometry update fail";

  /**turning：feedforward and updateOdometry updated properly*/
  dd2.reset(t);
  dd3.reset(t);
  tt.Wz=1;
  tt.Vx=0;
  tt.Vy=0;
  for (int i=0;i<10;i++)
  {
    dd2.feedforward(tt);
    jointstates=dd2.getJointStateFeedforward();
    dd3.updateOdometry(jointstates.w_left,jointstates.w_right);
  }
  PoseNow=dd2.pose();
  ASSERT_EQ(PoseNow.Wz,10-rigid2d::PI*4) << "turning:feedforward update fail";
  ASSERT_EQ(PoseNow.Vx,2) << "turning:feedforward update fail";
  ASSERT_EQ(PoseNow.Vy,1) << "turning:feedforward update fail";
  PoseNow=dd3.pose();
  ASSERT_EQ(PoseNow.Wz,10-rigid2d::PI*4) << "turning:updateOdometry update fail";
  ASSERT_EQ(PoseNow.Vx,2) << "turning:updateOdometry update fail";
  ASSERT_EQ(PoseNow.Vy,1) << "turning:updateOdometry update fail";

  /**translating and turning：feedforward and updateOdometry updated properly*/
  dd2.reset(t);
  dd3.reset(t);
  tt.Wz=1;
  tt.Vx=1;
  tt.Vy=0;
  for (int i=0;i<10;i++)
  {
    dd2.feedforward(tt);
    jointstates=dd2.getJointStateFeedforward();
    dd3.updateOdometry(jointstates.w_left,jointstates.w_right);
  }
  PoseNow=dd2.pose();
  ASSERT_EQ(PoseNow.Wz,10-rigid2d::PI*4) << "translating and turning:feedforward update fail";
  ASSERT_NEAR(PoseNow.Vx,2+cos(10-rigid2d::PI/2),1.0e-5) << "translating and turning:feedforward update fail";
  ASSERT_NEAR(PoseNow.Vy,2+sin(10-rigid2d::PI/2),1.0e-5) << "translating and turning:feedforward update fail";
  PoseNow=dd3.pose();
  ASSERT_EQ(PoseNow.Wz,10-rigid2d::PI*4) << "translating and turning:feedforward update fail";
  ASSERT_NEAR(PoseNow.Vx,2+cos(10-rigid2d::PI/2),1.0e-5) << "translating and turning:updateOdometry update fail";
  ASSERT_NEAR(PoseNow.Vy,2+sin(10-rigid2d::PI/2),1.0e-5) << "translating and turning:updateOdometry update fail";
  

}
TEST(rigid2dTest,waypointsHpp)
{
  std::vector<rigid2d::Vector2D> wayP;
  rigid2d::Vector2D singleP;
  singleP.x=0.1;
  singleP.y=0;
  wayP.push_back(singleP);
  singleP.x=1;
  singleP.y=0;
  wayP.push_back(singleP);
  rigid2d::waypoints testdd(wayP,2,0.4,0.5,0.5,1);
  rigid2d::Twist2D wv;
  rigid2d::Twist2D POSENOW;
  wv=testdd.nextWaypoint();
  wv=testdd.nextWaypoint();
  wv=testdd.nextWaypoint();
  wv=testdd.nextWaypoint();
  wv=testdd.nextWaypoint();
  wv=testdd.nextWaypoint();
  wv=testdd.nextWaypoint();
  wv=testdd.nextWaypoint();
  wv=testdd.nextWaypoint();
  wv=testdd.nextWaypoint();
  wv=testdd.nextWaypoint();
  wv=testdd.nextWaypoint();
  wv=testdd.nextWaypoint();
  wv=testdd.nextWaypoint();
  wv=testdd.nextWaypoint();
  wv=testdd.nextWaypoint();
  wv=testdd.nextWaypoint();
  wv=testdd.nextWaypoint();
  wv=testdd.nextWaypoint();
  POSENOW=testdd.getpose();
  ASSERT_NEAR(POSENOW.Vx,0.6,1.0e-4)<<"nextwaypoints fail";
  ASSERT_NEAR(POSENOW.Vy,0,1.0e-4)<<"nextwaypoints fail";
  ASSERT_NEAR(POSENOW.Wz,0,1.0e-4)<<"nextwaypoints fail";

}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "utest");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
