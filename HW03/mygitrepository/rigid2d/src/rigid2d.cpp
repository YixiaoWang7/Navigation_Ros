#include "rigid2d/rigid2d.hpp"
#include <iostream>
#include <iosfwd> // contains forward definitions for iostream objects


//input and output
std::ostream & rigid2d::operator<<(std::ostream & os, const rigid2d::Vector2D & v)
{
  os<<"["<<v.x<<" "<<v.y<<"]";
  return os;
}

std::istream & rigid2d::operator>>(std::istream & is, rigid2d::Vector2D & v)
{ 
  std::cout<<"Enter a 2D vector separated by single space:"<<std::endl;
  is>>v.x>>v.y;
  return is;
}

std::ostream & rigid2d::operator<<(std::ostream & os, const rigid2d::Twist2D & t)
{
  os<<"["<<t.Wz<<" "<<t.Vx<<" "<<t.Vy<<"]";
  return os;
}

std::istream & rigid2d::operator>>(std::istream & is, rigid2d::Twist2D & t)
{ 
  std::cout<<"Enter a 2D Twist"<<std::endl;
  std::cout<<"Wz, Vx and Vy separated by single space:"<<std::endl;
  is>>t.Wz>>t.Vx>>t.Vy;
  return is;
}

//displacement from Transform2D

rigid2d::Twist2D rigid2d::Transform2D::displacement()
{
  rigid2d::Twist2D t;
  double theta;
  t.Vx=TM[0][2];
  t.Vy=TM[1][2];
  theta=acos(TM[0][0]);
  
  if (TM[1][0]<0) theta=-theta;
  t.Wz=theta;
  return t;
}


//Transform2D class functions

rigid2d::Transform2D::Transform2D()
{
  for (int i=0;i<2;i++)
  {
    for (int j=0;j<3;j++)
    {
      TM[i][j]=I[i][j];
    };
  };
}
rigid2d::Transform2D::Transform2D(const Vector2D & trans)
{
  for (int i=0;i<2;i++)
  {
    for (int j=0;j<3;j++)
    {
      TM[i][j]=I[i][j];
    };
  };
  TM[0][2]=trans.x;
  TM[1][2]=trans.y;
}

rigid2d::Transform2D::Transform2D(double radians)
{
  for (int i=0;i<2;i++)
  {
    TM[i][2]=I[i][2];
  };
  TM[0][0]=cos(radians);
  TM[0][1]=-sin(radians);
  TM[1][0]=sin(radians);
  TM[1][1]=cos(radians);
}
rigid2d::Transform2D::Transform2D(const Vector2D & trans, double radians)
{
  TM[0][2]=trans.x;
  TM[1][2]=trans.y;
  TM[0][0]=cos(radians);
  TM[0][1]=-sin(radians);
  TM[1][0]=sin(radians);
  TM[1][1]=cos(radians);
}
rigid2d::Vector2D rigid2d::Transform2D::operator()(Vector2D v) const
{
  Vector2D VafterT;
  VafterT.x=v.x*TM[0][0]+v.y*TM[0][1]+TM[0][2];
  VafterT.y=v.x*TM[1][0]+v.y*TM[1][1]+TM[1][2];
  return VafterT;
}
rigid2d::Twist2D rigid2d::Transform2D::operator()(Twist2D v) const
{
  Twist2D TafterT;
  TafterT.Wz=v.Wz;
  TafterT.Vx=TM[0][0]*v.Vx+TM[0][1]*v.Vy+TM[1][2]*v.Wz;
  TafterT.Vy=TM[1][0]*v.Vx+TM[1][1]*v.Vy-TM[0][2]*v.Wz;
  return TafterT;
}
rigid2d::Transform2D rigid2d::Transform2D::inv() const
{ 
  Transform2D InT;

  InT.TM[0][2]=-TM[0][0]*TM[0][2]-TM[1][0]*TM[1][2];
  InT.TM[1][2]=-TM[0][1]*TM[0][2]-TM[1][1]*TM[1][2];
  InT.TM[0][1]=TM[1][0];
  InT.TM[1][0]=TM[0][1];
  InT.TM[0][0]=TM[0][0];
  InT.TM[1][1]=TM[1][1];

  return InT;
}
rigid2d::Transform2D & rigid2d::Transform2D::operator*=(const Transform2D & rhs)
{
  float t11,t12,t13,t21,t22,t23,t31,t32,t33;
  t11=TM[0][0];
  t12=TM[0][1];
  t13=TM[0][2];
  t21=TM[1][0];
  t22=TM[1][1];
  t23=TM[1][2];
  t31=TM[2][0];
  t32=TM[2][1];
  t33=TM[2][2];

  float rt11,rt12,rt13,rt21,rt22,rt23,rt31,rt32,rt33;
  rt11=rhs.TM[0][0];
  rt12=rhs.TM[0][1];
  rt13=rhs.TM[0][2];
  rt21=rhs.TM[1][0];
  rt22=rhs.TM[1][1];
  rt23=rhs.TM[1][2];
  rt31=rhs.TM[2][0];
  rt32=rhs.TM[2][1];
  rt33=rhs.TM[2][2];

  TM[0][0]=t11*rt11+t12*rt21+t13*rt31;
  TM[0][1]=t11*rt12+t12*rt22+t13*rt32;
  TM[0][2]=t11*rt13+t12*rt23+t13*rt33;
  TM[1][0]=t21*rt11+t22*rt21+t23*rt31;
  TM[1][1]=t21*rt12+t22*rt22+t23*rt32;
  TM[1][2]=t21*rt13+t22*rt23+t23*rt33;
  TM[2][0]=t31*rt11+t32*rt21+t33*rt31;
  TM[2][1]=t31*rt12+t32*rt22+t33*rt32;
  TM[2][2]=t31*rt13+t32*rt23+t33*rt33;
  return *this;
}
std::ostream & rigid2d::operator<<(std::ostream & os, const rigid2d::Transform2D & tf)
{
  /**
  os<<"Transform Matrix:"<<std::endl;
  os<<tf.TM[0][0]<<","<<tf.TM[0][1]<<","<<tf.TM[0][2]<<std::endl;
  os<<tf.TM[1][0]<<","<<tf.TM[1][1]<<","<<tf.TM[1][2]<<std::endl;
  os<<tf.TM[2][0]<<","<<tf.TM[2][1]<<","<<tf.TM[2][2]<<std::endl;
  */
  double x,y,theta;
  x=tf.TM[0][2];
  y=tf.TM[1][2];
  theta=acos(tf.TM[0][0]);
  
  if (tf.TM[1][0]<0) theta=-theta;
  os<<"dtheta (degrees): "<<rigid2d::rad2deg(theta)<<" dx: "<<x<<" dy: "<<y<<std::endl;
  return os;
}
std::istream & rigid2d::operator>>(std::istream & is, rigid2d::Transform2D & tf)
{
  std::cout<<"Enter degree, x and y"<<std::endl;
  std::cout<<"Every consecutive two should be separated by single space: "<<std::endl;
  Vector2D v;
  double theta;
  is>>theta>>v.x>>v.y;
  Transform2D T(v,deg2rad(theta));
  tf*=T;
  return is;
}
rigid2d::Transform2D rigid2d::operator*(rigid2d::Transform2D lhs, const rigid2d::Transform2D & rhs)
{
  return lhs*=rhs;
}

/// \integrateTwist to form a transform2D
rigid2d::Transform2D rigid2d::integrateTwist(const rigid2d::Twist2D & t)
{
  double theta=0,Vx,Vy;
  rigid2d::Vector2D V;
  if (t.Wz!=0)
  {
    theta=t.Wz;
    Vx=t.Vx/theta;
    Vy=t.Vy/theta;
    V.x=sin(theta)*Vx+(cos(theta)-1)*Vy;
    V.y=(1-cos(theta))*Vx+sin(theta)*Vy;
  }else
  {
    V.x=t.Vx;
    V.y=t.Vy;
  }
  rigid2d::Transform2D tf(V,theta);
  return tf;
}

//Vector operations
rigid2d::Vector2D rigid2d::normalize(const rigid2d::Vector2D & v)
{
  rigid2d::Vector2D nv;
  nv.x=v.x/sqrt(v.x*v.x+v.y*v.y);
  nv.y=v.y/sqrt(v.x*v.x+v.y*v.y);
  return nv;
}

rigid2d::Vector2D & rigid2d::Vector2D::operator+=(const Vector2D & v)
{
  this->x+=v.x;
  this->y+=v.y;
  return *this;
}
rigid2d::Vector2D rigid2d::Vector2D::operator+(const rigid2d::Vector2D & v2)
{
  rigid2d::Vector2D v;
  v.x=this->x+v2.x;
  v.y=this->y+v2.y;
  return v;
}
rigid2d::Vector2D & rigid2d::Vector2D::operator-=(const Vector2D & v)
{
  this->x-=v.x;
  this->y-=v.y;
  return *this;
}
rigid2d::Vector2D rigid2d::Vector2D::operator-(const rigid2d::Vector2D & v2)
{
  rigid2d::Vector2D v;
  v.x=this->x-v2.x;
  v.y=this->y-v2.y;
  return v;
}
rigid2d::Vector2D & rigid2d::Vector2D::operator*=(double a)
{
  this->x*=a;
  this->y*=a;
  return *this;
}
rigid2d::Vector2D rigid2d::Vector2D::operator*(double a)
{
  rigid2d::Vector2D v;
  v.x=this->x*a;
  v.y=this->y*a;
  return v;
}
rigid2d::Vector2D rigid2d::operator*(double a, const rigid2d::Vector2D &v)
{
  rigid2d::Vector2D v1;
  v1.x=v.x*a;
  v1.y=v.y*a;
  return v1;
}

rigid2d::Twist2D & rigid2d::Twist2D::operator+=(const Twist2D & t)
{
  this->Vx+=t.Vx;
  this->Vy+=t.Vy;
  this->Wz+=t.Wz;
  return *this;
}
rigid2d::Twist2D rigid2d::Twist2D::operator+(const Twist2D & t2)
{
  rigid2d::Twist2D t;
  t.Vx=this->Vx+t2.Vx;
  t.Vy=this->Vy+t2.Vy;
  t.Wz=this->Wz+t2.Wz;
  return t;
}
rigid2d::Twist2D & rigid2d::Twist2D::operator-=(const Twist2D & t)
{
  this->Vx-=t.Vx;
  this->Vy-=t.Vy;
  this->Wz-=t.Wz;
  return *this;
}
rigid2d::Twist2D rigid2d::Twist2D::operator-(const Twist2D & t2)
{
  rigid2d::Twist2D t;
  t.Vx=this->Vx-t2.Vx;
  t.Vy=this->Vy-t2.Vy;
  t.Wz=this->Wz-t2.Wz;
  return t;
}
rigid2d::Twist2D rigid2d::Twist2D::operator*(double a)
{
  rigid2d::Twist2D t;
  t.Vx=this->Vx*a;
  t.Vy=this->Vy*a;
  t.Wz=this->Wz*a;
  return t;
}
rigid2d::Twist2D rigid2d::Twist2D::operator/(double a)
{
  rigid2d::Twist2D t;
  t.Vx=this->Vx/a;
  t.Vy=this->Vy/a;
  t.Wz=this->Wz/a;
  return t;
}


double rigid2d::length(const rigid2d::Vector2D &v)
{
  return sqrt(v.x*v.x+v.y*v.y);
}
double rigid2d::distance(const rigid2d::Vector2D &v1, const rigid2d::Vector2D &v2)
{
  return sqrt((v1.x-v2.x)*(v1.x-v2.x)+(v1.y-v2.y)*(v1.y-v2.y));
}
double rigid2d::angle(const rigid2d::Vector2D &v)
{
  return atan2(v.y,v.x);
}
rigid2d::Vector2D::Vector2D()
{
  x=0;
  y=0;
}
rigid2d::Vector2D::Vector2D(double a, double b)
{
  x=a;
  y=b;
}


