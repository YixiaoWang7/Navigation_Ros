#include <iostream>
#include <iosfwd> // contains forward definitions for iostream objects
#include "rigid2d.hpp"



int main(){

rigid2d::Transform2D Tab;
rigid2d::Transform2D Tba;
rigid2d::Transform2D Tbc;
rigid2d::Transform2D Tcb;
rigid2d::Transform2D Tac;
rigid2d::Transform2D Tca;
std::cout<<"Enter the Tab:"<<std::endl;
std::cin>>Tab;
std::cout<<"Enter the Tbc:"<<std::endl;
std::cin>>Tbc;

std::cout<<std::endl<<"Start computing."<<std::endl;
Tba=Tab.inv();
Tcb=Tbc.inv();
Tac=Tab*Tbc;
Tca=Tac.inv();

std::cout<<std::endl<<"Tab:"<<std::endl;
std::cout<<Tab;

std::cout<<"Tba:"<<std::endl;
std::cout<<Tba;

std::cout<<"Tbc:"<<std::endl;
std::cout<<Tbc;

std::cout<<"Tcb:"<<std::endl;
std::cout<<Tcb;

std::cout<<"Tac:"<<std::endl;
std::cout<<Tac;

std::cout<<"Tca:"<<std::endl;
std::cout<<Tca;


char TorV;
char frame;
std::cout<<std::endl<<"Transform twist or vector? (t or v)"<<std::endl;
std::cin>>TorV;
if (TorV=='v')
{
rigid2d::Vector2D v;
rigid2d::Vector2D va;
rigid2d::Vector2D vb;
rigid2d::Vector2D vc;
std::cout<<std::endl<<"Enter the target 2D vector:"<<std::endl;
std::cin>>v;

std::cout<<std::endl<<"Enter the frame of the vector: a, b or c"<<std::endl;
std::cin>>frame;

std::cout<<std::endl<<"Start computing."<<std::endl;

if (frame=='a')
{
  va=v;
  vb=Tba(v);
  vc=Tca(v);
}
else if (frame=='b')
{
  vb=v;
  va=Tab(v);
  vc=Tcb(v);
}
else
{
  vc=v;
  va=Tac(v);
  vb=Tbc(v);
}

std::cout<<std::endl<<"va: "<<va<<std::endl;
std::cout<<"vb: "<<vb<<std::endl;
std::cout<<"vc: "<<vc<<std::endl;
}
else if (TorV=='t')
{
rigid2d::Twist2D t,ta,tb,tc;
std::cout<<std::endl<<"Enter the target 2D twist:"<<std::endl;
std::cin>>t;

std::cout<<std::endl<<"Enter the frame of the twist: a, b or c"<<std::endl;
std::cin>>frame;

std::cout<<std::endl<<"Start computing."<<std::endl;

if (frame=='a')
{
  ta=t;
  tb=Tba(t);
  tc=Tca(t);
}
else if (frame=='b')
{
  tb=t;
  ta=Tab(t);
  tc=Tcb(t);
}
else
{
  tc=t;
  ta=Tac(t);
  tb=Tbc(t);
}

std::cout<<std::endl<<"ta: "<<ta<<std::endl;
std::cout<<"tb: "<<tb<<std::endl;
std::cout<<"tc: "<<tc<<std::endl;
}
return 0;
}
