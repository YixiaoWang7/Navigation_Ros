#include <iosfwd> 
#include <math.h>
#include <iostream>
#include <Eigen/Dense>
#include <ekf/ekf.hpp>
#include <vector>
#include <rigid2d/rigid2d.hpp>

ekf::ekfDiffLaser::ekfDiffLaser()
{
    x.resize(3);
    x<<0,0,0;
    P=1.0e-5*Eigen::MatrixXf::Identity(3,3);
}


ekf::ekfDiffLaser::ekfDiffLaser(const Eigen::Vector3f & initialx)
{
    x=initialx;
    P=1.0e-5*Eigen::MatrixXf::Identity(3,3);
}

Eigen::Vector3f ekf::ekfDiffLaser::ekfCall(const std::vector<int> &observationIndex, const std::vector<Eigen::Vector2f> observation,const Eigen::Vector3f & u)
{
    // if one index or tage is not detected, the state vector will extend its dimension. 
    if (int(observationIndex.size())>0)
    {
        obToGlobal.resize(int(observationIndex.size()));
        if (int(GlobalObservationIndex.size())==0)
        {
            for (int i=0;i<int(observationIndex.size());i++)
            {
                obToGlobal[i]=i;
                GlobalObservationIndex.push_back(observationIndex[i]);
                x.conservativeResize(int(x.size())+2);
                x(int(x.size())-2)=observation[i](0)*cos(x(2)+observation[i](1));
                x(int(x.size())-1)=observation[i](0)*sin(x(2)+observation[i](1));
            }
        }else
        {
            for (int i=0;i<int(observationIndex.size());i++)
            {
                // flag==1 means landmark already exist.
                int flag=0;
                for (int j=0;j<int(GlobalObservationIndex.size());j++)
                {
                    if (observationIndex[i]==GlobalObservationIndex[j])
                    {
                        flag=1;
                        obToGlobal[i]=j; 
                        break;
                    }
                }
                if (flag==0)
                {
                    GlobalObservationIndex.push_back(observationIndex[i]);
                    obToGlobal[i]=int(GlobalObservationIndex.size())-1;
                    x.conservativeResize(int(x.size())+2);
                    x(int(x.size())-2)=observation[i](0)*cos(x(2)+observation[i](1));
                    x(int(x.size())-1)=observation[i](0)*sin(x(2)+observation[i](1));
                }
            }
        } 
    }

    // motion model result
    x_predict=predictx(u);

    // calculate the Q
    Q=calQ(u);

    int dimP=P.rows();
    int dimx=x.size();
    
    // if x extends its dimension
    if (dimP!=dimx)
    {
        P.conservativeResize(dimx,dimx);
        P.block(dimP,0,dimx-dimP,dimx)=Eigen::MatrixXf::Zero(dimx-dimP,dimx);
        P.block(0,dimP,dimP,dimx-dimP)=Eigen::MatrixXf::Zero(dimP,dimx-dimP);
        P.block(dimP,dimP,dimx-dimP,dimx-dimP)=Eigen::MatrixXf::Identity(dimx-dimP,dimx-dimP)*1000;
    }
    
    // predict the covariance matrix P of x
    P.block<3,3>(0,0)=P.block<3,3>(0,0)+Q;

    // if there exists the observations
    if (int(observationIndex.size())>0)
    {
        H=Eigen::MatrixXf::Zero(int(observationIndex.size())*2,dimx);
        R=Eigen::MatrixXf::Zero(int(observationIndex.size())*2,int(observationIndex.size())*2);
        
        Eigen::VectorXf dz(int(observationIndex.size())*2);
        
        for (int i=0;i<int(observationIndex.size());i++)
        {
            
            int f=2*i; 
            int indexToGlobal=obToGlobal[i];
            int linx=3+2*indexToGlobal;
            double dx=0.0001,dy=0.0001,dr=0.01,dr2=0.01;
            
            dx=x_predict(linx)-x_predict(0);
            dy=x_predict(linx+1)-x_predict(1);
            dr2=dx*dx+dy*dy;
            dr=sqrt(dr2);
            H(f,linx)=dx/dr;
            H(f,linx+1)=dy/dr;
            H(f,0)=-dx/dr;
            H(f,1)=-dy/dr;
            
            H(f+1,linx)=-dy/dr2;
            H(f+1,linx+1)=dx/dr2;
            H(f+1,0)=dy/dr2;
            H(f+1,1)=-dx/dr2;
            H(f+1,2)=-1;

            R(f,f)=noiseR;
            R(f+1,f+1)=noiseTheta;

            dz(f)=observation[i](0)-dr;
            dz(f+1)=observation[i](1)-(atan2(dy,dx)-x_predict(2));
            dz(f+1)=rigid2d::normalize_angle(dz(f+1));
        }
        Eigen::MatrixXf S;
        Eigen::MatrixXf K;
        S=H*P*H.transpose()+R;
        K=P*H.transpose()*S.inverse();
        x=x_predict+K*dz;
        P=P-K*H*P;
    }else
    {
        x=x_predict;
    }

    Eigen::Vector3f xestimate;
    xestimate<<x(0),x(1),x(2);

    return xestimate;
    
}
Eigen::VectorXf ekf::ekfDiffLaser::predictx(const Eigen::Vector3f & u)
{
    Eigen::VectorXf x_temp;
    x_temp=x;

    x_temp(2)+=u(2);
    if (fabs(u(2))<1e-10)
    {
        x_temp(0)=x_temp(0)+cos(x(2))*u(0)-sin(x(2))*u(1);
        x_temp(1)=x_temp(1)+sin(x(2))*u(0)+cos(x(2))*u(1);
    }else
    {
        x_temp(0)=x_temp(0)+(sin(x_temp(2))-sin(x(2)))*u(0)/u(2)+(cos(x_temp(2))-cos(x(2)))*u(1)/u(2);
        x_temp(1)=x_temp(1)+(-cos(x_temp(2))+cos(x(2)))*u(0)/u(2)+(sin(x_temp(2))-sin(x(2)))*u(1)/u(2);
    }

    return x_temp;
}

Eigen::MatrixXf ekf::ekfDiffLaser::calQ(const Eigen::Vector3f & u)
{
    Eigen::MatrixXf tempQ;
    Eigen::MatrixXf uc(3,1);
    Eigen::MatrixXf ur(1,3);
    uc<< u(0),u(1),u(2);
    ur<< u(0),u(1),u(2);

    tempQ=uc*ur;
    return tempQ;
}

Eigen::MatrixXf ekf::ekfDiffLaser::returnR()
{
    int nofz;
    nofz=int(x.size())-3;
    Eigen::MatrixXf R_temp(nofz,nofz);
    for (int i=0;i<nofz/2;i++)
    {
        int f=2*i;
        R_temp(f,f)=noiseR;
        R_temp(f+1,f+1)=noiseTheta;
    }
    
    return R_temp;
}
Eigen::MatrixXf ekf::ekfDiffLaser::returnH(const Eigen::VectorXf & x_temp)
{
    Eigen::MatrixXf H_temp;
    H_temp=Eigen::MatrixXf::Zero(int(x_temp.size())-3,int(x_temp.size()));
    for (int i=0;i<int(H_temp.rows())/2;i++)
    {
        int f=2*i; 
        int linx=3+f;
        float dx=0.0001,dy=0.0001,dr=0.01,dr2=0.01;
        

        dx=x_temp(linx)-x_temp(0);
        
        dy=x_temp(linx+1)-x_temp(1);
        
        dr2=dx*dx+dy*dy;
        dr=sqrt(dr2);
        
        H_temp(f,linx)=dx/dr;
        
        H_temp(f,linx+1)=dy/dr;
        H_temp(f,0)=-dx/dr;
        H_temp(f,1)=-dy/dr;

        H_temp(f+1,linx)=-dy/dr2;
        H_temp(f+1,linx+1)=dx/dr2;
        H_temp(f+1,0)=dy/dr2;
        H_temp(f+1,1)=-dx/dr2;
        H_temp(f+1,2)=-1;
    }

    return H_temp;
}
Eigen::MatrixXf ekf::ekfDiffLaser::returnP()
{
    return P;
}
Eigen::VectorXf ekf::ekfDiffLaser::returnx()
{
    return x;
}
Eigen::VectorXf ekf::ekfDiffLaser::returnz_predict(const Eigen::Vector3f & u)
{
    Eigen::VectorXf x_temp;
    x_temp=predictx(u);

    z_predict=Eigen::VectorXf::Zero(int(x_temp.size())-3);
    for (int i=0;i<int(z_predict.size())/2;i++)
    {
        int f=2*i;
        double dx=0.0001,dy=0.0001,dr=0.01;
        dx=x_temp(3+f)-x_temp(0);
        dy=x_temp(4+f)-x_temp(1);
        dr=sqrt(dx*dx+dy*dy);
        z_predict(f)=dr;
        z_predict(f+1)=atan2(dy,dx)-x(2);
    }
    return z_predict;
}


