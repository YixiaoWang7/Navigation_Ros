#include <iosfwd> 
#include <math.h>
#include <iostream>
#include <Eigen/Dense>
#include "geometryfitting/geometryfitting.hpp"

Eigen::VectorXf geometryfitting::circlefitting(Eigen::VectorXf x, Eigen::VectorXf y)
{
    int len=x.size();
    double ax=0, ay=0, az=0;
    ax=x.mean();
    ay=y.mean();
    x=x-Eigen::VectorXf::Constant(len,ax);
    y=y-Eigen::VectorXf::Constant(len,ay);
    Eigen::VectorXf z(len);
    z=x.array()*x.array()+y.array()*y.array();
    az=z.mean();
    Eigen::MatrixXf Z(len,4);
    Z.col(0)=z;
    Z.col(1)=x;
    Z.col(2)=y;
    Z.col(3)=Eigen::VectorXf::Constant(len,1);
    Eigen::MatrixXf M;
    M=Z.transpose()*Z/len;
    Eigen::MatrixXf InH;
    InH=Eigen::MatrixXf::Zero(4,4);
    InH(1,1)=1;
    InH(2,2)=1;
    InH(0,3)=0.5;
    InH(3,0)=0.5;
    InH(3,3)=-2*az;
    Eigen::BDCSVD<Eigen::MatrixXf> svd(Z, Eigen::ComputeThinU | Eigen::ComputeThinV);
    // const Eigen::MatrixXf &U = svd.matrixU();
    const Eigen::MatrixXf &V = svd.matrixV();
    const Eigen::VectorXf &S = svd.singularValues();
    Eigen::VectorXf A;
    if (S(3)<1e-12)
    {
        A=V.col(3);
    }else
    {
        Eigen::MatrixXf SM(S.asDiagonal());
        Eigen::MatrixXf Y;
        Y=V*SM*V.transpose();
        Eigen::MatrixXf Q;
        Q=Y*InH*Y;
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eigensolver(Q);
        const Eigen::VectorXf &E = eigensolver.eigenvalues();
        const Eigen::MatrixXf &EV = eigensolver.eigenvectors();
        int index=-1;
        for (int i=0;i<E.size();i++)
        {
            if (E(i)>0)
            {
                index=i;
                break;
            }
        }
        if (index==-1)
        {
            throw "Eigenvalues are all negative.";
        }else
        {
            Eigen::VectorXf Astar=EV.col(index);
            A = Y.colPivHouseholderQr().solve(Astar);
        }
    }
    Eigen::VectorXf result(4);
    result(0)=-A(1)/2.0/A(0);
    result(1)=-A(2)/2.0/A(0);
    result(2)=(A(1)*A(1)+A(2)*A(2)-4.0*A(0)*A(3))/4.0/A(0)/A(0);
    x=x-Eigen::VectorXf::Constant(len,result(0));
    y=y-Eigen::VectorXf::Constant(len,result(1));
    Eigen::VectorXf temp;
    temp=x.array()*x.array()+y.array()*y.array();
    temp=temp-Eigen::VectorXf::Constant(len,result(2));
    result(3)=sqrt(temp.mean());
    result(0)+=ax;
    result(1)+=ay;
    result(2)=sqrt(result(2));

    return result;
}