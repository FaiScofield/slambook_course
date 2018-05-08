//
// Created by 高翔 on 2017/12/19.
// 本程序演示如何从Essential矩阵计算R,t
//

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;

#include <sophus/so3.hpp>
#include <algorithm>
#include <iostream>

using namespace std;

bool compare(double a, double b)
{
    return a > b;
}

int main(int argc, char **argv) {

    // 给定Essential矩阵
    Matrix3d E;
    E << -0.0203618550523477, -0.4007110038118445, -0.03324074249824097,
            0.3939270778216369, -0.03506401846698079, 0.5857110303721015,
            -0.006788487241438284, -0.5815434272915686, -0.01438258684486258;

    // 待计算的R,t
    Matrix3d R;
    Vector3d t;

    // SVD and fix sigular values
    // START YOUR CODE HERE
    JacobiSVD<Matrix3d> svd(E, ComputeFullV | ComputeFullU );   // ComputeThinU | ComputeThinV
    MatrixXd U = svd.matrixU(), V = svd.matrixV(), sv = svd.singularValues();

    double DD[3] = {sv(0), sv(1), sv(2)};
    sort(DD, DD+2, compare);
    Matrix3d D = Matrix3d::Zero();
    D(0) = (DD[0]+DD[1])/2;
    D(4) = D(0);
    cout << "U*D*V = " << U*D*V.transpose() << endl;
    // END YOUR CODE HERE

    // set t1, t2, R1, R2 
    // START YOUR CODE HERE
    Matrix3d t_wedge1;
    Matrix3d t_wedge2;
    Matrix3d R1;
    Matrix3d R2;
    Matrix3d R_z1 = AngleAxisd(M_PI/2, Vector3d(0,0,1)).toRotationMatrix();
    Matrix3d R_z2 = AngleAxisd(-M_PI/2, Vector3d(0,0,1)).toRotationMatrix();

    R1 = U * R_z1.transpose() * V.transpose();
    R2 = U * R_z2.transpose() * V.transpose();
    t_wedge1 = U * R_z1 * D * U.transpose();
    t_wedge2 = U * R_z2 * D * U.transpose();
    // END YOUR CODE HERE

    cout << "R1 = " << R1 << endl;
    cout << "R2 = " << R2 << endl;
    cout << "t1 = " << Sophus::SO3d::vee(t_wedge1).transpose() << endl;
    cout << "t2 = " << Sophus::SO3d::vee(t_wedge2).transpose() << endl;

    // check t^R=E up to scale
    Matrix3d tR = t_wedge1 * R1;
    cout << "t^R = " << tR << endl;
    cout << "s * t^R = " << E(0)/tR(0) * tR << endl;

    return 0;
}
