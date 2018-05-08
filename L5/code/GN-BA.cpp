#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;

#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>

#include "sophus/se3.h"

using namespace std;

typedef vector<Vector3d, Eigen::aligned_allocator<Vector3d>> VecVector3d;
typedef vector<Vector2d, Eigen::aligned_allocator<Vector3d>> VecVector2d;
typedef Matrix<double, 6, 1> Vector6d;

string p3d_file = "../p3d.txt";
string p2d_file = "../p2d.txt";

int main(int argc, char **argv) {

    VecVector2d p2d;
    VecVector3d p3d;
    Matrix3d K;
    double fx = 520.9, fy = 521.0, cx = 325.1, cy = 249.7;
    K << fx, 0, cx, 0, fy, cy, 0, 0, 1;

    // load points into p3d and p2d
    // START YOUR CODE HERE
    fstream p2d_fin(p2d_file.c_str()), p3d_fin(p3d_file.c_str());
    if (!p2d_fin || !p3d_fin) {
        cerr << "file(s) can not be found!" << endl;
        return -1;
    }

    // read the data
    while (!p2d_fin.eof() && !p2d_fin.eof()) {
        double p2d_data[2] = {0}, p3d_data[3] = {0};
        for (auto & p2 : p2d_data) p2d_fin >> p2;
        for (auto & p3 : p3d_data) p3d_fin >> p3;

        p2d.push_back( Vector2d(p2d_data[0], p2d_data[1]) );
        p3d.push_back( Vector3d(p3d_data[0], p3d_data[1], p3d_data[2]) );
    }
    p2d_fin.close();
    p3d_fin.close();
    // END YOUR CODE HERE
    assert(p3d.size() == p2d.size());   // 如果条件返回错误，则终止程序执行

    int iterations = 100;
    double cost = 0, lastCost = 0;
    int nPoints = p3d.size();
    cout << "points: " << nPoints << endl;

    Sophus::SE3 T_esti(Matrix3d::Identity(), Vector3d::Zero()); // estimated pose

    for (int iter = 0; iter < iterations; iter++) {
        Matrix<double, 6, 6> H = Matrix<double, 6, 6>::Zero();
        Vector6d b = Vector6d::Zero();
        Vector2d e; // 重投影误差

        cost = 0;   // 目标函数
        // compute cost
        for (int i = 0; i < nPoints; i++) {
            // compute cost for p3d[I] and p2d[I]
            // START YOUR CODE HERE
            Vector4d p_c = T_esti.matrix() * Vector4d(p3d[i][0], p3d[i][1], p3d[i][2], 1);
            double x = p_c[0];
            double y = p_c[1];
            double z = p_c[2];

            double u_e = p2d[i][0] - (fx*x/z + cx);
            double v_e = p2d[i][1] - (fy*y/z + cy);
            e << u_e, v_e;

            cost += 0.5 * e.transpose() * e;
            // END YOUR CODE HERE

            // compute jacobian
            Matrix<double, 2, 6> J; // g2o旋转在前，平移在后
            // START YOUR CODE HERE
            J(0, 0) = -fx / z;
            J(0, 1) = 0;
            J(0, 2) = fx*x / (z*z);
            J(0, 3) = fx*x*y / (z*z);
            J(0, 4) = -fx - fx*x*x / (z*z);
            J(0, 5) = fx*y / z;
            J(1, 0) = 0;
            J(1, 1) = -fy / z;
            J(1, 2) = fy*y / (z*z);
            J(1, 3) = fy + fy*y*y / (z*z);
            J(1, 4) = -fy*x*y / (z*z);
            J(1, 5) = -fy*x / z;
            // END YOUR CODE HERE

            H += J.transpose() * J;
            b += -J.transpose() * e;
        }

        // solve dx, H*dx=b
        Vector6d dx;

        // START YOUR CODE HERE
        dx = H.ldlt().solve(b);
        // END YOUR CODE HERE

        if (std::isnan(dx[0])) {
            cout << "result is nan!" << endl;
            break;
        }

        if (iter > 0 && cost > lastCost) {
            // cost increase, update is not good
            cout << "cost: " << cost << ", last cost: " << lastCost << endl;
            cout << "cost increase, update is not good." << endl;

            break;  // no update in this iteration
        }

        // update your estimation
        // START YOUR CODE HERE
        T_esti = Sophus::SE3::exp(dx) * T_esti;
        // END YOUR CODE HERE

        lastCost = cost;

        cout << "iteration " << iter << " cost = " << setprecision(12) << cost << endl;
    }

    cout << "estimated pose: \n" << setprecision(4) << T_esti.matrix() << endl;
    return 0;
}
