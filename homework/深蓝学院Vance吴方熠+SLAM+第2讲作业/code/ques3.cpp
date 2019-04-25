#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;

int main(int argc, char** argv)
{
    Eigen::Quaternion<float> q1(0.55, 0.3, 0.2, 0.2);
    Eigen::Quaternion<float> q2(-0.1, 0.3, -0.7, 0.2);
    q1 = q1.normalized();
    q2 = q2.normalized();
    cout << "q1:" << endl << q1.coeffs() << endl << endl;
    cout << "q2:" << endl << q2.coeffs() << endl << endl;

    Eigen::Vector3f t1(0.7, 1.1, 0.2);
    Eigen::Vector3f t2(-0.1, 0.4, 0.8);
    Eigen::Matrix<float,4,1> p1, p2;
    p1 << 0.5, -0.1, 0.2, 1;

    Eigen::Isometry3f T1 = Eigen::Isometry3f::Identity();
    Eigen::Isometry3f T2 = Eigen::Isometry3f::Identity();
    T1.rotate(q1);  // q1.toRotationMatrix()
    T1.pretranslate(t1);
    T2.rotate(q2);
    T2.pretranslate(t2);
    cout << "T1:" << endl << T1.matrix() << endl << endl;
    cout << "T2:" << endl << T2.matrix() << endl << endl;

    p2 = T2 * T1.inverse() * p1;
    cout << "p2:" << endl << p2 << endl << endl;

    return 0;
}
