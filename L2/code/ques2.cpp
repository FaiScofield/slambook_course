#include <iostream>
#include <cstdlib>
#include <ctime>

#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/QR>

using namespace std;

int main(int argc, char** argv)
{
    srand((unsigned)time(NULL));

    Eigen::MatrixXd A = Eigen::MatrixXd::Random(100, 100);
    Eigen::VectorXd b = Eigen::VectorXd::Random(100);

    Eigen::VectorXd x1 = (A.adjoint()*A).ldlt().solve(A.adjoint()*b);   // 正或负半定
    Eigen::VectorXd x2 = (A.adjoint()*A).llt().solve(A.adjoint()*b);    // 正定
    Eigen::VectorXd x3 = A.householderQr().solve(b);
    Eigen::VectorXd x4 = A.colPivHouseholderQr().solve(b);
    Eigen::VectorXd x5 = A.fullPivHouseholderQr().solve(b);

    cout << "x1(using ldlt Cholesky):" << endl << x1 << endl << endl;
    cout << "x2(using llt Cholesky):" << endl << x2 << endl << endl;
    cout << "x3(using householderQr):" << endl << x3 << endl << endl;
    cout << "x4(using colPivHouseholderQr):" << endl << x4 << endl << endl;
    cout << "x5(using fullPivHouseholderQr):" << endl << x5 << endl << endl;

    return 0;
}
