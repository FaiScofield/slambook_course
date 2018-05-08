#include <boost/concept_check.hpp>

#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.h>

#include "rotation.h"
#include "projection.h"

class CameraSE3BAL
{
public:
    CameraSE3BAL() {}
    CameraSE3BAL(Eigen::VectorXd camera_) {
        Eigen::Matrix<double, 6, 1> se3;
        se3.head<3>() = camera_.block<3, 1>(3, 0);  // 注意：Sophus中,平移在前,旋转在后
        se3.tail<3>() = camera_.head<3>();

        _SE3 = Sophus::SE3::exp(se3);
        _f = camera_[6];
        _k1 = camera_[7];
        _k2 = camera_[8];
    }

public:
    Sophus::SE3 _SE3;
    double _f;
    double _k1;
    double _k2;
};

class VertexCameraBAL : public g2o::BaseVertex<9, CameraSE3BAL>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexCameraBAL() {}

    virtual bool read ( std::istream& /*is*/ )
    {
        return false;
    }

    virtual bool write ( std::ostream& /*os*/ ) const
    {
        return false;
    }

    virtual void setToOriginImpl() {}

    virtual void oplusImpl ( const double* update_ )
    {
        Eigen::Matrix<double, 6, 1> updated_se3;
        updated_se3 << update_[3], update_[4], update_[5], update_[0], update_[1], update_[2];
        _estimate._SE3 = Sophus::SE3::exp(updated_se3) * _estimate._SE3;
        _estimate._f += update_[6];
        _estimate._k1 += update_[7];
        _estimate._k2 += update_[8];
    }

};

class VertexPointBAL : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexPointBAL() {}

    virtual bool read ( std::istream& /*is*/ )
    {
        return false;
    }

    virtual bool write ( std::ostream& /*os*/ ) const
    {
        return false;
    }

    virtual void setToOriginImpl() {}

    virtual void oplusImpl ( const double* update )
    {
        Eigen::Vector3d::ConstMapType v ( update );
        _estimate += v;
    }
};

class EdgeObservationBAL : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, VertexCameraBAL, VertexPointBAL>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeObservationBAL() {}

    virtual bool read ( std::istream& /*is*/ )
    {
        return false;
    }

    virtual bool write ( std::ostream& /*os*/ ) const
    {
        return false;
    }

    // The virtual function comes from the Edge base class. Must define if you use edge.
    virtual void computeError() override
    {
        const VertexCameraBAL* cam = static_cast<const VertexCameraBAL*>(vertex(0));
        const VertexPointBAL* point = static_cast<const VertexPointBAL*>(vertex(1));

        Eigen::Vector3d Pc = cam->estimate()._SE3 * point->estimate(); // camera frame
        double xn = -Pc[0]/Pc[2];   // normolized frame
        double yn = -Pc[1]/Pc[2];
        const double& f = cam->estimate()._f;
        const double& k1 = cam->estimate()._k1;
        const double& k2 = cam->estimate()._k2;
        double r2 = xn*xn + yn*yn;
        double distortion = 1.0 + r2 * (k1 + k2 * r2);

        _error[0] = _measurement[0] - f * distortion * xn;
        _error[1] = _measurement[1] - f * distortion * yn;
    }

    virtual void linearizeOplus() override
    {
        const VertexCameraBAL* cam = static_cast<const VertexCameraBAL*>(vertex(0));
        const VertexPointBAL* point = static_cast<const VertexPointBAL*>(vertex(1));

        Eigen::Vector3d Pc = cam->estimate()._SE3 * point->estimate(); // position
        double xc = Pc[0];
        double yc = Pc[1];
        double zc = Pc[2];
        double xn = -xc/zc;
        double yn = -yc/zc;
        const double& f = cam->estimate()._f;
        const double& k1 = cam->estimate()._k1;
        const double& k2 = cam->estimate()._k2;
        double r2 = xn*xn + yn*yn;
        double rp = 1.0 + k1*r2 + k2*r2*r2;

        Eigen::Matrix<double, 2, 6> de_dkesi;   // kesi is the cam pose
        Eigen::Matrix<double, 2, 3> de_dPc;     // Pc = exp(kesi^) * Pw;
        Eigen::Matrix<double, 3, 6> dPc_dkesi = Eigen::Matrix<double, 3, 6>::Zero();
        Eigen::Matrix<double, 2, 3> de_dPw;
        Eigen::Matrix<double, 3, 3> dPc_dPw;
        Eigen::Vector2d de_df, de_dk1, de_dk2;

        double zc_2 = zc * zc;
        double zc_3 = zc_2 * zc;
        double rp2 = k1 + 2*k2*r2;

        // all parts of jacobian
        de_dPc(0, 0) = f*rp/zc + 2*f*xc*xc*rp2/zc_3;
        de_dPc(0, 1) = 2*f*xc*yc*rp2/zc_3;
        de_dPc(0, 2) = -f*xc*rp/zc_2 - 2*f*xc*r2*rp2/zc_2;
        de_dPc(1, 0) = 2*f*xc*yc*rp2/zc_3;
        de_dPc(1, 1) = f*rp/zc + 2*f*yc*yc*rp2/zc_3;
        de_dPc(1, 2) = -f*yc*rp/zc_2 - 2*f*yc*r2*rp2/zc_2;

        // dPc_dkesi = [I, -Pc^]_3x6
        dPc_dkesi(0, 0) = 1;
        dPc_dkesi(0, 4) = zc;
        dPc_dkesi(0, 5) = -yc;
        dPc_dkesi(1, 1) = 1;
        dPc_dkesi(1, 3) = -zc;
        dPc_dkesi(1, 5) = xc;
        dPc_dkesi(2, 2) = 1;
        dPc_dkesi(2, 3) = yc;
        dPc_dkesi(2, 4) = -xc;

        de_dkesi = de_dPc * dPc_dkesi;

        // de_df, de_dk1, de_dk2
        de_df(0, 0) = xc*rp/zc;
        de_df(1, 0) = yc*rp/zc;
        de_dk1(0, 0) = f*xc*r2/zc;
        de_dk1(1, 0) = f*yc*r2/zc;
        de_dk2(0, 0) = f*xc*r2*r2/zc;
        de_dk2(1, 0) = f*yc*r2*r2/zc;

        // de_dPw
        dPc_dPw = cam->estimate()._SE3.rotation_matrix();
        de_dPw = de_dPc * dPc_dPw;

        // jacobian
        _jacobianOplusXi.block<2,6>(0,0) = de_dkesi;
        _jacobianOplusXi.block<2,1>(0,6) = de_df;
        _jacobianOplusXi.block<2,1>(0,7) = de_dk1;
        _jacobianOplusXi.block<2,1>(0,8) = de_dk2;
//        _jacobianOplusXi = -1 * _jacobianOplusXi;
        _jacobianOplusXj = de_dPw;
    }
};
