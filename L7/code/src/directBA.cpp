#include <iostream>
#include <boost/format.hpp>

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include <Eigen/Core>
#include <sophus/se3.h>
#include <pangolin/pangolin.h>

#include <opencv2/opencv.hpp>

using namespace std;

typedef vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> VecSE3;
typedef vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecVec3d;

// global variables
string pose_file = "../data/poses.txt";
string points_file = "../data/points.txt";

// intrinsics
float fx = 277.34;
float fy = 291.402;
float cx = 312.234;
float cy = 239.777;
int offset[] = {-2, -2,
                -2, -1,
                -2, 0,
                -2, 1,
                -1, -2,
                -1, -1,
                -1, 0,
                -1, 1,
                0, -2,
                0, -1,
                0, 0,
                0, 1,
                1, -2,
                1, -1,
                1, 0,
                1, 1};

// bilinear interpolation
inline float GetPixelValue(const cv::Mat &img, float x, float y) {
    uchar *data = &img.data[int(y) * img.step + int(x)];
    float xx = x - floor(x);
    float yy = y - floor(y);
    return float(
            (1 - xx) * (1 - yy) * data[0] +
            xx * (1 - yy) * data[1] +
            (1 - xx) * yy * data[img.step] +
            xx * yy * data[img.step + 1]
        );
}

// g2o vertex that use sophus::SE3 as pose
class VertexSophus : public g2o::BaseVertex<6, Sophus::SE3> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VertexSophus() {}
    ~VertexSophus() {}

    bool read(std::istream &is) {return true;}
    bool write(std::ostream &os) const {return true;}

    virtual void setToOriginImpl()
    {
        _estimate = Sophus::SE3();
    }

    virtual void oplusImpl(const double *update_)
    {
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> update(update_);
        setEstimate(Sophus::SE3::exp(update) * estimate());
    }
};

// TODO edge of projection error, implement it
// 16x1 error, which is the errors in patch
typedef Eigen::Matrix<double,16,1> Vector16d;
class EdgeDirectProjection : public g2o::BaseBinaryEdge<16, Vector16d, g2o::VertexSBAPointXYZ, VertexSophus> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeDirectProjection(vector<float> color, cv::Mat &target)
    {
        this->origColor = color;
        this->targetImg = target;
    }

    ~EdgeDirectProjection() {}

    virtual void computeError() override
    {
        // TODO START YOUR CODE HERE
        // compute projection error ...
        const g2o::VertexSBAPointXYZ* point = static_cast<const g2o::VertexSBAPointXYZ*>(vertex(0));
        const VertexSophus* cam = static_cast<const VertexSophus*>(vertex(1));
        Eigen::Vector3d Pc = cam->estimate() * point->estimate();
        double u = fx * Pc[0]/Pc[2] + cx;
        double v = fy * Pc[1]/Pc[2] + cy;

        if (u-3<0 || u+2>targetImg.cols || v-3<0 || v+2>targetImg.rows) {
            this->setLevel(1);
            for(size_t i = 0; i < 16; ++i) { _error[i] = 0; }
        }
        else {
            for(size_t i = 0; i < 16; ++i){
                _error[i] = origColor[i] - GetPixelValue(targetImg, u+offset[2*i], v+offset[2*i+1]);
            }
        }
        // END YOUR CODE HERE
    }
    virtual void linearizeOplus() override
    {
        if(level() == 1) {
            _jacobianOplusXi = Eigen::Matrix<double, 16, 3>::Zero();
            _jacobianOplusXj = Eigen::Matrix<double, 16, 6>::Zero();
            return;
        }

        const g2o::VertexSBAPointXYZ* pPoint = static_cast<const g2o::VertexSBAPointXYZ*> (vertex(0));
        const VertexSophus* pCamera = static_cast<const VertexSophus*> (vertex(1));
        Eigen::Vector3d Pc = pCamera->estimate() * pPoint->estimate();
        float x = Pc[0];
        float y = Pc[1];
        float z = Pc[2];
        float u = fx * x / z + cx;
        float v = fy * y / z + cy;

        Eigen::Matrix<double, 2, 3> dPuv_dPc;
        dPuv_dPc(0, 0) = fx / z;
        dPuv_dPc(0, 1) = 0;
        dPuv_dPc(0, 2) = -fx * x / (z * z);
        dPuv_dPc(1, 0) = 0;
        dPuv_dPc(1, 1) = fy / z;
        dPuv_dPc(1, 2) = -fy * y / (z * z);

        Eigen::Matrix<double, 3, 6> dPc_dkesi = Eigen::Matrix<double, 3, 6>::Zero();
        dPc_dkesi(0, 0) = 1;
        dPc_dkesi(0, 4) = z;
        dPc_dkesi(0, 5) = -y;
        dPc_dkesi(1, 1) = 1;
        dPc_dkesi(1, 3) = -z;
        dPc_dkesi(1, 5) = x;
        dPc_dkesi(2, 2) = 1;
        dPc_dkesi(2, 3) = y;
        dPc_dkesi(2, 4) = -x;

        Eigen::Matrix<double, 1, 2> dI_duv;
        for(size_t i = 0; i < 16; i++){
            dI_duv(0, 0) = (GetPixelValue(targetImg, u+offset[2*i]+1, v+offset[2*i+1])
                    - GetPixelValue(targetImg, u+offset[2*i]-1, v+offset[2*i+1])) / 2;
            dI_duv(0, 1) = (GetPixelValue(targetImg, u+offset[2*i], v+offset[2*i+1]+1)
                    - GetPixelValue(targetImg, u+offset[2*i], v+offset[2*i+1]-1)) / 2;
            _jacobianOplusXi.block<1,3>(i, 0) = -dI_duv * dPuv_dPc * pCamera->estimate().rotation_matrix();
            _jacobianOplusXj.block<1,6>(i, 0) = -dI_duv * dPuv_dPc * dPc_dkesi;
        }
        return;
    }
    // Let g2o compute jacobian for you

    virtual bool read(istream &in) {return true;}
    virtual bool write(ostream &out) const {return true;}

private:
    cv::Mat targetImg;  // the target image
    vector<float> origColor;   // 16 floats, the color of this point
};

// plot the poses and points for you, need pangolin
void Draw(const VecSE3 &poses, const VecVec3d &points) {
    if (poses.empty() || points.empty()) {
        cerr << "parameter is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (!pangolin::ShouldQuit()) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

        // draw poses
        float sz = 0.1;
        int width = 640, height = 480;
        for (auto &Tcw: poses) {
            glPushMatrix();
            Sophus::Matrix4f m = Tcw.inverse().matrix().cast<float>();
            glMultMatrixf((GLfloat *) m.data());
            glColor3f(1, 0, 0);
            glLineWidth(2);
            glBegin(GL_LINES);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
            glEnd();
            glPopMatrix();
        }

        // points
        glPointSize(2);
        glBegin(GL_POINTS);
        for (size_t i = 0; i < points.size(); i++) {
            glColor3f(0.0, points[i][2]/4, 1.0-points[i][2]/4);
            glVertex3d(points[i][0], points[i][1], points[i][2]);
        }
        glEnd();

        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}

int main(int argc, char **argv) {
    // read poses data
    VecSE3 poses;
    VecVec3d points;
    ifstream fin(pose_file);

    while (!fin.eof()) {
        double timestamp = 0;
        fin >> timestamp;
        if (timestamp == 0) break;
        double data[7];
        for (auto &d: data) fin >> d;
        poses.push_back(Sophus::SE3(
                Eigen::Quaterniond(data[6], data[3], data[4], data[5]),
                Eigen::Vector3d(data[0], data[1], data[2]) )
                );
        if (!fin.good()) break;
    }
    fin.close();

    // read points data
    vector<vector<float>> color;
    fin.open(points_file);
    while (!fin.eof()) {
        double xyz[3] = {0};
        for (int i = 0; i < 3; i++) { fin >> xyz[i]; }
        if (xyz[0] == 0) break;
        points.push_back(Eigen::Vector3d(xyz[0], xyz[1], xyz[2]));

        vector<float> c(16);
        for (int i = 0; i < 16; i++) {
            fin >> c[i];
        }
        color.push_back(c);

        if (!fin.good()) break;
    }
    fin.close();

    cout << "poses: " << poses.size() << ", points: " << points.size() << endl;

    // read images
    vector<cv::Mat> images;
    boost::format fmt("../data/%d.png");
    for (int i = 0; i < 7; i++) {
        images.push_back(cv::imread((fmt % i).str(), 0));
    }

    // build optimization problem
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3> > DirectBlock;  // 求解的向量是6＊1的
    std::unique_ptr<DirectBlock::LinearSolverType> linearSolver(new g2o::LinearSolverDense<DirectBlock::PoseMatrixType>());
    std::unique_ptr<DirectBlock> solver_ptr(new DirectBlock(std::move(linearSolver) ) );
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr) ); // L-M
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    // TODO add vertices, edges into the graph optimizer
    // START YOUR CODE HERE
    for(size_t i = 0; i < points.size(); ++i){
        g2o::VertexSBAPointXYZ* pPoint = new g2o::VertexSBAPointXYZ();
        pPoint->setEstimate(points[i]);
        pPoint->setId(i);
        pPoint->setMarginalized(true);

        optimizer.addVertex(pPoint);    // add points vertex
    }

    for(size_t j = 0; j < poses.size(); ++j){
        VertexSophus* pCamera = new VertexSophus();
        pCamera->setEstimate(poses[j]);
        pCamera->setId(j + points.size());

        optimizer.addVertex(pCamera);   // add camera pose vertex
    }

    for(size_t i = 0; i < points.size(); ++i) {
        for(size_t j = 0; j < poses.size(); ++j) {
            Eigen::Vector3d Pc = poses[j] * points[i];  // Pc: point[i]'s pose in camera[j]
            float u = fx * Pc[0] / Pc[2] + cx;
            float v = fy * Pc[1] / Pc[2] + cy;
            // make the projected point away from the image border
            if(u>10 && u+10<images[0].cols && v>10 && v+10<images[0].rows) {
                EdgeDirectProjection* edge = new EdgeDirectProjection(color[i], images[j]);
                edge->setVertex(0, dynamic_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(i)));
                edge->setVertex(1, dynamic_cast<VertexSophus*>(optimizer.vertex(j+points.size())) );
                edge->setInformation(Eigen::Matrix<double, 16, 16>::Identity());
                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                rk->setDelta(1.0);
                edge->setRobustKernel(rk);

                optimizer.addEdge(edge) ;
            }
        }
    }
    // END YOUR CODE HERE

    // perform optimization
    optimizer.initializeOptimization(0);
    optimizer.optimize(200);

    // TODO fetch data from the optimizer
    // START YOUR CODE HERE
    for(size_t i = 0; i < points.size(); ++i) {
        points[i] = dynamic_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(i))->estimate();
    }
    for(size_t j = 0; j < poses.size(); ++j) {
        poses[j] = dynamic_cast<VertexSophus*>(optimizer.vertex(j+points.size()))->estimate();
    }
    // END YOUR CODE HERE

    // plot the optimized points and poses
    Draw(poses, points);

    return 0;
}



