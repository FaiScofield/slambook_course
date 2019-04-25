#include <sophus/se3.h>
#include <string>
#include <iostream>
#include <fstream>

// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>
#include <Eigen/Core>

using namespace std;

typedef vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> vector_poses;

// path to trajectory file
string trajectory_file = "/home/vance/slam_ws/slambook/class/05/L5/code/compare.txt";

// function for read trajectory file and get poses
void getPoses(const string& filename, vector_poses& p_e, vector_poses& p_g);

// fuction for ICP
void ICP_solve(const vector_poses& p_e, const vector_poses& p_g, Eigen::Matrix3d& R, Eigen::Vector3d& t);

// function for plotting trajectory, don't edit this code
// start point is red and end point is blue
void DrawTrajectory(const vector_poses& poses1, const vector_poses& poses2);

// main
int main(int argc, char **argv) {

    vector_poses poses_e, poses_g;
    getPoses(trajectory_file, poses_e, poses_g);
//    DrawTrajectory(poses_e, poses_g);

    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    ICP_solve(poses_e, poses_g, R, t);

    for (auto& pe : poses_e) {
        pe.translation() = R * pe.translation() + t;
    }

    DrawTrajectory(poses_e, poses_g);

    return 0;
}

/*******************************************************************************************/
void DrawTrajectory(const vector_poses& poses1, const vector_poses& poses2) {
    if (poses1.empty() || poses2.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Data logger object
    pangolin::DataLog log;
    // Optionally add named labels
    std::vector<std::string> labels;
    labels.push_back(std::string("poses_estimate"));
    labels.push_back(std::string("poses_groundtruth"));
    log.SetLabels(labels);

    // Define Projection and initial ModelView matrix
    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while ( !pangolin::ShouldQuit() ) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        // draw poses 1
        for (size_t i=0; i<poses1.size()-1; i++) {
            glColor3f(1.0f, 0.0f, 0.0f);
            glBegin(GL_LINES);
            auto p1 = poses1[i], p2 = poses1[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }

        // draw poses 2
        for (size_t j=0; j<poses2.size()-1; j++) {
            glColor3f(0.0f, 1.0f, 0.0f);
            glBegin(GL_LINES);
            auto p1 = poses2[j], p2 = poses2[j + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }

        pangolin::FinishFrame();
        usleep(5000000);
    }

    return;
}


void getPoses(const string& filename, vector_poses& p_e, vector_poses& p_g){
    // implement pose reading code
    Eigen::Vector3d t_g(0, 0, 0), t_e(0, 0, 0);
    Eigen::Quaternion<double> q_g(0, 0, 0, 0), q_e(0, 0, 0, 0);
    string str;
    double timestamp[2];

    ifstream fin(filename.c_str());
    if (!fin) {
        cerr << "compare file can not be found!" << endl;
        return;
    }

    while (!fin.eof()) {
        getline(fin, str);
        istringstream ss(str);

        ss >> timestamp[0] >> t_e[0] >> t_e[1] >> t_e[2]
           >> q_e.x() >> q_e.y() >> q_e.z() >> q_e.w()
           >> timestamp[1] >> t_g[0] >> t_g[1] >> t_g[2]
           >> q_g.x() >> q_g.y() >> q_g.z() >> q_g.w();

        Sophus::SE3 T_e(q_e, t_e), T_g(q_g, t_g);

        p_e.push_back(T_e);
        p_g.push_back(T_g);
    }
    fin.close();
}


void ICP_solve(const vector_poses& p_e, const vector_poses& p_g, Eigen::Matrix3d& R, Eigen::Vector3d& t){
    // center mass
    Eigen::Vector3d center_e, center_g;
    int N = p_e.size();
    for (int i=0; i<N; i++) {
        center_e += p_e[i].translation();
        center_g += p_g[i].translation();
    }
    center_e /= N;
    center_g /= N;

    // remove the center
    vector<Eigen::Vector3d> t_e, t_g;
    for (int i=0; i<N; i++) {
        t_e.push_back( p_e[i].translation() - center_e );
        t_g.push_back( p_g[i].translation() - center_g );
    }

    // compute t_e * t_g^T
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for (int i=0; i<N; i++) {
        W += t_g[i] * t_e[i].transpose();
    }

    // SVD on W
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    R = U * V.transpose();
    t = center_g - R * center_e;
}



