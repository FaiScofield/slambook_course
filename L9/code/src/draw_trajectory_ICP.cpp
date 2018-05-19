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
string trajectory_file = "../my_kitti_keyframe_trajectory.txt";
string trajectory_index_file = "../my_kitti_keyframe_index.txt";
string groundtruth_file = "/media/vance/00077298000E1760/dataset/KITTI/poses/05.txt";

// function for read keyframe trajectory file and get poses
void getPoses(const string& filename, vector_poses& pose);

// function for read keyframe trajectory index file and get keyframe index
void getPosesIndex(const string& filename, vector<int>& poses_index);

// fuction for ICP
void ICP_solve(const vector_poses& p_e, const vector_poses& p_g, Eigen::Matrix3d& R, Eigen::Vector3d& t);

// function for plotting trajectory, don't edit this code
// start point is red and end point is blue
void DrawTrajectory(const vector_poses& poses1, const vector_poses& poses2);

// main
int main(int argc, char **argv) {

    vector_poses poses_e, poses_g, poses_g_cut;
    vector<int> poses_e_index;

    getPoses(trajectory_file, poses_e);
    getPoses(groundtruth_file, poses_g);
    getPosesIndex(trajectory_index_file, poses_e_index);
    cout << "get trajectory posese: " << poses_e.size() << endl;
    cout << "get groundtruth posese: " << poses_g.size() << endl;
    cout << "get trajectory index number: " << poses_e_index.size() << endl;

    // cut poses_g to make the poses in same time
    for (auto& i : poses_e_index) {
        poses_g_cut.push_back(poses_g[i]);
    }
    if (poses_e.size() != poses_g_cut.size()) {
        cerr << "keep two pose size same failed." << endl;
        return -1;
    }
    cout << "get groundtruth_cut posese: " << poses_g_cut.size() << endl;

//    DrawTrajectory(poses_e, poses_g_cut);

    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    ICP_solve(poses_e, poses_g_cut, R, t);

    for (auto& pe : poses_e) {
        pe.translation() = R * pe.translation() + t;
    }

    cout << "Draw the trajectories..." << endl;
    cout << "Red trajectory for estimate, green trajectory for groundtruth." << endl;
    DrawTrajectory(poses_e, poses_g_cut);

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
        usleep(500);
    }

    return;
}


void getPoses(const string& filename, vector_poses& pose){
    // implement pose reading code
    Eigen::Vector3d t(0, 0, 0);
    Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
    Eigen::Matrix4d T_m = Eigen::Matrix4d::Zero();

    string str;
//    double timestamp;

    ifstream fin(filename.c_str());
    if (!fin) {
        cerr << "Trajectory file can not be found!" << endl;
        return;
    }

    while (!fin.eof()) {
        getline(fin, str);
        istringstream ss(str);

        ss >> T_m(0,0) >> T_m(0,1) >> T_m(0,2) >> T_m(0,3)
           >> T_m(1,0) >> T_m(1,1) >> T_m(1,2) >> T_m(1,3)
           >> T_m(2,0) >> T_m(2,1) >> T_m(2,2) >> T_m(2,3)
           >> T_m(3,0) >> T_m(3,1) >> T_m(3,2) >> T_m(3,3);

        R = T_m.block<3,3>(0,0);
        t = T_m.block<3,1>(0,3);

        Sophus::SE3 T(R, t);

        pose.push_back(T);
    }
    fin.close();


}

void getPosesIndex(const string& filename, vector<int>& poses_index)
{
    int index;
    string str;
    ifstream fin(filename.c_str());
    if (!fin) {
        cerr << "Trajectory index file can not be found!" << endl;
        return;
    }

    while (!fin.eof()) {
        getline(fin, str);
        istringstream ss(str);
        ss >> index;

        poses_index.push_back(index);
    }
    fin.close();
}

void ICP_solve(const vector_poses& p_e, const vector_poses& p_g, Eigen::Matrix3d& R, Eigen::Vector3d& t){
    // center mass
    Eigen::Vector3d center_e = Eigen::Vector3d::Zero();
    Eigen::Vector3d center_g = Eigen::Vector3d::Zero();
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

    // compute t_g * t_e^T
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


