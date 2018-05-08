#include <iostream>
#include <fstream>
#include <string>

#include <sophus/se3.h>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <pangolin/pangolin.h>

using namespace std;

// path to trajectory file
string groundtruth_file = "/home/vance/slam_ws/slambook/class/03/L3/code/groundtruth.txt";
string estimated_file = "/home/vance/slam_ws/slambook/class/03/L3/code/estimated.txt";

int main(int argc, char **argv) {

    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3> > gt_poses;
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3> > es_poses;

    // open files
    fstream gt_fin(groundtruth_file.c_str());
    fstream es_fin(estimated_file.c_str());
    if (!gt_fin || !es_fin) {
        cerr << "trajectory file can not be found!" << endl;
        return -1;
    }
    // read the data
    while (!gt_fin.eof() && !es_fin.eof()) {
        double gt_data[8] = {0};
        double es_data[8] = {0};

        for ( auto& d1:gt_data ) gt_fin >> d1;
        for ( auto& d2:es_data ) es_fin >> d2;

        Eigen::Quaterniond q1( gt_data[7], gt_data[4], gt_data[5], gt_data[6]);
        Sophus::SE3 T_gt(q1, Eigen::Vector3d( gt_data[1], gt_data[2], gt_data[3]));
        gt_poses.push_back(T_gt);

        Eigen::Quaterniond q2( es_data[7], es_data[4], es_data[5], es_data[6]);
        Sophus::SE3 T_es(q2, Eigen::Vector3d( gt_data[1], gt_data[2], gt_data[3]));
        es_poses.push_back(T_es);
    }
    // close files
    gt_fin.close();
    es_fin.close();

    // calculate the rmse
    int num = gt_poses.size() - 1;
    double rmse = 0;
    for (int i=0; i<num; i++) {
        Sophus::SE3 T1 = gt_poses[i];
        Sophus::SE3 T2 = es_poses[i];
        Eigen::Matrix<double,6,1> v = (T1.inverse()*T2).log();
        rmse += v.transpose() * v; // v.squaredNorm();
    }
    rmse = sqrt(rmse/num);
    cout << "RMSE:" << rmse << endl;

    return 0;
}


