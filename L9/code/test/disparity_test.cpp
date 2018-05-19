#include <iostream>
#include <unistd.h>
#include <string>
#include <unordered_map>

#include <Eigen/Core>
#include <pangolin/pangolin.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
//#include <opencv2/imgproc.hpp>

using namespace std;
using namespace Eigen;
using namespace cv;

void showPointCloud(const vector<Vector4d, Eigen::aligned_allocator<Vector4d>> &pointcloud) {

    if (pointcloud.empty()) {
        cerr << "Point cloud is empty!" << endl;
        return;
    }

    pangolin::CreateWindowAndBind("Point Cloud Viewer", 1024, 768);
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

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glPointSize(2);
        glBegin(GL_POINTS);
        for (auto &p: pointcloud) {
            glColor3f(p[3], p[3], p[3]);
            glVertex3d(p[0], p[1], p[2]);
        }
        glEnd();
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
    return;
}

void WriteToPCDFile(const std::string& filename, const vector<Vector4d, Eigen::aligned_allocator<Vector4d>> &pointcloud){
    std::ofstream of(filename.c_str());

    of << "VERSION .7"
       << '\n' << "FIELDS x y z rgb"
       << '\n' << "SIZE 4 4 4 4"
       << '\n' << "TYPE F F F F"
       << '\n' << "COUNT 1 1 1 1"
       << '\n' << "WIDTH " << pointcloud.size()
       << '\n' << "HEIGHT 1"
       << '\n' << "VIEWPOINT 0 0 0 1 0 0 0"
       << '\n' << "POINTS " << pointcloud.size()
       << '\n' << "DATA ascii" << std::endl;

    for(int i = 0; i < pointcloud.size(); ++i){
      of << pointcloud[i][0] << ' ' << pointcloud[i][1] << ' ' << pointcloud[i][2] << ' ' << pointcloud[i][3] << '\n';
    }

    of.close();
}

void orb_extra(Mat& imgLeft, Mat& imgRight)
{
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

    vector<KeyPoint> kpsLeft, kpsRight;
    Mat descLeft, descRight;

    detector->detect(imgLeft, kpsLeft);
    detector->detect(imgRight, kpsRight);
    descriptor->compute(imgLeft, kpsLeft, descLeft);
    descriptor->compute(imgRight, kpsRight, descRight);
    std::vector<DMatch> matches, good_matches;
    matcher->match(descLeft, descRight, matches);

    // find good matches
    double min_dist = 10000, max_dist = 0;
    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for (auto i = 0; i < descLeft.rows; i++ ) {
        double dist = matches[i].distance;
        if ( dist < min_dist )
            min_dist = dist;
        if ( dist > max_dist )
            max_dist = dist;
    }

    printf ( "[Frame]: Max dist : %f \n", max_dist );
    printf ( "[Frame]: Min dist : %f \n", min_dist );

    for (auto i=0; i<descLeft.rows; i++ ) {
        if ( matches[i].distance <= max(2*min_dist, 30.0) ) {
            good_matches.push_back(matches[i]);
        }
    }

    cv::Mat img_match, img_goodmatch;
    cv::drawMatches ( imgLeft, kpsLeft, imgRight, kpsRight, matches, img_match );
    cv::drawMatches ( imgLeft, kpsLeft, imgRight, kpsRight, good_matches, img_goodmatch );
    cv::imshow ( "所有匹配点对", img_match );
    cv::imshow ( "优化后匹配点对", img_goodmatch );
    cv::waitKey(0);
}


int main(int argc, char *argv[])
{
    // image disparity
    cv::Mat imgLeft = cv::imread("../left.png", 0);
    cv::Mat imgRight = cv::imread("../right.png", 0);

    orb_extra(imgLeft, imgRight);

    cv::Mat disparity;

    cv::Ptr<cv::StereoBM> bm = cv::StereoBM::create();
    bm->setPreFilterType(CV_STEREO_BM_XSOBEL);
    bm->setPreFilterSize(9);
    bm->setPreFilterCap(31);
    bm->setBlockSize(15);
    bm->setMinDisparity(0);
    bm->setNumDisparities(32);
    bm->setTextureThreshold(10);
    bm->setUniquenessRatio(15);
    bm->setSpeckleWindowSize(100);
    bm->setSpeckleRange(32);
    bm->compute(imgLeft, imgRight, disparity);
    // 新版本下的BM和SGBM方法计算出的视差都是CV_16S格式的
    // 使用32位float格式可以得到真实的视差值，所以我们需要除以16
    disparity.convertTo(disparity, CV_32F, 1.0 / 16);

    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create();
    sgbm->setSpeckleWindowSize(100);
    sgbm->setSpeckleRange(32);
    sgbm->setDisp12MaxDiff(1);
    sgbm->setBlockSize(15);
    sgbm->setMinDisparity(0);
    sgbm->setP1(4*11*11);
    sgbm->setP2(32*11*11);
    sgbm->setNumDisparities(32);
    sgbm->setUniquenessRatio(10);
    sgbm->compute(imgLeft, imgRight, disparity);
    cv::imwrite("disparity.png", disparity);
    std::cout << "done." << std::endl;

    // 生成点云
    float bf = 186.1448;
    float fx = 718.856;
    float fy = 718.856;
    float cx = 607.1928;
    float cy = 185.2157;
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> pointcloud;
    for (int v = 0; v < imgLeft.rows; v++) {
        for (int u = 0; u < imgLeft.cols; u++) {
            Eigen::Vector4d point(0, 0, 0, imgLeft.at<uchar>(v, u) / 255.0); // 前三维为xyz,第四维为颜色

            // 根据双目模型计算 point 的位置
            if (disparity.at<uchar>(v, u) == 0) { continue; }
            point[2] = bf / disparity.at<uchar>(v, u);
            point[0] = (u - cx) * point[2] / fx;
            point[1] = (v - cy) * point[2] / fy;

            pointcloud.push_back(point);
            // end your code here
        }
    }

    // 画出点云
    showPointCloud(pointcloud);
    WriteToPCDFile("aa.pcd", pointcloud);

    return 0;
}
