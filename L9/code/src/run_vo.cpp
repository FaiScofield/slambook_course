// -------------- test the visual odometry -------------
#include <fstream>
#include <iomanip>  // for stw() & setfill()
#include <boost/timer.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "KittiStereo/Config.h"
#include "KittiStereo/VisualOdometry.h"

using namespace std;

void loadImages(const string &file_path, vector<string> &left_imgs,
                vector<string> &right_imgs, vector<double> &img_times)
{
    cout << "[Main] Loading the data...";
    // read time data
    string strPathTimeFile = file_path + "/times.txt";
    ifstream fTimes;
    fTimes.open(strPathTimeFile.c_str());
    if (!fTimes.is_open()) {
        cerr << "[Main] Time file: " << strPathTimeFile.c_str() << " does not exist! " << endl;
        return;
    }
    while(!fTimes.eof()) {
        string s;
        getline(fTimes, s);
        if(!s.empty()) {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            img_times.push_back(t);
        }
    }

    // read image data
    string strPrefixLeft = file_path + "/image_0/";
    string strPrefixRight = file_path + "/image_1/";
    const int nTimes = img_times.size();
    left_imgs.resize(nTimes);
    right_imgs.resize(nTimes);

    for(int i=0; i<nTimes; i++) {
        stringstream ss;
        ss << std::setfill('0') << setw(6) << i;
        left_imgs[i] = strPrefixLeft + ss.str() + ".png";
        right_imgs[i] = strPrefixRight + ss.str() + ".png";
    }
    cout << "[main] Data loaded." << endl;
}

void writeKeyFrameIndex(const string &file_name, KittiStereo::Frame::Ptr frame)
{
    ofstream f;
    f.open(file_name, ios_base::app);
    f << fixed;
    f << frame->id_ << endl;
    f.close();
}

void writeKeyFrameTrajectory(const string &file_name, KittiStereo::Frame::Ptr frame)
{
    SE3 Twc = frame->Tcw_.inverse();
    Eigen::Matrix4d Tcw_m = Twc.matrix().inverse();
    cv::Mat Tcw = (cv::Mat_<double>(4,4) << Tcw_m(0,0), Tcw_m(0,1), Tcw_m(0,2), Tcw_m(0,3),
                   Tcw_m(1,0), Tcw_m(1,1), Tcw_m(1,2), Tcw_m(1,3),
                   Tcw_m(2,0), Tcw_m(2,1), Tcw_m(2,2), Tcw_m(2,3),
                   Tcw_m(3,0), Tcw_m(3,1), Tcw_m(3,2), Tcw_m(3,3));

    ofstream f;
    f.open(file_name, ios_base::app);
    f << fixed;
    f << setiosflags(ios::fixed)<< setprecision(6)
      << Tcw.at<double>(0,0) << " " << Tcw.at<double>(0,1)  << " " << Tcw.at<double>(0,2) << " "
      << Tcw.at<double>(0,3) << " "
      << Tcw.at<double>(1,0) << " " << Tcw.at<double>(1,1)  << " " << Tcw.at<double>(1,2) << " "
      << Tcw.at<double>(1,3) << " "
      << Tcw.at<double>(2,0) << " " << Tcw.at<double>(2,1)  << " " << Tcw.at<double>(2,2) << " "
      << Tcw.at<double>(2,3) << endl;
    f.close();
}


int main ( int argc, char** argv )
{
    // check the usage
    if ( argc != 2 ) {
        cerr << "[Main] Usage: run_vo parameter_file" << endl;
        return -1;
    }

    // load the data
    KittiStereo::Config::setParameterFile(argv[1]);
    string dataset_dir = KittiStereo::Config::get<string>("dataset_dir");
    cout << "[Main] Dataset path: " << dataset_dir << endl;
    vector<string> left_img_files, right_img_files;
    vector<double> img_times;
    loadImages(dataset_dir, left_img_files, right_img_files, img_times);

    // 开启VO
    KittiStereo::VisualOdometry::Ptr vo(new KittiStereo::VisualOdometry);
    KittiStereo::Camera::Ptr camera(new KittiStereo::Camera);
    cout << "[Main] Read total " << left_img_files.size() << " entries" << endl;

    ofstream f, fk;
    f.open("../my_kitti_keyframe_index.txt");
    f << fixed;
    f.close();
    fk.open("../my_kitti_keyframe_trajectory.txt");
    fk << fixed;
    fk.close();

    // 对每一帧图像进行处理
    for (uint i=0; i<left_img_files.size(); i++ )  {
        cout << "[Main] ****** loop " << i << " ******" << endl;
        string left_img_file = left_img_files.at(i);
        string right_img_file = right_img_files.at(i);
        Mat img_left = cv::imread(left_img_file, -1);
        Mat img_right = cv::imread(right_img_file, -1);

        if ( img_left.empty() || img_right.empty() ) {
            cerr << "[Main] Failed to load image at: " << i << endl;
            break;
        }

        // create frame
        KittiStereo::Frame::Ptr pFrame = KittiStereo::Frame::createFrame();
        pFrame->setParameters(camera, img_left, img_right, img_times[i]);
        pFrame->computeStereoMatches();
        pFrame->computeDepth();

        bool ok;
        boost::timer timer;
        ok = vo->addFrame(pFrame);
//        cout << "[Main] VO costs time: " << timer.elapsed() << endl;
        // write the camera pose
        if (ok && vo->curr_->isKeyFrame_) {
            writeKeyFrameIndex("../my_kitti_keyframe_index.txt", vo->curr_);
            writeKeyFrameTrajectory("../my_kitti_keyframe_trajectory.txt", vo->curr_);
        }

        // stop when vo state is LOST
        if ( vo->state_ == KittiStereo::VisualOdometry::LOST ) {
            cerr << "[Main] VO state: LOST." << endl;
            break;
        }

        // show image and make the keypoint mark colored
        Mat img_show = img_left.clone();
        cv::cvtColor(img_show, img_show, CV_GRAY2RGB);
        for ( auto& pt : vo->map_->map_points_ )  {
            KittiStereo::MapPoint::Ptr p = pt.second;
            Vector2d pixel = pFrame->camera_->world2pixel(p->pos_, pFrame->Tcw_);
            cv::circle(img_show, cv::Point2f(pixel(0,0), pixel(1,0)), 3, cv::Scalar(0,255,0), 1);
        }
//        for ( auto& kpt : pFrame->kpsLeft_ )  {
//            cv::circle(img_show, cv::Point2f(kpt.pt.x, kpt.pt.y), 3, cv::Scalar(0,255,0), 1);
//        }
        stringstream text;
        text << "Image " << i << ", MapPoints: " << vo->map_->map_points_.size();
        cv::putText(img_show, text.str(), cv::Point(img_left.cols/2-150,img_left.rows-20), CV_FRONT, 1.0, CV_RGB(255,0,0));
        cv::imshow("MapPoints on current frame", img_show);
        cv::waitKey(100);

        cout << endl;
    }

    cout << "[Main] VO done. Please run the 'draw_trajectory' program to check the result." << endl;
    return 0;
}
