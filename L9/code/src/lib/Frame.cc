/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "KittiStereo/Frame.h"
#include "KittiStereo/Config.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;

namespace KittiStereo
{

Frame::Frame()
    : id_(-1), timeStamp_(-1), isKeyFrame_(false), Tcw_(SE3()), camera_(nullptr)
{}

Frame::Frame(long id, double time_stamp, SE3 T_c_w, Camera::Ptr camera, Mat imgLeft, Mat imgRight)
    : id_(id), timeStamp_(time_stamp), isKeyFrame_(false), imgLeft_(imgLeft),
      imgRight_(imgRight), Tcw_(T_c_w), camera_(camera)
{}

Frame::~Frame()
{}

Frame::Ptr Frame::createFrame()
{
    static long factory_id = 0;

    return Frame::Ptr(new Frame(factory_id++));
}

Vector3d Frame::getCameraCenter() const
{
    return Tcw_.inverse().translation();
}

void Frame::setParameters(const Camera::Ptr camera, const cv::Mat img_left,
                          const cv::Mat img_right, const double img_time)
{
    camera_         = camera;
    imgLeft_        = img_left;
    imgRight_       = img_right;
    timeStamp_      = img_time;
    numFeatures_    = KittiStereo::Config::get<int>("ORB.nFeatures");
    pyramidLevels_  = KittiStereo::Config::get<int>("ORB.pyramidLevels_");
    scaleFactor_    = KittiStereo::Config::get<float>("ORB.scaleFactor");
    matchRatio_     = KittiStereo::Config::get<float>("match_ratio");
}

void Frame::setPose(const SE3& T_c_w)
{
    Tcw_ = T_c_w;
}

bool Frame::isInFrame(const Vector3d& pt_world)
{
    Vector3d p_cam = camera_->world2camera(pt_world, Tcw_);
    if (p_cam(2,0) < 0 ) { return false; }

    Vector2d pixel = camera_->world2pixel(pt_world, Tcw_);
    return pixel(0,0) > 0 && pixel(1,0) > 0
           && pixel(0,0) < imgLeft_.cols
           && pixel(1,0) < imgLeft_.rows;
}

void Frame::computeStereoMatches()
{
    if (imgLeft_.empty() || imgRight_.empty())
        return;

    // 计算左右图像的ORB特征点并匹配
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create(numFeatures_, scaleFactor_, pyramidLevels_);
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create(numFeatures_, scaleFactor_, pyramidLevels_);
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

    detector->detect(imgLeft_, kpsLeft_);
    detector->detect(imgRight_, kpsRight_);
    descriptor->compute(imgLeft_, kpsLeft_, descLeft_);
    descriptor->compute(imgRight_, kpsRight_, descRight_);
    std::vector<cv::DMatch> matches_1st, matches_2nd;
    matcher->match(descLeft_, descRight_, matches_1st);

    // 找到正确的匹配
    double min_dist = 10000;
    // 找出所有匹配之间的最小距离和最大距离
    // 即是最相似的和最不相似的两组点之间的距离
    for (auto& m : matches_1st ) {
        double dist = m.distance;
        if (dist < min_dist) { min_dist = dist; }
    }

    // 当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.
    // 但有时候最小距离会非常小,设置一个经验值30作为下限.
    for (auto& m : matches_1st ) {
        if ( m.distance <= std::max(matchRatio_ * min_dist, 20.0) ) {
            matches_2nd.push_back(m);
        }
    }

    // RANSAC 再次消除误匹配
    vector<KeyPoint> kpsLeft_2nd, kpsRight_2nd;
    vector<Point2f> kpl, kpr;
    Mat descLeft_2nd;
    for (size_t i=0; i<matches_2nd.size(); i++) {
       // 经过此步 kpsLeft_2nd 和 kpsRight_2nd 在顺序上有一一匹配关系了
       kpsLeft_2nd.push_back(kpsLeft_[matches_2nd[i].queryIdx]);
       kpsRight_2nd.push_back(kpsRight_[matches_2nd[i].trainIdx]);
       kpl.push_back(kpsLeft_2nd[i].pt);
       kpr.push_back(kpsRight_2nd[i].pt);
       descLeft_2nd.push_back(descLeft_.row(matches_2nd[i].queryIdx).clone());
    }

    vector<uchar> RansacStatus;
    Mat H = findHomography(kpl, kpr, CV_RANSAC, 3, RansacStatus);

    vector<KeyPoint> aft_ran_kpsl, aft_ran_kpsr;
    Mat aft_ran_descl;
    int index = 0;
    for (size_t i=0; i<matches_2nd.size(); i++) {
       if (RansacStatus[i] != 0) {
           aft_ran_kpsl.push_back(kpsLeft_2nd[i]);
           aft_ran_kpsr.push_back(kpsRight_2nd[i]);
           matches_2nd[i].queryIdx = index;
           matches_2nd[i].trainIdx = index;
           matches_.push_back(matches_2nd[i]);  // matche index based on aft_ran_kps
           aft_ran_descl.push_back(descLeft_2nd.row(i).clone());
           index++;
       }
    }

    // update kps and description
    kpsLeft_.swap(aft_ran_kpsl);
    kpsRight_.swap(aft_ran_kpsr);
    descLeft_.resize(aft_ran_descl.rows, aft_ran_descl.cols);
    aft_ran_descl.copyTo(descLeft_);

    // show

    cv::Mat img1_show, img2_show;
    cv::cvtColor(imgLeft_, img1_show, CV_GRAY2BGR);
    cv::cvtColor(imgRight_, img2_show, CV_GRAY2BGR);
    cv::Mat img_match_show(2*img1_show.rows, img1_show.cols, CV_8UC3);
    img1_show.copyTo(img_match_show(cv::Rect(0, 0, img1_show.cols, img1_show.rows)));
    img2_show.copyTo(img_match_show(cv::Rect(0, img1_show.rows, img2_show.cols, img2_show.rows)));

    for (auto &m : matches_) {
        float b = 255*float ( rand() ) /RAND_MAX;
        float g = 255*float ( rand() ) /RAND_MAX;
        float r = 255*float ( rand() ) /RAND_MAX;
        cv::circle(img_match_show, cv::Point2d(kpsLeft_[m.queryIdx].pt.x, kpsLeft_[m.queryIdx].pt.y), 3, cv::Scalar(b,g,r), 2);
        cv::circle(img_match_show, cv::Point2d(kpsRight_[m.trainIdx].pt.x, kpsRight_[m.trainIdx].pt.y+imgLeft_.rows), 3, cv::Scalar(b,g,r), 2);
        cv::line(img_match_show, cv::Point2d(kpsLeft_[m.queryIdx].pt.x, kpsLeft_[m.queryIdx].pt.y), cv::Point2d(kpsRight_[m.trainIdx].pt.x, kpsRight_[m.trainIdx].pt.y+imgLeft_.rows), cv::Scalar(b,g,r), 1);

    }
    cv::imshow("Matches between left & right images", img_match_show);
    cv::waitKey(1);

}

void Frame::computeDepth()
{
    // 根据左右两幅图像的特征点匹配情况，去掉没有深度值的特征点
    std::vector<cv::KeyPoint> kps_tmp;
    Mat desc_tmp;
    for (auto& m : matches_) {
        double disparity = kpsLeft_[m.queryIdx].pt.x - kpsRight_[m.trainIdx].pt.x;
        if (disparity < 0.01) { continue; }

        double z = camera_->bf_ / disparity;
        if (z > 300) { continue; }

        kps_tmp.push_back(kpsLeft_[m.queryIdx]);
        desc_tmp.push_back(descLeft_.row(m.queryIdx).clone());
        kpsDepth_.push_back(z);
    }

    // 更新本帧的特征点及其描述子成员变量
    kpsLeft_.swap(kps_tmp);
    descLeft_ = desc_tmp.clone();

    cout << "[Frame] extracted keypoints: " << kpsLeft_.size() << endl;
}

} // namespace
