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

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <algorithm>
#include <boost/timer.hpp>

#include "KittiStereo/Config.h"
#include "KittiStereo/VisualOdometry.h"
#include "KittiStereo/g2o_types.h"


namespace KittiStereo
{

VisualOdometry::VisualOdometry() :
    numInliers_(0), numLost_(0), state_(INITIALIZING), map_(new Map),
    ref_(nullptr), curr_(nullptr), matcherFlann_(new cv::flann::LshIndexParams(5,10,2))
{
    matchRatio_         = Config::get<float>("match_ratio");
    maxNumlost_         = Config::get<int>("max_num_lost");
    minInliers_         = Config::get<int>("min_inliers");
    minRatio_           = Config::get<float>("min_ratio");
    keyFrameMinRot_     = Config::get<double>("keyframe_rotation");
    keyFrameMinTrans_   = Config::get<double>("keyframe_translation");
    mapPointEraseRatio_ = Config::get<double>("map_point_erase_ratio");
}

VisualOdometry::~VisualOdometry()
{}

bool VisualOdometry::addFrame(Frame::Ptr frame)
{
    if(frame->kpsLeft_.size() != frame->kpsDepth_.size() ||
       frame->kpsLeft_.size() != (unsigned)frame->descLeft_.rows ) {
        cerr << "[VO] Wrong size of kpsLeft_ or kpsDepth_ or descLeft_.rows!!" << endl;
    }

    switch (state_)  {
        case INITIALIZING: {
            state_ = OK;
            curr_ = ref_ = frame;
            keypointsCurr_ = curr_->kpsLeft_;
            descriptorsCurr_ = curr_->descLeft_;
            depthCurr_ = curr_->kpsDepth_;

            addKeyFrame();  // 第一帧总为关键帧
            break;
          }
        case OK: {
            curr_ = frame;
            curr_->Tcw_ = ref_->Tcw_;
            keypointsCurr_ = curr_->kpsLeft_;
            descriptorsCurr_ = curr_->descLeft_;
            depthCurr_ = curr_->kpsDepth_;

            featureMatching();
            poseEstimationPnP();
//            poseEstimationICP();
            if ( checkEstimatedPose() == true ) {  // 位姿估计良好的情况
                curr_->Tcw_ = Tcw_estimated_;
                optimizeMap();
                numLost_ = 0;
                if ( checkKeyFrame() == true ) {  // 判断其是否可以为关键帧
                    addKeyFrame();
                }
            } else {  // 多种原因造成位姿误差太大的情况
                numLost_++;
                if ( numLost_ > maxNumlost_ ) {
                    cerr << "[VO] 'num_lost' has reach the 'max_num_lost' threthod." << endl;
                    state_ = LOST;
                }
                curr_->isKeyFrame_ = false;
                cout << "[VO] this is not a keyframe, skip." << endl;
                return false;
            }
            break;
        }
        case LOST: {
            cout << "[VO] vo has lost." << endl;
            break;
        }
    } // switch

    return true;
}

// 对当前帧特征点与地图点进行匹配
void VisualOdometry::featureMatching()
{
    boost::timer timer;
    vector<cv::DMatch> matches;

    // 在地图中选取候选点
    cv::Mat desp_map;
    vector<MapPoint::Ptr> candidate;    // 候选点
    cout << "[VO] number of map_points: " <<  map_->map_points_.size() << endl;
    for ( auto& allpoints : map_->map_points_ ) {
        MapPoint::Ptr& p = allpoints.second; // first为键值， second为地图点
        // 判断该地图点是否在当前帧的视野中
        if ( curr_->isInFrame(p->pos_) ) {
            // 如果在视野中则加入候选点的容器中
            p->visibleTimes_++;
            candidate.push_back(p);
            desp_map.push_back(p->descriptor_);
        }
    }
    matcherFlann_.match(desp_map, descriptorsCurr_, matches);

    // 选择最好的匹配点
    float min_dis = std::min_element(matches.begin(), matches.end(),
                        [](const cv::DMatch& m1, const cv::DMatch& m2)
                        { return m1.distance < m2.distance; } )->distance;

    match_3dpts_.clear();
    match_2dkp_index_.clear();
    for ( cv::DMatch& m : matches ) {
        if ( m.distance < std::max<float>(min_dis * matchRatio_, 20.0) )  {
            candidate[m.queryIdx]->good_ = true;
            match_3dpts_.push_back(candidate[m.queryIdx]);
            match_2dkp_index_.push_back(m.trainIdx);
        }
    }

    cout << "[VO] good matches: " << match_3dpts_.size() << endl;
    cout << "[VO] match cost time: " << timer.elapsed() << endl;
}

// 位姿优化
void VisualOdometry::poseEstimationPnP()
{
    if (match_3dpts_.size() < 4 ) {
        cerr << "[VO] good matches size is less then 4, cannot estimate PnP." << endl;
        return;
    }
    // 构造3纬、2纬观测
    vector<cv::Point3f> pts3d;
    vector<cv::Point2f> pts2d;

    for (int index : match_2dkp_index_) {
        pts2d.push_back(keypointsCurr_[index].pt);
    }
    for (MapPoint::Ptr pt : match_3dpts_) {
        pts3d.push_back(pt->getPositionCV());
    }

    Mat K = ( cv::Mat_<double>(3,3) <<
              ref_->camera_->fx_, 0, ref_->camera_->cx_,
              0, ref_->camera_->fy_, ref_->camera_->cy_,
              0, 0, 1);

    Mat rvec, tvec, inliers;
    cv::solvePnPRansac(pts3d, pts2d, K, Mat(), rvec, tvec,
                       false,   // useExtrinsicGuess
                       100,     // 迭代次数
                       4.0,     // 重投影误差
                       0.99,    // 置信度
                       inliers );
    numInliers_ = inliers.rows;
    cout << "[VO] pnp inliers: " << numInliers_ << endl;

    Tcw_estimated_ = SE3(SO3(rvec.at<double>(0,0), rvec.at<double>(1,0), rvec.at<double> (2,0)),
                         Vector3d(tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0)) );

    // 用BA优化位姿
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,2>> Block;
    std::unique_ptr<Block::LinearSolverType> linearSolver(new g2o::LinearSolverDense<Block::PoseMatrixType>());
    std::unique_ptr<Block> solver_ptr( new Block(std::move(linearSolver)) );
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( std::move(solver_ptr));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
    pose->setId(0);
    pose->setEstimate(g2o::SE3Quat(Tcw_estimated_.rotation_matrix(), Tcw_estimated_.translation()) );
    optimizer.addVertex(pose);

    // 边
    for ( int i=0; i<inliers.rows; i++ ) {
        int index = inliers.at<int>(i, 0);
        // 三维到二维的投影
        EdgeProjectXYZ2UVPoseOnly* edge = new EdgeProjectXYZ2UVPoseOnly();
        edge->setId(i);
        edge->setVertex(0, pose);
        edge->camera_ = curr_->camera_.get();
        edge->point_ = Vector3d(pts3d[index].x, pts3d[index].y, pts3d[index].z);
        edge->setMeasurement(Vector2d(pts2d[index].x, pts2d[index].y) );
        edge->setInformation(Eigen::Matrix2d::Identity());
        optimizer.addEdge(edge);
        // 设置局内点
        match_3dpts_[index]->matchedTimes_++;
    }

    optimizer.initializeOptimization();
    optimizer.optimize(10);

    Tcw_estimated_ = SE3(pose->estimate().rotation(),
                         pose->estimate().translation());

    cout << "[VO] Tcw_estimated_ by PnP: " << endl << Tcw_estimated_.matrix() << endl;

}

void VisualOdometry::poseEstimationICP()
{
    vector<Vector3d> pts_map;
    vector<Vector3d> pts_curr;

    for (int index : match_2dkp_index_) {
        Vector3d pcurr = Vector3d(keypointsCurr_[index].pt.x,
                                  keypointsCurr_[index].pt.y,
                                  depthCurr_[index]);
        pts_curr.push_back(pcurr);
    }
    for (MapPoint::Ptr pt : match_3dpts_) {
        cv::Point3f pmap = pt->getPositionCV();
        pts_map.push_back(Vector3d(pmap.x, pmap.y, pmap.z));
    }

    // ICP
    // 质心
    Vector3d p1, p2;
    uint N = pts_map.size();
    for (uint i=0; i<N; i++) {
        p1 += pts_map[i];
        p2 += pts_curr[i];
    }
    p1 /= N;
    p2 /= N;

    // 去质心坐标
    vector<Vector3d>  q1(N), q2(N);
    for (uint i=0; i<N; i++) {
        q1[i] = pts_map[i] - p1;
        q2[i] = pts_curr[i] - p2;
    }

    // 计算q1*q2^T
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for ( uint i=0; i<N; i++ ) {
        W += Eigen::Vector3d(q1[i][0], q1[i][1], q1[i][2]) * Eigen::Vector3d(q2[i][0], q2[i][1], q2[i][2]).transpose();
    }

    // 用SVD解W
    Eigen::JacobiSVD<Eigen::Matrix3d> svd( W, Eigen::ComputeFullU | Eigen::ComputeFullV );
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    Eigen::Matrix3d R_ = U * V.transpose();
    Eigen::Vector3d t_ = Eigen::Vector3d(p1[0], p1[1], p1[2]) - R_ * Eigen::Vector3d(p2[0], p2[1], p2[2]);
    Tcw_estimated_ = SE3(SO3(R_), t_);

    // BA优化位姿
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,2>> Block;
    std::unique_ptr<Block::LinearSolverType> linearSolver(new g2o::LinearSolverDense<Block::PoseMatrixType>());
    std::unique_ptr<Block> solver_ptr( new Block(std::move(linearSolver)) );
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( std::move(solver_ptr));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
    pose->setId(0);
    pose->setEstimate(g2o::SE3Quat(Tcw_estimated_.rotation_matrix(), Tcw_estimated_.translation()) );
    optimizer.addVertex(pose);

    // 边
    uint index = 1;
    vector<EdgeProjectXYZRGBDPoseOnly*> edges;
    for ( uint i=0; i<N; i++ ) {
        EdgeProjectXYZRGBDPoseOnly* edge = new EdgeProjectXYZRGBDPoseOnly(pts_curr[i]);
        edge->setId(index);
        edge->setVertex(0, dynamic_cast<g2o::VertexSE3Expmap*>(pose));
        edge->setMeasurement(pts_map[i]);
        edge->setInformation(Eigen::Matrix3d::Identity()*1e4);
        optimizer.addEdge(edge);
        index++;
        edges.push_back(edge);
    }

    optimizer.initializeOptimization();
    optimizer.optimize(10);

    Tcw_estimated_ = SE3(
        pose->estimate().rotation(),
        pose->estimate().translation()
    );

    cout << "[VO] Tcw_estimated_ by PnP: " << endl << Tcw_estimated_.matrix() << endl;
}

bool VisualOdometry::checkEstimatedPose()
{
    // 检查估计的位姿是否准确
    float ratio = (float)numInliers_ / (float)match_3dpts_.size();
    if ( numInliers_ < minInliers_ && ratio < minRatio_  ) {
        clog << "[VO] reject because inlier is too small: " << numInliers_ << ", or ratio is too low: " << ratio << endl;
        return false;
    }
    // 如果计算出的运动过大，可能计算出错，丢弃该帧
    SE3 Trc = ref_->Tcw_ * Tcw_estimated_.inverse();
    Sophus::Vector6d d = Trc.log();
    if ( d.norm() > 5.0 ) {
        clog << "[VO] reject because motion is too large: " << d.norm() << endl;
        return false;
    }
    return true;
}

// 检查是否为关键帧
bool VisualOdometry::checkKeyFrame()
{
    SE3 Trc = ref_->Tcw_ * Tcw_estimated_.inverse();
    Sophus::Vector6d d = Trc.log();
    Vector3d trans = d.head<3>();   // 前三维平移
    Vector3d rot = d.tail<3>();     // 后三维旋转
    if ( rot.norm() > keyFrameMinRot_ || trans.norm() > keyFrameMinTrans_ )
        return true;

    return false;
}

// 添加关键帧
void VisualOdometry::addKeyFrame()
{
    // 第一帧的所有特征点都记为地图点中
    curr_->isKeyFrame_ = true;

    if ( map_->keyframes_.empty() ) {
        int lost_d = 0;
        for ( size_t i=0; i<curr_->kpsLeft_.size(); i++ ) {
            double d = curr_->kpsDepth_[i];
            if ( d < 0 ) {
                lost_d++;
                continue;
            }

            Vector3d p_world = ref_->camera_->pixel2world (
                Vector2d(curr_->kpsLeft_[i].pt.x, curr_->kpsLeft_[i].pt.y),
                curr_->Tcw_,
                d );
            Vector3d n = p_world - ref_->getCameraCenter();
            n.normalize();
            MapPoint::Ptr map_point = MapPoint::createMapPoint(
                p_world, n, curr_->descLeft_.row(i), curr_.get()
            );
            map_->insertMapPoint(map_point);
        }
        cout << "[VO] inserted " << curr_->kpsLeft_.size()-lost_d << " MapPoints. " << endl;
    }

    // 其他关键帧的处理直接插入即可
    map_->insertKeyFrame(curr_);
    ref_ = curr_;
}

// 向地图中添加新的地图点
void VisualOdometry::addMapPoints()
{
    if (curr_->kpsLeft_.size() != keypointsCurr_.size()) {
        cerr << "[VO] size of curr_->kpsLeft_ != keypoints_curr_.size()" << endl;
        return;
    }
    if (curr_->kpsLeft_.size() == 0) {
        cerr << "[VO] size of curr_->kpsLeft_ is 0." << endl;
        return;
    }

    vector<bool> matched(keypointsCurr_.size(), false);
    for ( auto index : match_2dkp_index_ )
        matched[index] = true;
    for (size_t i=0; i<keypointsCurr_.size(); i++) {
        // 已匹配的点跳过
        if ( matched[i] == true )
            continue;

        // 未匹配的且深度值大于0的点加入到地图点中
        double d = curr_->kpsDepth_[i];
        if ( d < 0 )
            continue;

        Vector3d p_world = curr_->camera_->pixel2world (
            Vector2d ( keypointsCurr_[i].pt.x, keypointsCurr_[i].pt.y ),
            curr_->Tcw_, d
        );
        Vector3d n = p_world - curr_->getCameraCenter();
        n.normalize();

        MapPoint::Ptr map_point = MapPoint::createMapPoint(
            p_world, n, curr_->descLeft_.row(i), curr_.get()
        );
        map_->insertMapPoint(map_point);
    }
}

// 对地图进行优化
void VisualOdometry::optimizeMap()
{
    // 移除很久没有被观测到的地图点
    for ( auto iter = map_->map_points_.begin(); iter != map_->map_points_.end();) {
        if ( !curr_->isInFrame(iter->second->pos_) ) {
            // 注意不要在调用erase()后还访问指针的值，否则会出现溢出的情况
            iter = map_->map_points_.erase(iter);
            continue;
        }

        float match_ratio = float(iter->second->matchedTimes_ / iter->second->visibleTimes_);
        if ( match_ratio < mapPointEraseRatio_ ) {
            iter = map_->map_points_.erase(iter);
            continue;
        }

        /* 目前没有发生过这种情况 */
        if ( iter->second->good_ == false ) {
            // TODO try triangulate this map point
            clog << "[VO] there is a bad mappoint." << endl;
        }

        iter++;
    }

    if ( match_2dkp_index_.size() < 100 ) {
        cout << "[VO] matched points less then 100, adding MapPoits." << endl;
        addMapPoints();
    } else if ( map_->map_points_.size() > 1000 ) {
        cout << "[VO] matched points bigger then 1000, deleting MapPoits." << endl;
        mapPointEraseRatio_ += 0.05; // 提高比值可以剔除更多MapPoint
    } else {
        mapPointEraseRatio_ = 0.1;
    }

    cout << "[VO] number of map_points(optimized): " << map_->map_points_.size() << endl;
}

} // namespace
