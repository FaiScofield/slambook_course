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

#ifndef VISUALODOMETRY_H
#define VISUALODOMETRY_H

#include "KittiStereo/common_include.h"
#include "KittiStereo/Map.h"

#include <opencv2/features2d/features2d.hpp>

namespace KittiStereo
{

class VisualOdometry
{
public:
    typedef std::shared_ptr<VisualOdometry> Ptr;

    enum VOState {
        INITIALIZING = -1,
        OK = 0,
        LOST
    };

    int     numInliers_;            // ICP局内点个数
    int     numLost_;               // 丢失帧次数

    float   matchRatio_;            // 挑选 good matches 的比率
    int     maxNumlost_;            // 最大连续丢帧数
    int     minInliers_;            // 最小局内点数
    float   minRatio_;              // 当前局内点与当前局部地图点数的最小比率，低于这个比率的帧跳过
    double  keyFrameMinRot_;        // 两关键帧间的最小旋转值
    double  keyFrameMinTrans_;      // 两关键帧间的最小平移量
    double  mapPointEraseRatio_;    // 地图点剔除率

    VOState     state_;             // 当前帧的VO状态
    Sophus::SE3 Tcw_estimated_;     // 当前帧的估计位姿
    Map::Ptr    map_;               // 包含所有关键帧和地图点的地图
    Frame::Ptr  ref_;               // 参考帧
    Frame::Ptr  curr_;              // 当前帧

    std::vector<cv::KeyPoint>   keypointsCurr_;     // 当前帧的特征点
    cv::Mat                     descriptorsCurr_;   // 当前帧特征点的描述子
    std::vector<double>         depthCurr_;         // 当前帧的特征点depth

    cv::FlannBasedMatcher       matcherFlann_;      // flann 匹配对象
    std::vector<MapPoint::Ptr>  match_3dpts_;       // 匹配到的地图点
    std::vector<int>            match_2dkp_index_;  // 匹配到的特征点 keypointsCurr_ 像素索引

public:
    VisualOdometry();
    ~VisualOdometry();

    // 创建新的帧
    bool addFrame(Frame::Ptr frame);

protected:
    // 内部函数
    void extractKeyPoints();
    void computeDescriptors();
    void featureMatching();
    void poseEstimationPnP();
    void poseEstimationICP();
    void optimizeMap();

    void addKeyFrame();
    void addMapPoints();
    bool checkEstimatedPose();
    bool checkKeyFrame();
};

} // namespace

#endif // VISUALODOMETRY_H
