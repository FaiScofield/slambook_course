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

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "KittiStereo/common_include.h"

namespace KittiStereo
{

class Frame;

class MapPoint
{
public:
    typedef std::shared_ptr<MapPoint> Ptr;

    static unsigned long factoryId_;    // factory id
    unsigned long   id_;                // 该点属于的帧数
    bool            good_;              // 是否是一个好的地图点
    int             visibleTimes_;      // 被后续帧看到的次数
    int             matchedTimes_;      // 被后续帧匹配到的次数
    Eigen::Vector3d pos_;               // 世界坐标
    Eigen::Vector3d norm_;              // 观测方向法线
    cv::Mat         descriptor_;        // 该点的描述子

    std::list<Frame*> observedFrames_;  // 观测到此点的关键帧列表

public:
    MapPoint();

    MapPoint(unsigned long id, const Vector3d& position, const Vector3d& norm,
             Frame* frame=nullptr, const Mat& descriptor=Mat() );

    // 获取改点的位姿，以cv::Point3f存储
    inline cv::Point3f getPositionCV() const {
        return cv::Point3f(pos_(0,0), pos_(1,0), pos_(2,0));
    }

    // 创建地图点
    static MapPoint::Ptr createMapPoint();
    static MapPoint::Ptr createMapPoint(const Vector3d& pos_world,
                                        const Vector3d& norm_,
                                        const cv::Mat& descriptor,
                                        Frame* frame );
};

} // namespace

#endif // MAPPOINT_H
