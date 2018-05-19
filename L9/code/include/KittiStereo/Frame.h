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

#ifndef FRAME_H
#define FRAME_H

#include "KittiStereo/common_include.h"
#include "KittiStereo/Camera.h"

namespace KittiStereo
{

// 前向声明
class MapPoint;

class Frame
{
public:
    // 定义指针
    typedef std::shared_ptr<Frame> Ptr;

    unsigned long   id_;                    // 该帧的id
    double          timeStamp_;             // 该帧的时间戳
    bool            isKeyFrame_;            // 是否为关键帧
    cv::Mat         imgLeft_, imgRight_;    // 该帧的左右图像
    cv::Mat         descLeft_, descRight_;  // 左右图像特征点的描述子
    Sophus::SE3     Tcw_;                   // 世界坐标系到相机坐标系的变换
    Camera::Ptr     camera_;                // Camere类做成员变量
    std::vector<cv::KeyPoint>   kpsLeft_;   // 左图像的特征点
    std::vector<cv::KeyPoint>   kpsRight_;  // 右图像的特征点
    std::vector<double>         kpsDepth_;  // 特征点的深度值
    std::vector<cv::DMatch>     matches_;   // 左右图之间的匹配情况
//    std::vector<MapPoint*>      mapPoints_; // 该帧生成的MapPoint

    // ORB 参数
    int             numFeatures_;           // 待提取的特征点总数
    int             pyramidLevels_;         // 高斯金字塔层数
    float           scaleFactor_;           // 金字塔图像之间的尺度
    float           matchRatio_;            // 特征点描述子间最小距离的匹配比率

public:
    // 构造函数
    Frame();

    // 拷贝构造函数
    Frame(long id, double time_stamp=0, SE3 T_c_w=SE3(),
          Camera::Ptr camera=nullptr, Mat color=Mat(), Mat depth=Mat());

    // 析构函数
    ~Frame();

    // 成员函数，创建一个Frame
    static Frame::Ptr createFrame();

    // 成员函数，获取相机的光心位置
    Vector3d getCameraCenter() const;

    /**
     * @brief 成员函数，设置
     * @param[in] camera    camera指针成员变量
     * @param[in] img_left  该帧的左图像
     * @param[in] img_right 该帧的右图像
     * @param[in] img_time  该帧的记录时间
     */
    void setParameters(const Camera::Ptr camera, const cv::Mat img_left,
                       const cv::Mat img_right, const double img_time);

    /**
     * @brief 成员函数，设置相机的位姿
     * @param[in] T_c_w 李代数表示的位姿
     */
    void setPose(const SE3& T_c_w);

    /**
     * @brief 成员函数，检查某点是在该帧的视野中
     * @param[in] pt_world 点的世界坐标
     * @return  点在该帧的视野中则返回true，反之为false
     */
    bool isInFrame(const Vector3d& pt_world);

    /**
     * @brief 成员函数，匹配左右两幅图的特征点
     * 该函数会先计算左右两幅图的orb特征点和描述子
     * 再对左右两幅图的特征点进行匹配，剔除坏匹配
     */
    void computeStereoMatches();

    /**
     * @brief 成员函数，计算特征点的深度值
     *
     * 该函数会根据左右两幅的匹配情况计算特征点深度值
     * 再该帧的特征点进行更新，只保留好的匹配点做特征点
     * 即只保留有深度值的特征点
     */
    void computeDepth();

};

} // namespace

#endif // FRAME_H
