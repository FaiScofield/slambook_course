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

#ifndef CAMERA_H
#define CAMERA_H

#include "KittiStereo/common_include.h"

namespace KittiStereo
{

// 针孔相机模型
class Camera
{
public:
    typedef std::shared_ptr<Camera> Ptr;

    float fx_, fy_;         // 相机内参
    float cx_, cy_;
    float k1_, k2_;         // 相机畸变参数
    float p1_, p2_;
    float fps_;             // 相机每秒帧数
    float bf_;              // 相机基线与fx的乘积，bf=b*f
    int width_, height_;    // 图像的尺寸

public:
    Camera();

    Camera(float fx, float fy, float cx, float cy,
           float k1=0, float k2=0, float p1=0, float p2=0)
        : fx_(fx), fy_(fy), cx_(cx), cy_(cy),
          k1_(k1), k2_(k2), p1_(p1), p2_(p2)
    {}

    ~Camera();

    /**
     * @brief 坐标转换函数
     * @param   略
     * @return  对应坐标系下的坐标
     */
    Vector3d world2camera( const Vector3d& p_w, const SE3& T_c_w );
    Vector3d camera2world( const Vector3d& p_c, const SE3& T_c_w );
    Vector2d camera2pixel( const Vector3d& p_c );
    Vector3d pixel2camera( const Vector2d& p_p, double depth=1 );
    Vector3d pixel2world ( const Vector2d& p_p, const SE3& T_c_w, double depth=1 );
    Vector2d world2pixel ( const Vector3d& p_w, const SE3& T_c_w );
};

} // namespace

#endif // CAMERA_H
