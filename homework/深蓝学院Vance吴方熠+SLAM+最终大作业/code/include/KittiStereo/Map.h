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

#ifndef MAP_H
#define MAP_H

#include "KittiStereo/common_include.h"
#include "KittiStereo/Frame.h"
#include "KittiStereo/MapPoint.h"

namespace KittiStereo
{

class Map
{
public:
    typedef std::shared_ptr<Map> Ptr;

    std::unordered_map<unsigned long, MapPoint::Ptr>  map_points_;   // 所有的路标点
    std::unordered_map<unsigned long, Frame::Ptr>     keyframes_;    // 所有的关键帧

public:
    Map() {}

    // 参入关键帧
    void insertKeyFrame(Frame::Ptr frame);

    // 参入MapPoint
    void insertMapPoint(MapPoint::Ptr map_point);
};

} // namespace

#endif // MAP_H
