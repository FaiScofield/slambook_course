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

#include "KittiStereo/common_include.h"
#include "KittiStereo/MapPoint.h"

namespace KittiStereo
{

MapPoint::MapPoint()
    : id_(-1), good_(false), visibleTimes_(0), matchedTimes_(0),
      pos_(Vector3d(0,0,0)), norm_(Vector3d(0,0,0))
{}

MapPoint::MapPoint(long unsigned int id, const Vector3d& position,
                   const Vector3d& norm, Frame* frame, const Mat& descriptor)
    : id_(id), good_(false), visibleTimes_(1), matchedTimes_(1), pos_(position),
      norm_(norm),  descriptor_(descriptor)
{
    observedFrames_.push_back(frame);
}

MapPoint::Ptr MapPoint::createMapPoint()
{
    return MapPoint::Ptr(
        new MapPoint(factoryId_++, Vector3d(0,0,0), Vector3d(0,0,0))
    );
}

MapPoint::Ptr MapPoint::createMapPoint(const Vector3d& pos_world,
                                       const Vector3d& norm,
                                       const Mat& descriptor,
                                       Frame* frame )
{
    return MapPoint::Ptr(
        new MapPoint(factoryId_++, pos_world, norm, frame, descriptor)
    );
}

unsigned long MapPoint::factoryId_ = 0;

}
