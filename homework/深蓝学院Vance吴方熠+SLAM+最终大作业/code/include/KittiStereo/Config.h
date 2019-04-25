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

#ifndef CONFIG_H
#define CONFIG_H

#include "KittiStereo/common_include.h"

namespace KittiStereo
{

class Config
{
private:
    static std::shared_ptr<Config> config_;

    cv::FileStorage file_;  // 配置文件
    Config () {}            // 将构造函数定位私有成员防止其在其他地方被建立

public:
    ~Config();

    // 设置配置文件
    static void setParameterFile(const std::string& filename);

    // 获取参数值
    template< typename T >
    static T get(const std::string& key_name)
    {
        return T(Config::config_->file_[key_name]);
    }
};

} // namespace

#endif // CONFIG_H
