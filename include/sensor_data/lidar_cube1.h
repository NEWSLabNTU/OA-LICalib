/*
 * OA-LICalib:
 * Observability-Aware Intrinsic and Extrinsic Calibration of LiDAR-IMU Systems
 *
 * Copyright (C) 2022 Jiajun Lv
 * Copyright (C) 2022 Xingxing Zuo
 * Copyright (C) 2022 Kewei Hu
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
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <angles/angles.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <iostream>
#include <vector>

#include <sensor_data/lidar_feature.h>
#include <sensor_msgs/PointCloud2.h>

namespace liso {



class Cube1LiDAR {
 public:
  typedef std::shared_ptr<Cube1LiDAR> Ptr;

  void get_organized_and_raw_cloud(
      const sensor_msgs::PointCloud2::ConstPtr &lidarMsg,
      LiDARFeature &output) {
    BlickfeldCloud pc_in;
    printf("parsing lidar message");
    pcl::fromROSMsg(*lidarMsg, pc_in);
    printf("finish parsing lidar message");


    double timebase = lidarMsg->header.stamp.toSec();
    output.timestamp = timebase;

    /// point cloud
    output.full_features->clear();
    output.full_features->height = lidarMsg->height;
    output.full_features->width = lidarMsg->width;
    output.full_features->is_dense = lidarMsg->is_dense;
    output.full_features->resize(output.full_features->height *
                                 output.full_features->width);

    /// raw_data
    output.raw_data->height = lidarMsg->height;
    output.raw_data->width = lidarMsg->width;
    output.raw_data->is_dense = false;
    output.raw_data->resize(output.raw_data->height * output.raw_data->width);

    PosPoint NanPoint;
    NanPoint.x = NAN;
    NanPoint.y = NAN;
    NanPoint.z = NAN;
    NanPoint.timestamp = timebase;

    int num_points = lidarMsg->height * lidarMsg->width;
    for (int k = 0; k < num_points; k++) {
      output.full_features->points[k] = NanPoint;
      output.raw_data->points[k] = NanPoint;
    }

    for (int h = 0; h < lidarMsg->height; h++) {
      for (int w = 0; w < lidarMsg->width; w++) {
        const auto &src = pc_in.at(w, h);

        // std::cout << w << "," << h << "," << src.x << "," << src.y
        //           << "," << src.z << "," << src.t << "\n";

        double depth = sqrt(src.x * src.x + src.y * src.y + src.z * src.z);
        if (depth > 60) continue;

        PosPoint dst_point;
        dst_point.x = src.x;
        dst_point.y = src.y;
        dst_point.z = src.z;
        dst_point.timestamp = timebase;

        PosPoint point_raw;
        // t_offset wrt. first point
        point_raw.timestamp = timebase;
        // laser id
        point_raw.x = src.id;
        // angle rad
        point_raw.y = timebase;
        // range m
        point_raw.z = depth;

        output.full_features->at(w, h) = dst_point;
        output.raw_data->at(w, h) = point_raw;
      }
    }
    // pcl::io::savePCDFileBinaryCompressed(
    //     "/home/ha/ros_ws/catkin_liso/ouster_cloud_in.pcd", pc_in);
    // pcl::io::savePCDFileBinaryCompressed(
    //     "/home/ha/ros_ws/catkin_liso/ouster_cloud_out.pcd",
    //     *output.full_features);
  }

 private:

};
}  // namespace liso
