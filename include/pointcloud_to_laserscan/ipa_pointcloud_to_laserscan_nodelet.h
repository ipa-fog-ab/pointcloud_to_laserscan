/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  Copyright (c) 2015, Fraunhofer IPA.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holders nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */
 
/*
 * Author: Paul Bovbel
 * Author: Sofie Nilsson
 */

#ifndef IPA_POINTCLOUD_TO_LASERSCAN_POINTCLOUD_TO_LASERSCAN_NODELET
#define IPA_POINTCLOUD_TO_LASERSCAN_POINTCLOUD_TO_LASERSCAN_NODELET

#include "ros/ros.h"
#include "boost/thread/mutex.hpp"

#include "nodelet/nodelet.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "sensor_msgs/PointCloud2.h"
#include <pointcloud_to_laserscan/scan_outlier_removal_filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>


namespace pointcloud_to_laserscan
{
  typedef tf2_ros::MessageFilter<sensor_msgs::PointCloud2> MessageFilter;
/**
* Class to process incoming pointclouds into laserscans. Some initial code was pulled from the defunct turtlebot
* pointcloud_to_laserscan implementation.
*/
  class IpaPointCloudToLaserScanNodelet : public nodelet::Nodelet
  {

  public:
    IpaPointCloudToLaserScanNodelet();
    void configure_filter();

  private:
    virtual void onInit();

    void cloudCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);

    void convert_pointcloud_to_laserscan(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, sensor_msgs::LaserScan &output, const tf2::Transform &T, const double range_min );

    ros::NodeHandle nh_, private_nh_;
    ros::Publisher pub_;
    boost::mutex connect_mutex_;

    boost::shared_ptr<tf2_ros::Buffer> tf2_;
    boost::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

    ros::Subscriber sub_;
    
    scan_outlier_filter::ScanOutlierRemovalFilter outlier_filter_;
    // ROS Parameters
    unsigned int input_queue_size_;
    std::string target_frame_;
    double tolerance_;
    double min_height_, max_height_, angle_min_, angle_max_, angle_increment_, scan_time_, range_min_, range_max_;
    bool use_inf_;
    bool use_outlier_filter_;

// new addition : pointclouds definition for pcl
    pcl::PointCloud<pcl::PointXYZ> cloud1_in, cloud2_in, cloud_out;
    void concatenate_PCL(const pcl::PointCloud<pcl::PointXYZ> cloud_in);
   
  };

}  // pointcloud_to_laserscan

#endif  // IPA_POINTCLOUD_TO_LASERSCAN_POINTCLOUD_TO_LASERSCAN_NODELET
