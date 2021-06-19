// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK Lab
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
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
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
 *********************************************************************/

#include "jsk_pcl_ros_utils/point_indices_to_pointcloud.h"
#include "jsk_pcl_ros/pcl_conversion_util.h"

#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

namespace jsk_pcl_ros_utils
{
  void PointIndicesToPointCloud::onInit()
  {
    DiagnosticNodelet::onInit();
    pub_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "output", 1);
  }

  void PointIndicesToPointCloud::subscribe()
  {
    sub_indices_.subscribe(*pnh_, "input", 1);
    sub_cloud_.subscribe(*pnh_, "input/cloud", 1);
    if (approximate_sync_) {
      async_ = boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy> >(100);
      async_->connectInput(sub_indices_, sub_cloud_);
      async_->registerCallback(
        boost::bind(&PointIndicesToPointCloud::convert, this, _1, _2));
    }
    else {
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
      sync_->connectInput(sub_indices_, sub_cloud_);
      sync_->registerCallback(
        boost::bind(&PointIndicesToPointCloud::convert, this, _1, _2));
    }
  }

  void PointIndicesToPointCloud::unsubscribe()
  {
    sub_cloud_.unsubscribe();
    sub_indices_.unsubscribe();
  }

  void PointIndicesToPointCloud::convert(
    const PCLIndicesMsg::ConstPtr& indices_msg,
    const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
  {
    pcl::PointCloud<PointT>::Ptr input(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*cloud_msg, *input);
    pcl::PointIndices::Ptr indices (new pcl::PointIndices ());
    pcl_conversions::toPCL(*indices_msg, *indices);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(input);
    extract.setIndices(indices);
    pcl::PointCloud<PointT>::Ptr output(new pcl::PointCloud<PointT>());
    extract.filter(*output);

    sensor_msgs::PointCloud2::Ptr out_cloud(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*output, *out_cloud);
    out_cloud->header = cloud_msg->header;
    pub_.publish(out_cloud);
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros_utils::PointIndicesToPointCloud, nodelet::Nodelet);
