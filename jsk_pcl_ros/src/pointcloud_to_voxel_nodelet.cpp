#define BOOST_PARAMETER_MAX_ARITY 7

#include <pcl_conversions/pcl_conversions.h>
#include "jsk_pcl_ros/pointcloud_to_voxel.h"

namespace jsk_pcl_ros
{

  void PointCloudToVoxel::onInit()
  {
    DiagnosticNodelet::onInit();

    pub_ = advertise<costmap_2d::VoxelGrid>(*pnh_, "output", 1);
    onInitPostProcess();
  }

  void PointCloudToVoxel::subscribe()
  {
    sub_ = pnh_->subscribe("input", 1, &PointCloudToVoxel::convert, this);
  }

  void PointCloudToVoxel::unsubscribe()
  {
    sub_.shutdown();
  }

  void PointCloudToVoxel::convert(
    const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg)
  {
    vital_checker_->poke();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud->is_dense = false;

    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(*cloud, ros_cloud);
    ros_cloud.header = pointcloud_msg->markers[0].header;
    pub_.publish(ros_cloud);
  }

}  // namespace jsk_pcl_ros_utils

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_pcl_ros::PointCloudToVoxel, nodelet::Nodelet);
