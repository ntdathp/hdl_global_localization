#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <hdl_global_localization/SetGlobalMap.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "set_global_map_from_pcd");
  ros::NodeHandle nh("~");

  std::string pcd_path;
  nh.param<std::string>("pcd_path", pcd_path, "");
  std::string frame_id;
  nh.param<std::string>("frame_id", frame_id, "map");

  if (pcd_path.empty()) {
    ROS_ERROR("~pcd_path is empty. Set it in launch file.");
    return 1;
  }

  ROS_INFO("Loading PCD: %s", pcd_path.c_str());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile(pcd_path, *cloud) != 0) {
    ROS_ERROR("Failed to load PCD file: %s", pcd_path.c_str());
    return 1;
  }

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud, cloud_msg);
  cloud_msg.header.stamp = ros::Time::now();
  cloud_msg.header.frame_id = frame_id;

  ros::ServiceClient client =
      nh.serviceClient<hdl_global_localization::SetGlobalMap>("/hdl_global_localization/set_global_map");

  ROS_INFO("Waiting for /hdl_global_localization/set_global_map service...");
  client.waitForExistence();

  hdl_global_localization::SetGlobalMap srv;
  srv.request.global_map = cloud_msg;

  ROS_INFO("Calling /hdl_global_localization/set_global_map ...");
  if (client.call(srv)) {
    ROS_INFO("SetGlobalMap success.");
  } else {
    ROS_ERROR("SetGlobalMap failed.");
    return 1;
  }

  ROS_INFO("Done. Shutting down.");
  return 0;
}
