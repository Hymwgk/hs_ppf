#include <iostream>
#include <fstream>
#include<ros/ros.h>
//PCL 的相关的头文件
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include"subsampleAndCalculateNormals.h"


int main (int argc, char** argv)
{
  // Initialize ROS 声明节点的名称
  ros::init (argc, argv, "hs_ppf");
	
  ros::spin();
  
  return 0;

}
