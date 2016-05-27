//ROS
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"




//PCL
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/io/ply_io.h>
//#include <pcl/conversions.h>
//#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/PCLPointCloud2.h>

pcl::visualization::CloudViewer viewer ("Cluster viewer");




void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());


}

void read_file(){

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  
//  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/leonardo/Downloads/2011-11-28_20.58.03/pointcloud.pcd", *cloud) == -1) //* load the file
//  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/leonardo/Downloads/region_growing_tutorial.pcd", *cloud) == -1) //* load the file
  if (pcl::io::loadPLYFile<pcl::PointXYZ> ("/home/leonardo/Downloads/cloud.ply", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
  }
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;


  	viewer.showCloud(cloud);
}




// callback signature
void callback(const sensor_msgs::PointCloud2ConstPtr& input){
	ROS_INFO("Im in");
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	
	
	pcl::fromROSMsg(*input, *cloud );  
	
	
	
	
	viewer.showCloud(cloud);


}




int main(int argc, char **argv)
{

  ros::init(argc, argv, "pcl_test");


  ros::NodeHandle n;

	read_file();


  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
// to create a subscriber, you can do this (as above):
  ros::Subscriber subPC = n.subscribe<sensor_msgs::PointCloud2> ("camera/depth/points", 1, callback);
  ros::spin();

  return 0;
}
