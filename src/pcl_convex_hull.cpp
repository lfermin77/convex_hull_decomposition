//ROS
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"

#include <sstream>


//PCL
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/features/normal_3d.h>

#include <pcl/io/ply_io.h>
#include <pcl/surface/convex_hull.h>

#include <pcl/visualization/cloud_viewer.h>





//#include <pcl/conversions.h>
//#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/PCLPointCloud2.h>





//pcl::visualization::CloudViewer viewer ("Cluster viewer");





void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());


}

void read_file(){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  	clock_t begin, end;
  
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
            



//////////////////////////////////////////////////////////////////////



	begin = clock();
	
	// Create a Concave Hull representation of the projected inliers
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ConvexHull<pcl::PointXYZ> chull;
	chull.setInputCloud (cloud);
	chull.setDimension(3);
	
	std::vector<pcl::Vertices> triangles;
	chull.reconstruct (*cloud_hull, triangles);
	
	pcl::PolygonMesh polygon;
//	polygon.cloud = *cloud_hull;
	polygon.polygons = triangles;

	sensor_msgs::PointCloud2  msg;
	
	pcl::toROSMsg (*cloud_hull, msg  );
	polygon.cloud = msg;


/*
  std::cout << "Concave hull has: " << cloud_hull->points.size ()  << " data points." << std::endl;
  std::cout << "Concave hull has: " << triangles.size ()  << " triangles" << std::endl;
  for (int i=0; i<triangles.size();i++){
	std::cout << "Polygon " << i <<" has "<<triangles[i].vertices.size()<<" points"  << std::endl;
	  for (int j=0; j<triangles[i].vertices.size(); j++){
		std::cout << "       vertex " << triangles[i].vertices[j] <<" is "<<cloud_hull->points[triangles[i].vertices[j] ]<< " points"  << std::endl;
	  }
  }
*/

	end = clock();
	std::cout << "Convex hull takes " << double(end - begin) / CLOCKS_PER_SEC <<" s" << std::endl;

/////////////////////////////////////////////////////////////////////

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCentroids (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normalsCentroids (new pcl::PointCloud<pcl::Normal>);
	// Fill in the cloud data
	cloudCentroids->width  = triangles.size();
	cloudCentroids->height = 1;
	cloudCentroids->points.resize (cloudCentroids->width * cloudCentroids->height);
	
	normalsCentroids->width  = triangles.size();
	normalsCentroids->height = 1;
	normalsCentroids->points.resize (normalsCentroids->width * normalsCentroids->height);
	
	// Generate the data
	for (size_t i = 0; i < triangles.size(); ++i)
	{
		pcl::PointXYZ a = cloud->points[ triangles[i].vertices[0] ];
		pcl::PointXYZ b = cloud->points[ triangles[i].vertices[1] ];
		pcl::PointXYZ c = cloud->points[ triangles[i].vertices[2] ];
		
		cloudCentroids->points[i].x = (a.x + b.x + c.x)/3;
		cloudCentroids->points[i].y = (a.y + b.y + c.y)/3;
		cloudCentroids->points[i].z = (a.z + b.z + c.z)/3;
		
		pcl::PointXYZ n1;
		n1.x = b.x - a.x;
		n1.y = b.y - a.y;
		n1.z = b.z - a.z;
		
		pcl::PointXYZ n2;
		n2.x = c.x - b.x;
		n2.y = c.y - b.y;
		n2.z = c.z - b.z;
		
		normalsCentroids->points[i].normal_x = (n1.y * n2.z) - (n1.z - n2.y);
		normalsCentroids->points[i].normal_y = (n1.z * n2.x) - (n1.x - n2.z);
		normalsCentroids->points[i].normal_z = (n1.x * n2.y) - (n1.y - n2.x);
		
		float centroid_norm = 	normalsCentroids->points[i].normal_x*normalsCentroids->points[i].normal_x +
								normalsCentroids->points[i].normal_y*normalsCentroids->points[i].normal_y +
								normalsCentroids->points[i].normal_z*normalsCentroids->points[i].normal_z;
		
		normalsCentroids->points[i].normal_x = normalsCentroids->points[i].normal_x / centroid_norm;
		normalsCentroids->points[i].normal_y = normalsCentroids->points[i].normal_y / centroid_norm;
		normalsCentroids->points[i].normal_z = normalsCentroids->points[i].normal_z / centroid_norm;
									
	}



//////////////////////////////////////////////////////////////////////////////

	begin = clock();
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_hull);
  n.setInputCloud (cloud_hull);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*normals);

	end = clock();
	std::cout << "Normal Estimation takes " << double(end - begin) / CLOCKS_PER_SEC <<" s" << std::endl;



//*
	// visualize normals
	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
		
//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloudCentroids, 0, 255, 0);
//	viewer.addPointCloud<pcl::PointXYZ> (cloudCentroids, single_color, "sample cloud");
//	viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloudCentroids, normalsCentroids, 1, 0.05, "normals");
	
	
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2(cloud_hull, 0, 0, 255);
	viewer.addPointCloud<pcl::PointXYZ> (cloud_hull, single_color2, "sample cloud 2");


	const std::string tmp("rectangle");
	const char* name = tmp.c_str();
	
	
	viewer.addPolygonMesh (polygon,name);
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255,255,255, name);
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,0.2f,name);


	
	
	
	
	
//	viewer.addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
	
	while (!viewer.wasStopped ())
	{
	viewer.spinOnce ();
	}
//*/





//  	viewer.showCloud(cloud_hull);
}




// callback signature
void callback(const sensor_msgs::PointCloud2ConstPtr& input){
	ROS_INFO("Im in");
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	
	
	pcl::fromROSMsg(*input, *cloud );  
	
	
	
	
//	viewer.showCloud(cloud);


}




int main(int argc, char **argv)
{

 // ros::init(argc, argv, "pcl_test");


//  ros::NodeHandle n;

	read_file();



	





/*
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
// to create a subscriber, you can do this (as above):
  ros::Subscriber subPC = n.subscribe<sensor_msgs::PointCloud2> ("camera/depth/points", 1, callback);
  ros::spin();
*/
  return 0;
}
