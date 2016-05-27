//ROS
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include <tf/transform_listener.h>



//PCL
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>


//#include <pcl/conversions.h>
//#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/PCLPointCloud2.h>

#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>







class PCL_concatenate
{
	ros::NodeHandle n;
	std::string camera_name_;
	ros::Subscriber subPC_;
	ros::Subscriber subChatter_;	
	
	pcl::visualization::CloudViewer viewer_;

	bool first_time_;	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_acum_;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transform_;
	
	tf::TransformListener listener_;

	
	public:
		PCL_concatenate(const std::string& camera_name) : camera_name_(camera_name), viewer_("Cluster viewer")
		{
			subPC_ = n.subscribe ("camera/depth/points", 10, &PCL_concatenate::PCL_callback,this);
			subChatter_ = n.subscribe ("chatter", 1, &PCL_concatenate::chatterCallback,this);
			first_time_= true;
			cloud_acum_.reset(new pcl::PointCloud<pcl::PointXYZ>);
			cloud_transform_.reset(new pcl::PointCloud<pcl::PointXYZ>);
		}

		~PCL_concatenate()
		{
			
		}




	void PCL_callback(const sensor_msgs::PointCloud2ConstPtr& input){
//		pcl::PCLPointCloud2 pcl_pc;
//		pcl_conversions::toPCL(*input, pcl_pc);
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		
		pcl::fromROSMsg(*input, *cloud );  //GROOVY

//		pcl::fromPCLPointCloud2(pcl_pc, *cloud);



    tf::StampedTransform transform;
    
    try{
		listener_.waitForTransform("/openni_rgb_optical_frame", "/world", ros::Time(0), ros::Duration(3.0));
        listener_.lookupTransform( "/world", "/openni_rgb_optical_frame",  ros::Time(0), transform);
		ROS_INFO("Transform OK");
//		std::cout << "Entre y la transformacion esta bien"<< std::endl;
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s  pp",ex.what());

    }
	

	Eigen::Matrix4f trans_eigen = transformTf2Affine(transform);			
	pcl::transformPointCloud (*cloud, *cloud_transform_, trans_eigen);
	

	
//	pcl_ros::transformPointCloud (*cloud, *cloud_transform_,transform);



		if (first_time_){
			*cloud_acum_ = *cloud_transform_;
//			cloud_acum_=cloud;
			first_time_=false;
		}
		else{
			*cloud_acum_ =  *cloud_transform_ + *cloud_acum_;
//			*cloud_acum_ =  *cloud_transform_ + *cloud;
			ROS_INFO("Im in");
		}
		
		viewer_.showCloud(cloud_transform_);
	}




	void chatterCallback(const std_msgs::String::ConstPtr& msg)
	{
		ROS_INFO("I heard: [%s]", msg->data.c_str());
	//	viewer_.showCloud(cloud_acum_);
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

		  // Create the filtering object
		pcl::VoxelGrid<pcl::PointXYZ> sor;
		sor.setInputCloud (cloud_acum_);
		sor.setLeafSize (0.01f, 0.01f, 0.01f);
		sor.filter (*cloud_filtered);
		
		pcl::io::savePCDFileASCII ("Concatenated_small.pcd", *cloud_filtered);
		ROS_INFO("I heard: [%s] again", msg->data.c_str());
	
	}



	Eigen::Matrix4f transformTf2Affine(tf::StampedTransform transform){

		
		tf::Matrix3x3 mat = transform.getBasis();
		
//		tfScalar roll, pitch, yaw;		
//		mat.inverse().getRPY(roll, pitch, yaw,1);
//		std::cerr << "roll "<< roll*180/M_PI<<"  pitch  "<<pitch*180/M_PI<< "  yaw  "<< yaw*180/M_PI  << std::endl;                
		
		
		Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
		
		transform_1(0,0)= mat.getRow(0).x();
		transform_1(0,1)= mat.getRow(0).y();
		transform_1(0,2)= mat.getRow(0).z();		
		
		transform_1(1,0)= mat.getRow(1).x();
		transform_1(1,1)= mat.getRow(1).y();
		transform_1(1,2)= mat.getRow(1).z();
		
		transform_1(2,0)= mat.getRow(2).x();
		transform_1(2,1)= mat.getRow(2).y();
		transform_1(2,2)= mat.getRow(2).z();		
					
//		transform_1=transform_1.inverse();
						
		transform_1(0,3)= transform.getOrigin().x();
		transform_1(1,3)= transform.getOrigin().y();
		transform_1(2,3)= transform.getOrigin().z();

//		transform_1=transform_1.inverse();
								
		std::cerr << "Matrix "      << std::endl;                
		std::cerr << transform_1 << std::endl;
	
		return transform_1;
	}



 


};
















int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "pcl_concatenate");
		

	
	std::string camera_name = "camera";
	PCL_concatenate pcl_C(camera_name);

	ros::spin();
	
	return 0;

}
