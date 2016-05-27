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

//Boost
//#include <math/special_functions/spherical_harmonic.hpp>
#include "/usr/include/boost/math/special_functions/spherical_harmonic.hpp"


//openCV
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>




class PCL_2_Panoramic
{
	ros::NodeHandle n;
	std::string camera_name_;
	ros::Subscriber subPC_;
	ros::Subscriber subChatter_;	
	



	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
	

	ros::Timer timer_;

	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;	
	cv_bridge::CvImagePtr cv_ptr;
	
	public:
		PCL_2_Panoramic(const std::string& camera_name) : camera_name_(camera_name),it_(n)
		{
			subPC_ = n.subscribe ("camera/depth/points", 10, &PCL_2_Panoramic::PCL_callback,this);
			subChatter_ = n.subscribe ("chatter", 1, &PCL_2_Panoramic::chatterCallback,this);

			cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
			
			timer_ = n.createTimer(ros::Duration(0.5), &PCL_2_Panoramic::metronomeCallback, this);
			image_pub_ = it_.advertise("/image_out", 1);
			
			cv_ptr.reset (new cv_bridge::CvImage);
			cv_ptr->encoding = "32FC1";
			
			Initialize_Panoramic(3,2);
			

		}

		~PCL_2_Panoramic()
		{
//			cv::destroyWindow("OPENCV_WINDOW");
		}




	void PCL_callback(const sensor_msgs::PointCloud2ConstPtr& input){

		
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		
		pcl::fromROSMsg(*input, *cloud );  //GROOVY


//   HYDRO /////////////////////////////////
//		pcl::PCLPointCloud2 pcl_pc;
//		pcl_conversions::toPCL(*input, pcl_pc);
//		pcl::fromPCLPointCloud2(pcl_pc, *cloud);

		
//		viewer_.showCloud(cloud);
	}






	void chatterCallback(const std_msgs::String::ConstPtr& msg)
	{
		ROS_INFO("I heard: [%s]", msg->data.c_str());


		if (pcl::io::loadPCDFile<pcl::PointXYZ> ("Concatenated_small.pcd", *cloud_) == -1) //* load the file
		{
			PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
		}
		std::cout << "Loaded "
				<< cloud_->width * cloud_->height
				<< " data points from Concatenated.pcd with the following fields: "
				<< std::endl;
		
			ROS_INFO("I heard: [%s] again", msg->data.c_str());
			pcl2pano();
		
	}


	void Initialize_Panoramic(unsigned n, int m){
		float pano[180][360];
		float latitud_float, longitud_float;
		cv::Mat Pano_Sphere = cv::Mat::zeros(180, 360, CV_32F);

		std::cout << "Spherical Starting " << std::endl;
				
		for(unsigned int latitud = 0; latitud < 180; latitud++) {
			for(unsigned int longitud = 0; longitud < 360; longitud++) {
				latitud_float   =  latitud*M_PI/180;
				longitud_float  =  longitud*M_PI/180;
				pano[latitud][longitud] = boost::math::spherical_harmonic_i(3, 2, latitud_float, longitud_float);
				Pano_Sphere.at<float>(latitud , longitud) = pano[latitud][longitud];
			}
		}
		
//		cv::imshow("Imagen ",Pano_Sphere);

		Pano_Sphere.copyTo(cv_ptr->image);////mas importante
		
		std::cout << "Spherical Done " << pano[25][42] << std::endl;
	}


/////////////////		
		void metronomeCallback(const ros::TimerEvent&)
		{
//		  ROS_INFO("tic tac");
		 publish_Image();
		}



////////////////////////////		
		void publish_Image(){
			image_pub_.publish(cv_ptr->toImageMsg());
//			cv::imshow("OPENCV_WINDOW", cv_ptr->image);
//			cv::waitKey(3);
		}
		
		
		void pcl2pano(){
			cv::Mat Pano_from_PCL = cv::Mat::zeros(180, 360, CV_32F);
			float x, y, z, rho, previous;
			int theta, phi;
			
			for (int i=0; i < cloud_->size(); i++){
				x=cloud_->points[i].x;
				y=cloud_->points[i].y;
				z=cloud_->points[i].z - 1.5;
				
				rho = sqrt(x*x + y*y + z*z);
				
				phi = round ( acos( z / rho) *180/M_PI);
				
				theta = round ( atan2(y , x)*180/M_PI   ) + 180;
				
//				std::cout << "rho  " << rho << "  phi  " << phi << "  theta  " << theta << std::endl;
				previous = Pano_from_PCL.at<float> (phi, theta) ;
				
				if (previous == 0){
					Pano_from_PCL.at<float> (phi, theta) = rho;
				}
				else{
					Pano_from_PCL.at<float> (phi, theta) =  std::min( rho,  Pano_from_PCL.at<float> (phi, theta)  );
				}
				
			}
			
			cv::Size s = Pano_from_PCL.size();
			
			pcl::PointXYZ minimum, maximum;// =new(pcl::PointXYZ);
//			Eigen::Vector4f maximun;// =new(pcl::PointXYZ);

			pcl::getMinMax3D(*cloud_, minimum, maximum);
			
			
//			std::cout << "Width " << s.width << "  Height  "<< s.height << std::endl;
			std::cout << "Min_X " << minimum.x  << "Min_Y " << minimum.y << "Min_Z " << minimum.z << std::endl;
			std::cout << "Max_X " << maximum.x  << "Max_Y " << maximum.y << "Max_Z " << maximum.z << std::endl;



			
			for (theta = 0; theta < s.width ; theta++){
				for (phi =0; phi < s.height ; phi++){
					if ( Pano_from_PCL.at<float> (phi, theta) == 0   && phi < 80){
						Pano_from_PCL.at<float> (phi, theta) = (1.3) /  (  cos(phi*(M_PI/180) )   );// ceiling at 2.8 m
						int a=1;
					}
				}
			}

			cv::Mat Pano_16U = cv::Mat::zeros(180, 360, CV_16U);			
			Pano_from_PCL.convertTo(Pano_16U, CV_16U, 1000, 0 );			
			
			cv::medianBlur(Pano_16U, Pano_16U, 3);
			
			imwrite( "Gray_Image.png", Pano_16U );
			
			

			
			Pano_from_PCL.copyTo(cv_ptr->image);
			publish_Image();
			for(int k=0; k<180; k++){
				std::cout << "Prueba de alguno " << Pano_from_PCL.at<float> (60, k) << std::endl;
			}
			
		}

 


};
















int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "pcl_2_Pano");
		

//	boost::math::spherical_harmonic_r(unsigned n, int m, T1 theta, T2 phi); //  Theta [0 to PI]  Phi [0 to 2 PI] 
	
	std::string camera_name = "camera";
	PCL_2_Panoramic pcl2Pano(camera_name);

	ros::spin();
	
	return 0;

}
