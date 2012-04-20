#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/gpu/gpu.hpp>        // GPU structures and methods

using namespace std;
using namespace cv;

namespace enc = sensor_msgs::image_encodings;

//static const char WINDOW[] = "Image window";

class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;
	int number_;

	cv::gpu::GpuMat keypoints1_dev, descriptors1_dev;
	cv::gpu::GpuMat keypoints2_dev, descriptors2_dev;
	cv::gpu::GpuMat img_dev, mask_dev;

	cv::gpu::SURF_GPU surf;	
  
  
public:
	ImageConverter() : it_(nh_)
	{
		number_ = 0;		// File number
		image_pub_ = it_.advertise("out", 1);
		image_sub_ = it_.subscribe("in", 1, &ImageConverter::imageCb, this);
//		cv::namedWindow(WINDOW);

		// GPU initialization
		
		cv::Mat mask_host = cv::Mat::ones(480,640,CV_8UC1);
		mask_dev.upload(mask_host);
		

		cv::Mat src_host(480,640,CV_8UC3);
		cv::gpu::GpuMat dst_device, src_device;
		src_device.upload(src_host);
		cv::gpu::cvtColor(src_device,dst_device,CV_BGR2GRAY);
		cv::Mat result_host;
		dst_device.download(result_host);
		ROS_INFO("GPU initialization done...");
  }

	~ImageConverter()
	{
//    cv::destroyWindow(WINDOW);
	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
		ros::Time begin,end1,end2,end3,end4,end;
		begin = ros::Time::now();
		
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

//    	cv::imshow(WINDOW, cv_ptr->image);

		try
		{
			cv::gpu::GpuMat src_dev;
			src_dev.upload(cv_ptr->image);
			cv::gpu::cvtColor(src_dev,img_dev,CV_BGR2GRAY);
			end1 = ros::Time::now();
			// SURF GPU	
			surf(img_dev,mask_dev,keypoints2_dev, descriptors2_dev);
			end2 = ros::Time::now();						
			vector<cv::KeyPoint> keypoints2_host;			
			surf.downloadKeypoints(keypoints2_dev, keypoints2_host);
			

			
//			BruteForceMatcher<cv::L2<float> > matcher;
//			vector<DMatch> matches;
//			matcher.match(descriptors_host, descriptors_host, matches);

			
			cv::Mat result1_host;
			img_dev.download(result1_host);
			cv::imshow("Result", result_host);
			drawKeypoints(result1_host,keypoints2_host,result1_host);
			end3 = ros::Time::now();
			char filename_gpu[40];
			sprintf(filename_gpu,"gpu_kinect_rgb_%03d.jpg",number_);
		    cv::imwrite(filename_gpu,result1_host);    
		}
		catch(const cv::Exception& ex)
		{
			std::cout << "Error: " << ex.what() << std::endl;
		}

//		char filename[40];
//		sprintf(filename,"kinect_rgb_%03d.jpg",number_);
//		cv::imwrite(filename,cv_ptr->image);    
//		number_++;	// File number    
		
		end4 = ros::Time::now();
		image_pub_.publish(cv_ptr->toImageMsg());
		end = ros::Time::now();
		ROS_INFO("Callback takes %f %f %f %f %f %f second",end.toSec() - begin.toSec(),end1.toSec() - begin.toSec(),end2.toSec() - end1.toSec(),end3.toSec() - end2.toSec(),end4.toSec() - end3.toSec(),end.toSec() - end4.toSec());
 		
	}
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
