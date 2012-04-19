#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/gpu/gpu.hpp>        // GPU structures and methods

namespace enc = sensor_msgs::image_encodings;

//static const char WINDOW[] = "Image window";

class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;
	int number_;
  
public:
	ImageConverter() : it_(nh_)
	{
		number_ = 0;		// File number
		image_pub_ = it_.advertise("out", 1);
		image_sub_ = it_.subscribe("in", 1, &ImageConverter::imageCb, this);
//		cv::namedWindow(WINDOW);
  }

	~ImageConverter()
	{
//    cv::destroyWindow(WINDOW);
	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
		ROS_INFO("Callback...");
		
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
			ROS_INFO("GPU starts...");
			cv::Mat src_host;
			cvtColor(cv_ptr->image, src_host, CV_BGR2GRAY);
	
			cv::gpu::GpuMat dst, src;
			src.upload(src_host);

			cv::gpu::threshold(src, dst, 128.0, 255.0, CV_THRESH_BINARY);

			cv::Mat result_host;
			dst.download(result_host);
//			cv::imshow("Result", result_host);
			char filename_gpu[40];
			sprintf(filename_gpu,"kinect_rgb_%3d_gpu.jpg",number_);
		    cv::imwrite(filename_gpu,result_host);    
			ROS_INFO("GPU end...");
			cv::waitKey();
		}
		catch(const cv::Exception& ex)
		{
			std::cout << "Error: " << ex.what() << std::endl;
		}

		char filename[40];
		sprintf(filename,"kinect_rgb_%3d.jpg",number_);
		cv::imwrite(filename,cv_ptr->image);    
		cv::waitKey(3);
		number_++;	// File number    
		
		image_pub_.publish(cv_ptr->toImageMsg());
		ROS_INFO("Callback end...");
		
	}
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
