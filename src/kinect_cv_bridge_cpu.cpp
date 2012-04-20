#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/features2d/features2d.hpp>
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

	cv::Mat mask;
	cv::Mat img1,img2;
	vector<KeyPoint> keypoints1, keypoints2;
	vector<float> descriptors1, descriptors2;
  
public:
	ImageConverter() : it_(nh_)
	{
		number_ = 1;		// File number
		image_pub_ = it_.advertise("out", 1);
		image_sub_ = it_.subscribe("in", 1, &ImageConverter::imageCb, this);
//		cv::namedWindow(WINDOW);
		mask = cv::Mat::ones(480,640,CV_8UC1);
		// GPU initialization
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
		if (number_ % 2)
			img1 = cv_ptr->image;
		else
			img2 = cv_ptr->image;

		cv::SURF surf;		
		if (number_ % 2)
			surf(img1, keypoints1, mask, descriptors1);
		else
			surf(img2, keypoints2, mask, descriptors2);

	
		cv::BruteForceMatcher<L2<float> > matcher;
		vector<DMatch> matches;
		
		matcher.match(descriptors1, descriptors2, matches);
		
		cv::Mat img_matches;		
		if (!(number_ % 2))
			drawMatches(img1, keypoints1, img2, keypoints2, matches, img_matches);
		else
			drawMatches(img2, keypoints2, img1, keypoints1, matches, img_matches);

		char filename_cpu[40];
		sprintf(filename_cpu,"cpu_kinect_rgb_matches_%03d.jpg",number_);
		cv::imwrite(filename_cpu,img_matches);    

//		char filename[40];
//		sprintf(filename,"kinect_rgb_%03d.jpg",number_);
//		cv::imwrite(filename,cv_ptr->image);    
		number_++;	// File number    W
		
		end4 = ros::Time::now();
		image_pub_.publish(cv_ptr->toImageMsg());
		end = ros::Time::now();
//		ROS_INFO("Callback takes %f %f %f %f %f %f second",
//					end.toSec() - begin.toSec(),
//					end1.toSec() - begin.toSec(),
//					end2.toSec() - end1.toSec(),
//					end3.toSec() - end2.toSec(),
//					end4.toSec() - end3.toSec(),
//					end.toSec() - end4.toSec());
 		ROS_INFO("%fs, %.2fHz",end.toSec() - begin.toSec(),1 / (end.toSec() - begin.toSec()));
 		
	}
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
