//
// Created by harish on 4/5/20.
//
#include "ros/ros.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

int i = 0;
int a = 0;
int j = 0;
int b = 0;
image_transport::Publisher pub;
// https://answers.ros.org/question/60182/error-unable-to-convert-32fc1-image-to-bgr8-while-extracting-rbgd-data-from-a-bag/

void depthToCV8UC1(const cv::Mat& float_img, cv::Mat& mono8_img){
    // Process images
    if(mono8_img.rows != float_img.rows || mono8_img.cols != float_img.cols){
        mono8_img = cv::Mat(float_img.size(), CV_8UC1);
    }
    cv::convertScaleAbs(float_img, mono8_img, 255.0/300.0, 0.0);
}

void depthCallback(const sensor_msgs::ImageConstPtr& original_image)
{
    cv_bridge::CvImagePtr cv_ptr;
    // Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
    try
    {
        cv_ptr = cv_bridge::toCvCopy(original_image);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }
    //Copy the image.data to imageBuf.
    cv::Mat depth_float_img = cv_ptr->image;
    cv::Mat depth_mono8_img;
    cv::patchNaNs(depth_float_img, 300.0);
    depthToCV8UC1(depth_float_img, depth_mono8_img);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", depth_mono8_img).toImageMsg();
    pub.publish(msg);
}

// USE THE FOLLOWING COMMAND IF THE IMAGES ARE NOT CLEAR
// rosrun image_view extract_images _sec_per_frame:=0.01 image:=/camera/rgb/image_raw
void rgbCallback(const sensor_msgs::ImageConstPtr& original_image)
{
    j++;
    if ( j % 90 != 0 ) {
        return;
    }
    cv_bridge::CvImagePtr cv_ptr;
    // Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
    try
    {
        cv_ptr = cv_bridge::toCvCopy(original_image);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat rgb_img = cv_ptr->image;
    cvtColor(rgb_img, rgb_img, CV_BGR2RGB);
    string path = "/home/harish/rosbag/baylands/rgb/";
    string frame = "frame";
    string num = to_string(b);
    string new_num = std::string(4 - num.length(), '0') + num;
    string filename = path + frame + new_num + ".jpg";
    b = b + 1;
    imwrite(filename, rgb_img);
}

void callback(const sensor_msgs::ImageConstPtr& depth, const sensor_msgs::ImageConstPtr& rgb)
{
    j++;
    if ( j % 20 != 0 ) {
        return; // Adjust this to decide how frequently to save images
    }
    cv_bridge::CvImagePtr cv_ptr;
    // Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
    try
    {
        cv_ptr = cv_bridge::toCvCopy(rgb);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat rgb_img = cv_ptr->image;
    cvtColor(rgb_img, rgb_img, CV_BGR2RGB);
    string path = "/home/harish/rosbag/test2/rgb/";
    string frame = "frame";
    string num = to_string(b);
    string new_num = std::string(4 - num.length(), '0') + num;
    string filename = path + frame + new_num + ".jpg";
    b = b + 1;
    imwrite(filename, rgb_img);
    //cv_bridge::CvImagePtr cv_ptr;
    // Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
    try
    {
        cv_ptr = cv_bridge::toCvCopy(depth);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }
    //Copy the image.data to imageBuf.
    cv::Mat depth_float_img = cv_ptr->image;
    cv::Mat depth_mono8_img;
    string path1 = "/home/harish/rosbag/test2/depth/";
    string frame1 = "frame";
    string num1 = to_string(b);
    string new_num1 = std::string(4 - num.length(), '0') + num;
    string filename1 = path1 + frame1 + new_num1 + ".jpg";
    //a = a + 1;
    double min, max;
    cv::minMaxLoc(depth_float_img, &min, &max);
    ROS_INFO("min: %f", min);
    ROS_INFO("max: %f", max);
    cv::patchNaNs(depth_float_img, 1000.0);
    depthToCV8UC1(depth_float_img, depth_mono8_img);
    imwrite(filename1, depth_mono8_img);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", depth_mono8_img).toImageMsg();
    pub.publish(msg);
}
  


int main( int argc, char** argv ){
    ros::init(argc, argv, "talker");

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber depth_sub = it.subscribe("/camera/depth/image_raw", 1, depthCallback);

//  image_transport::Subscriber rgb_sub = it.subscribe("/camera/rgb/image_raw", 1, rgbCallback);
//    Uncomment below lines to save depth and rgb data together
//    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth/image_raw", 1);
//    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
//    message_filters::TimeSynchronizer<sensor_msgs::Image,sensor_msgs::Image> sync(depth_sub, rgb_sub, 10);
//    sync.registerCallback(boost::bind(&callback, _1, _2));

    pub = it.advertise("/my_test/camera/image", 1);
    while (ros::ok())
    {
        ros::spinOnce();
    }

}
