#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Header.h>
#include <trn_imitation_learning/velocity.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <fstream>
#include <chrono>
#include <iostream>


//using namespace cv;
using namespace sensor_msgs;
using namespace message_filters;
using namespace trn_imitation_learning;
using namespace std;
int i = 0;
ros::Publisher pub;


void callback(const ImageConstPtr& original_image, const velocityConstPtr& vel)
{
    // TODO: Use lidar to better approximate the ground beneath
    i++;
    ROS_INFO("count: %s", std::to_string(i).c_str());
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(original_image);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat rgb_img = cv_ptr->image;
    cvtColor(rgb_img, rgb_img, cv::COLOR_BGR2RGB);
    std_msgs::Header h = original_image->header;
    string velocity = vel->velocity;
    std::string path = "";


    if (velocity == "0"){
        path = "/home/harish/learning/depth/images/stay/";
    } else if (velocity == "0.5"){
        path = "/home/harish/learning/depth/images/up/";
    } else if (velocity == "-0.5"){
        path = "/home/harish/learning/depth/images/down/";
    }
    std::string frame = "frame";
    unsigned long long current = std::chrono::duration_cast<std::chrono::milliseconds>(
                                        std::chrono::system_clock::now().time_since_epoch()).count();
    std::string filename = path + frame + to_string(current) + ".jpg";
    cv::imwrite(filename, rgb_img);
//    string range_str = to_string(lidar->range);
//    ofstream myfile;
//    myfile.open ("/home/harish/learning/data/example.txt", std::ios_base::app);
//    myfile << filename + ", " + range_str + ", " + velocity + "\n";
//    myfile.close();

    return;
}

void depthToCV8UC1(const cv::Mat& float_img, cv::Mat& mono8_img){
    // Process images
    if(mono8_img.rows != float_img.rows || mono8_img.cols != float_img.cols){
        mono8_img = cv::Mat(float_img.size(), CV_8UC1);
    }
    cv::convertScaleAbs(float_img, mono8_img, 255.0/300.0, 0.0);
}


void depth_callback(const ImageConstPtr& original_image, const velocityConstPtr& vel)
{
    // TODO: Use lidar to better approximate the ground beneath
    i++;
    ROS_INFO("count: %s", std::to_string(i).c_str());
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(original_image);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    string velocity = vel->velocity;
    std::string path = "";

    // Read depth image
    cv::Mat depth_float_img = cv_ptr->image;
    cv::Mat depth_mono8_img;
    // remove NaNs
    cv::patchNaNs(depth_float_img, 1000.0);
    depthToCV8UC1(depth_float_img, depth_mono8_img);


    if (velocity == "0"){
        path = "/home/harish/learning/depth/images/stay/";
    } else if (velocity == "0.5"){
        path = "/home/harish/learning/depth/images/up/";
    } else if (velocity == "-0.5"){
        path = "/home/harish/learning/depth/images/down/";
    }
    std::string frame = "frame";
    unsigned long long current = std::chrono::duration_cast<std::chrono::milliseconds>(
                                        std::chrono::system_clock::now().time_since_epoch()).count();
    std::string filename = path + frame + to_string(current) + ".jpg";
    cv::imwrite(filename, depth_mono8_img);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", depth_mono8_img).toImageMsg();
    pub.publish(msg);
    return;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_node");

  ros::NodeHandle nh;
  // message_filters::Subscriber<Image> image_sub(nh, "/camera/rgb/image_raw", 1); // for RGB image capture
  message_filters::Subscriber<Image> image_sub(nh, "/camera/depth/image_raw", 1); // for Depth image capture
//  message_filters::Subscriber<Range> info_sub(nh, "/iris_odom/range_down", 1); // for range sensor capture
  message_filters::Subscriber<velocity> vel_sub(nh, "/OffboardControl/velocity_command", 1);
  pub = nh.advertise<sensor_msgs::Image>("/my_test/camera/image", 10);

  typedef sync_policies::ApproximateTime<Image, velocity> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), image_sub, vel_sub);
  sync.registerCallback(boost::bind(&depth_callback, _1, _2));
  ros::spin();

  return 0;
}