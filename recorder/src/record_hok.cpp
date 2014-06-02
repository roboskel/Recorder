#include <stdio.h>
#include <time.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <cmath>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/LaserScan.h>
#include <chrono>
#include <ctime>
#include <sensor_msgs/Joy.h>

//CV
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/LaserScan.h>
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/cv.h"
#include "cv.h"
#include <opencv2/imgproc/imgproc.hpp> 
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <cvaux.h>
#include <cxcore.h>
#include <highgui.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
//
   
#define NUM_READINGS 1128

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

float hok_read[NUM_READINGS] = {};
int REC = 0;
long int timer;
double seconds ,seconds_, seconds_mc ;

time_t t;
ros::Time hok_cur_time, hok_last_time;
ros::Time total_cur_time, total_time;
ofstream file, file2 ;
//laser_scan
std::stringstream sstream_hok ;
std::string hok_ts;
std::chrono::time_point<std::chrono::system_clock> p1;
//
//motor commands
std::stringstream sstream2;
std::string odom_ts;
std::chrono::time_point<std::chrono::system_clock> p2;
ros::Time mc_cur_time, mc_last_time;
//

//RGB IMAGE

long int timer_rgb ;
double seconds_rgb ;
std::chrono::time_point<std::chrono::system_clock> p3;
ros::Time rgb_cur_time, rgb_last_time;

static const char WINDOW[] = "Image window";
std::string path ="./rec/img_rgb/";
//std::string pathskel=".rec/skel.txt";
std::string png = ".png";
std::string path_ts;
std::stringstream sstream_rgb ;
//


class recorder
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_rgb_; //image subscriber

public:
recorder()
    : it_(nh_)
    {
        image_rgb_ = it_.subscribe("/camera/rgb/image_color", 1, &recorder::imageCb_rgb, this);
        //image_depth_ = it_.subscribe("/camera/depth/image",1,&recorder::imageCb_di,this);
    }

~recorder()
    {
        cv::destroyWindow(WINDOW);
    }

void imageCb_rgb(const sensor_msgs::ImageConstPtr& msg)
{
    //sensor_msgs::CvBridge bridge;//we need this object bridge for facilitating conversion from ros-img to opencv
    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
    try
    {
        //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv_ptr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat &img = cv_ptr->image;
    //cv::imshow("windowName", img);
    //IplImage* img = cv_ptr.imgMsgToCv(msg,"bgr8"); //image being converted from ros to opencv using cvbridge
    if (REC==1)
    {
        //ROS_INFO("IMAGE CALLBACK");
        rgb_cur_time = ros::Time::now();
        seconds_rgb=(rgb_cur_time.toSec())-(rgb_last_time.toSec());
        if(seconds_rgb>0.1)
        {
            p3 = std::chrono::system_clock::now();
            //sstream_rgb << path ;
            sstream_rgb << std::chrono::duration_cast<std::chrono::milliseconds>(
            p3.time_since_epoch()).count() ;
            /*
            path_ts = sstream_rgb.str();
            path_ts=path+path_ts+png;
            */
            sstream_rgb << png;
            //ros::Duration(0.01).sleep();
            string cstr = sstream_rgb.str();
            //cout<<sstream_rgb.str();
            if(!(cv::imwrite(cstr, cv_ptr->image)))
            {
                ROS_INFO("Can't Capture RGB Image\n");
                //ros::Duration(0.01).sleep();
            }
            else
            {
                ROS_INFO("RGB Image Captured\n");
                //ros::Duration(0.01).sleep();
            }
            rgb_last_time=rgb_cur_time;
            rgb_cur_time=ros::Time::now();
            path_ts="";
            sstream_rgb.str(std::string());
            sstream_rgb.clear();
            
        }
    }
    cvWaitKey(2);
    }
};






void hok_cb(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    std::vector<float> ranges = scan->ranges;
    if(REC==1)
    {
        //TIME INTERVAL STUFF
        total_cur_time=ros::Time::now();
        hok_cur_time = ros::Time::now();
        seconds=hok_cur_time.toSec()-hok_last_time.toSec();
        if(seconds>0.05)
        {
            //file<<"Time ";
            p1 = std::chrono::system_clock::now();
            sstream_hok << std::chrono::duration_cast<std::chrono::milliseconds>(
            p1.time_since_epoch()).count() ;
            hok_ts = sstream_hok.str();
            file<<hok_ts<<" ";
            for( std::vector<float>::size_type i = 0; i != ranges.size(); i++)
            {
                file<<ranges[i]<<" ";
            }
            file<<endl ;
            hok_ts.clear();
            sstream_hok.str(std::string());
            sstream_hok.clear();
            hok_last_time=hok_cur_time;
            hok_cur_time=ros::Time::now();
        }
    }
    
}


void chatterCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    int start = msg->data[11]; //START gia na 3ekinhsei to record
    int cancel = msg->data[4]; //X gia na stamathsei
    //printf("Press Start to Start Recording \n");
    if (start==1)
    {
        printf("Started Recording Hok\n");
        REC = 1;
        ros::Duration(1).sleep();
    }
    else if ((REC==1)&&(cancel==1))
    {
        REC = 0;
        printf("Stopped Recording Hok \n");
        ros::shutdown();
    }
}

void callback(const std_msgs::Float64::ConstPtr& msg)
{
    float start = msg->data; //START gia na 3ekinhsei to record
    //X gia na stamathsei
    //printf("Press Start to Start Recording \n");
    if (start==1)
    {
        printf("Started Recording Hok\n");
        //printf("Started Recording Skeleton\n");
        total_time = ros::Time::now();
        REC = 1;
        ros::Duration(1).sleep();
    }
    else if ((REC==1)&&(start==0))
    {
        printf("Stopped Recording Hok\n");
        REC = 0;
        //printf("Stopped Recording Skeleton \n");
        ros::shutdown();
    }
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    if(msg->buttons[1]==1)
    {
        if (REC == 0)
        {
            ROS_INFO("lelelle");
            REC = 1;
        }
        else
        {
            REC = 0;
        }
    }
    if(msg->buttons[0]==1)
    {
        ros::shutdown();
    }
}

void callback2(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
    int RM = msg->data[0];
    int LM = msg->data[1];
    if(REC==1)
    {
        //TIME INTERVAL STUFF
        mc_cur_time = ros::Time::now();
        seconds_mc = mc_cur_time.toSec()-mc_last_time.toSec();
        if(seconds_mc>0.05)
        {
            //file2<<"Time ";
            p2 = std::chrono::system_clock::now();
            sstream2 << std::chrono::duration_cast<std::chrono::milliseconds>(
            p2.time_since_epoch()).count() ;
            odom_ts = sstream2.str();
            file2<<odom_ts<<" ";
            file2<<RM<<" "<<LM<<" ";
            file2<<endl ;
            odom_ts.clear();
            sstream2.str(std::string());
            sstream2.clear();
            mc_last_time=mc_cur_time;
            mc_cur_time=ros::Time::now();
        }
    }
}

int main(int argc, char** argv)
{
    file.open("./_laser.data", ios::out | ios::binary);
    file2.open("./_odometry.data", ios::out | ios::binary);
    //cvNamedWindow("windowName", CV_WINDOW_AUTOSIZE);
    //cvStartWindowThread();
    ros::init(argc, argv, "recorder");
    ros::NodeHandle n;
    ros::Subscriber gp_in, hok_in, mc_in, joy_sub;
    ros::Publisher turn_off;
    mc_last_time = ros::Time::now();
    hok_last_time = ros::Time::now();
    rgb_last_time = ros::Time::now();
    recorder ic;
    printf("Waiting gamepad Input\n");
    gp_in = n.subscribe("/coms",1,callback);
    joy_sub = n.subscribe("/joy", 100, joyCallback);
    //mc_in = n.subscribe("xar_odom", 1, callback2);
    //hok_in = n.subscribe("/scan",1,hok_cb);
    while (ros::ok())
    {
        ros::spin();
    }
    file.close();
    file2.close();
    //cvDestroyWindow("windowName");
    return 0;
}
