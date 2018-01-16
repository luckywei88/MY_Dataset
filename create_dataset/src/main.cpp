#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <sys/stat.h>
#include <sys/types.h>
using namespace std;
using namespace cv;

string dir;
fstream depio,rgbio;

void callback(const sensor_msgs::ImageConstPtr& rgb, const sensor_msgs::ImageConstPtr& dep)
{
    ros::Time rgb_t=rgb->header.stamp;
    ros::Time dep_t=dep->header.stamp;
    cv_bridge::CvImagePtr rgb_cv,dep_cv;
    cv::Mat rgb_m,dep_m;
    try
    {
	rgb_cv=cv_bridge::toCvCopy(rgb,sensor_msgs::image_encodings::BGR8);
	rgb_m=rgb_cv->image;
	dep_cv=cv_bridge::toCvCopy(dep,"16UC1");
	dep_m=dep_cv->image;
    }
    catch(cv_bridge::Exception& e)
    {
	ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    string rgbbuf,depbuf;
    stringstream rgbs; 
    stringstream deps;
    rgbs<<dir<<"/rgb/"<<rgb_t<<".png";
    deps<<dir<<"/depth/"<<dep_t<<".png";
    rgbs>>rgbbuf;
    deps>>depbuf;
    imshow("rgb",rgb_m);
    cvWaitKey(20);
	rgbio<<rgb_t<<" "<<"rgb/"<<rgb_t<<".png"<<endl;
	depio<<dep_t<<" "<<"depth/"<<dep_t<<".png"<<endl;
	imwrite(rgbbuf,rgb_m);
	/*
	IplImage* depthImage=new IplImage(dep_m);
	IplImage* img = cvCreateImage(cvSize(depthImage->width, depthImage->height), IPL_DEPTH_8U, 1);
	char* depthData = (char*)depthImage->imageData;
	for (int i = 0, j = 0; i < depthImage->imageSize; i+=2, ++j)
	        img->imageData[j] = (((depthData[i + 1] << 5) + (depthData[i] >> 3)) / 4096.0);
		*/
	imwrite(depbuf,dep_m);
}

int main(int argc,char **argv)
{
    ROS_ERROR("start");
    string dataset="lucky";
    dir="/home/lucky/dataset/my_dataset/";
    ros::init(argc,argv,"dataset");
    ros::start();
    ros::NodeHandle nh;
    ros::NodeHandle n_private("~");
    n_private.param("dataset",dataset,dataset);
    dir=dir+dataset;
    cout<<dir<<endl;
    string rgbdir,depdir;
    rgbdir=dir+"/rgb";
    depdir=dir+"/depth";
    mkdir(dir.c_str(),S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
    mkdir(rgbdir.c_str(),S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
    mkdir(depdir.c_str(),S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
    string rgbfile,depfile,groundfile;
    rgbfile=dir+"/rgb.txt";
    depfile=dir+"/depth.txt";
    rgbio.open(rgbfile.c_str(),ofstream::out|ofstream::trunc);
    depio.open(depfile.c_str(),ofstream::out|ofstream::trunc);
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh,"/rgb",2);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh,"/depth",2);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(callback,_1,_2));
    ros::spin();
    rgbio.close();
    depio.close();
    ROS_ERROR("close");
    ros::shutdown();
    return 0;
}
