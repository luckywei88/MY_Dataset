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
#include <chrono>

using namespace std;
using namespace cv;

ros::Publisher rgbpub,deppub;

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
		vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
	ifstream fAssociation;
	fAssociation.open(strAssociationFilename.c_str());
	while(!fAssociation.eof())
	{
		string s;
		getline(fAssociation,s);
		if(!s.empty())
		{
			stringstream ss;
			ss << s;
			double t;
			string sRGB, sD;
			ss >> t;
			vTimestamps.push_back(t);
			ss >> sRGB;
			vstrImageFilenamesRGB.push_back(sRGB);
			ss >> t;
			ss >> sD;
			vstrImageFilenamesD.push_back(sD);

		}
	}
}

void Send_Image(cv::Mat &rgbm, cv::Mat &depm, double &timestamps)
{
	cv_bridge::CvImage rgb,dep;
	sensor_msgs::Image rgb_image,dep_image;

	rgb.encoding=sensor_msgs::image_encodings::BGR8;
	rgb.image=rgbm;
	rgb.toImageMsg(rgb_image);

	dep.encoding="16UC1";
	dep.image=depm;
	dep.toImageMsg(dep_image);

	rgbpub.publish(rgb_image);
	deppub.publish(dep_image);
}

int main(int argc,char **argv)
{
	string sequence="lucky";
	string dataset="lucky";
	string dir="/home/lucky/dataset/";

	ros::init(argc,argv,"send_dataset");
	ros::start();
	ros::NodeHandle nh;
	ros::NodeHandle n_private("~");
	n_private.param("dataset",dataset,dataset);
	n_private.param("sequence",sequence,sequence);
	rgbpub=nh.advertise<sensor_msgs::Image>("/rgb",1);
	deppub=nh.advertise<sensor_msgs::Image>("/depth",1);
	
	dir=dir+dataset+"/"+sequence;
	string rgbdir,depdir;
	rgbdir=dir+"/rgb";
	depdir=dir+"/depth";
	string assofile;
	assofile=dir+"/associate.txt";

	vector<string> vstrImageFilenamesRGB;
	vector<string> vstrImageFilenamesD;
	vector<double> vTimestamps; 
	LoadImages(assofile, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

	
	int nImages = vstrImageFilenamesRGB.size();
	if(vstrImageFilenamesRGB.empty())
	{
		cerr << endl << "No images found in provided path." << endl;
		return 1;
	}
	else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
	{
		cerr << endl << "Different number of images for rgb and depth." << endl;
		return 1;
	}

	cv::Mat imRGB, imD;
	for(int ni=0; ni<nImages; ni++)
	{
		// Read image and depthmap from file
		imRGB = cv::imread(dir+"/"+vstrImageFilenamesRGB[ni],CV_LOAD_IMAGE_UNCHANGED);
		imD = cv::imread(dir+"/"+vstrImageFilenamesD[ni],CV_LOAD_IMAGE_UNCHANGED);
		double tframe = vTimestamps[ni];

		if(imRGB.empty())
		{
			cerr << endl << "Failed to load image at: "
				<< dir << "/" << vstrImageFilenamesRGB[ni] << endl;
			return 1;
		}

#ifdef COMPILEDWITHC11
		std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
		std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif
		// send to ROS
		Send_Image(imRGB,imD,tframe);

#ifdef COMPILEDWITHC11
		std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
		std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

		double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

		// Wait to load the next frame
		double T=0;
		if(ni<nImages-1)
			T = vTimestamps[ni+1]-tframe;
		else if(ni>0)
			T = tframe-vTimestamps[ni-1];

		if(ttrack<T)
			usleep((T-ttrack)*1e6);
		
		ros::spinOnce();
	}


	ros::shutdown();
	return 0;
}
