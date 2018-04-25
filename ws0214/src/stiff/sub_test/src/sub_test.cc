#include<ros/ros.h>
#include<iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
//#include <iostream>
//#include <stdio.h>
#include<string>
#include "sensor_driver_msgs/stiffwater.h"

using namespace std;


void call_back(const sensor_driver_msgs::stiffwater& msg)
{
//	cout<<"timestamp....."<<msg.header.stamp<<endl;
//	cout<<"frame_id......"<<msg.header.frame_id<<endl;
//	cout<<"vehicle_x......"<<msg.vehicle_x<<endl;
//	cout<<">>>>>>>>>>>>>>>>>"<<endl;
//	cout<<"ogm_height......"<<msg.ogmheight<<endl;
//	ROS_INFO("SDFSDFSDFSDFSDFSDFF");
		ros::Rate loop_rate(20);
	char* windowname="testwindow";
	cvNamedWindow(windowname,0);
	IplImage *slopemat = cvCreateImage(cvSize(msg.ogmwidth,msg.ogmheight),IPL_DEPTH_8U,1);
	cvZero(slopemat);
	for(int i=0;i<msg.ogmheight;i++)
	{
		unsigned char* pdata = (unsigned char*)(slopemat->imageData + i* slopemat->widthStep);//这里 有一个坐标系变换的过程
		for(int j=0;j<msg.ogmwidth;j++)
		{
//			cout<<"vehicle_x......"<<msg.vehicle_x<<endl;
			if(msg.data[(msg.ogmheight-i-1)*msg.ogmwidth+j]==5)
				pdata[j]=255;

		}

	}
//		cout<<"ogm_height......"<<msg.ogmheight<<endl;
//		cout<<"ogm_height......"<<msg.ogmwidth<<endl;
//		cout<<"whatsldkfa;ldkffa;lkkdf"<<endl;
	cvShowImage(windowname,slopemat);
	cvWaitKey(10);
	cvReleaseImage(&slopemat);
		loop_rate.sleep();
}

int main(int argc,char **argv)
{


    ros::init(argc,argv,"sub_test");
    ros::NodeHandle nh;
    ros::Subscriber sub=nh.subscribe("stiffwaterogm",2,call_back);

    ros::spin();
//    return 0;


}
