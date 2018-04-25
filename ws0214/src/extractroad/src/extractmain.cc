
#include <cmath>
#include "transform/rigid_transform.h"
#include <common/common.h>
#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <fstream>
#include <list>
#include <glog/logging.h>
#include <string.h>

#include "velodyne/HDL32Structure.h"
#include "util/boostudp/boostudp.h"
#include "common/blocking_queue.h"
#include "common/make_unique.h"
#include "iv_slam_ros_msgs/TraversibleArea.h"//!!!!!!!!!!!!!!!!!!
#include "CalPredictPoint.hh"
#include <typeinfo>

CalPredictPoint extractroad;
_OGMDataStruct* OGMData_ptr=new _OGMDataStruct;
void callback(const iv_slam_ros_msgs::TraversibleArea& msg){
	unsigned char data[msg.width*msg.height];

	OGMData_ptr->lTimeStamp=msg.header.stamp;
//	cout<<"i should say something shouldn't i"<<endl;
	int i=0;
//	cout<<"tye ies  "<<typeid(msg.cells).name()<<endl;
	for(;i<msg.cells.size();i++){
//		cout<<">"<<endl;
		data[i]=msg.cells[i];

	}	//这里应该是有点问题的，传过来的大小应该是不确定的

	MSG msgmine;
	msgmine.stamp=msg.header.stamp;
	msgmine.width=msg.width;
	msgmine.height=msg.height;
	msgmine.triD_submap_pose_image_index_x=msg.triD_submap_pose_image_index_x;
	msgmine.triD_submap_pose_image_index_y=msg.triD_submap_pose_image_index_y;
	msgmine.cells=data;

	OGMData_ptr->m_OccupiedMap=data;
//	some output
//	cout<<"triD_submap_pose_image_index_x is  "<<msg.triD_submap_pose_image_index_x<<endl;
//	cout<<"triD_submap_pose_image_index_y is  "<<msg.triD_submap_pose_image_index_y<<endl;
//	cout<<"pose is "<<msg.triD_submap_pose.orientation.w<<" "<<msg.triD_submap_pose.orientation.x<<" "<<msg.triD_submap_pose.orientation.y<<" "<<msg.triD_submap_pose.orientation.z<<endl;
//	some output
//	visualization
//	char* windowname="testwindow";
//	cvNamedWindow(windowname,0);
//	IplImage *slopemat = cvCreateImage(cvSize(msg.width,msg.height),IPL_DEPTH_8U,3);
//	for(int i=0;i<msg.height;i++){
//		unsigned char* pdata = (unsigned char*)(slopemat->imageData + (msg.height-i-1)* slopemat->widthStep);
//		for(int j=0;j<msg.width;j++){
//		unsigned char val = OGMData_ptr->m_OccupiedMap[j + i*msg.width];
//		if(val==0){
////			cout<<"000000000";
//			pdata[3*j]=0;
//			pdata[3*j+1]=0;
//			pdata[3*j+2]=0;
//		}
//		else if(val==2){
//			pdata[3*j]=255;
//			pdata[3*j+1]=255;
//			pdata[3*j+2]=255;
//		}
//		else{
//			pdata[3*j]=0;
//			pdata[3*j+1]=255;
//			pdata[3*j+2]=0;
//		}
//		}
//	}
//	int height=msg.height-msg.triD_submap_pose_image_index_y,width=msg.triD_submap_pose_image_index_x;
//	cvCircle(slopemat,cvPoint(width,height),5,CvScalar(125,125,0),10);
//    cvShowImage(windowname,slopemat);
//    cvWaitKey(10);
//    cvReleaseImage(&slopemat);
    extractroad.setup(msgmine);
//    memset(slopemat,0,800*1000*sizeof(char));
//	visualization
//	cout<<"size is "<<msg.cells.size()<<endl;
//	cout<<"width is "<<msg.width<<endl;
//	cout<<"height is "<<msg.height<<endl;

//	extractroad.ExtratCenterPoint(OGMData_ptr);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "extractroad");
  ros::NodeHandle nh;

  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  FLAGS_colorlogtostderr = true; //设置输出到屏幕的日志显示相应颜色
//  CalPredictPoint extractroad(nh);
//  ros::Publisher pubStiffwaterOgm;
//  pubStiffwaterOgm=nh.advertise<sensor_driver_msgs::stiffwater> ("stiffwaterogm",20);

  cout<<"whatis going on "<<endl;
  ros::Subscriber ogmsub=nh.subscribe("traversible_area_topic",1,callback);
//  nodehandle_.subscribe<sensor_msgs::PointCloud2>
//  										 ("lidar_cloud_calibrated", 1, boost::bind(&PostProcess::laserCloudHandler,this,_1));
//  ros::Rate loop_rate(20);
//  while(ros::ok)
//  {
//	  sensor_driver_msgs::stiffwater msg_send;
//	  msg_send.header.stamp=postprocess.gettime();
//	  msg_send.header.frame_id="stiffwater";
//	  msg_send.ogmheight=postprocess.getOGM()->ogmheight_cell;
//	  msg_send.ogmwidth=postprocess.getOGM()->ogmwidth_cell;
//	  msg_send.resolution=postprocess.getOGM()->ogmresolution;
//	  msg_send.vehicle_x=100;
//	  msg_send.vehicle_y=100;
////	  msg_send.ogm_data=postprocess.getOGM();
////	  std::cout<<"..........."<<msg_send.vehicle_x<<std::endl;
////	  std::cout<<"..........."<<msg_send.header.stamp<<std::endl;
//
//	  for(int i=0;i<msg_send.ogmheight*msg_send.ogmwidth;i++)
//	  {
////		  std::cout<<postprocess.getOGM()->ogm[i]<<" ";
//		  msg_send.data.push_back(postprocess.getOGM()->ogm[i]);
//	  }
//
//
//	  pubStiffwaterOgm.publish(msg_send);
////	  memset(postprocess.getOGM(),'a',10000*sizeof(char));
//	  loop_rate.sleep();
//	  ros::spinOnce();
//  }


  ros::spin();
//
//  std::cout<<">>>>>>>>>>>>>>>>>>>>><<"<<std::endl;
  return 0;
}
