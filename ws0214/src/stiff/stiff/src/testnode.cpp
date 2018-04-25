// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

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
#include "postprocess.h"
#include "sensor_driver_msgs/stiffwater.h"




int main(int argc, char** argv)
{
  ros::init(argc, argv, "stiff_detect");
  ros::NodeHandle nh;

  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  FLAGS_colorlogtostderr = true; //设置输出到屏幕的日志显示相应颜色
  std::string corectionfiles="/home/zx/CODE/ws0214/src/sensor_driver/sensor_driver/config/64S3db.xml";
  ros::Publisher pubStiffwaterOgm;
  pubStiffwaterOgm=nh.advertise<sensor_driver_msgs::stiffwater> ("stiffwaterogm",20);
  PostProcess postprocess(nh,corectionfiles);
  ros::Rate loop_rate(20);
  while(ros::ok)
  {
	  sensor_driver_msgs::stiffwater msg_send;
	  msg_send.header.stamp=postprocess.gettime();
	  msg_send.header.frame_id="stiffwater";
	  msg_send.ogmheight=postprocess.getOGM()->ogmheight_cell;
	  msg_send.ogmwidth=postprocess.getOGM()->ogmwidth_cell;
	  msg_send.resolution=postprocess.getOGM()->ogmresolution;
	  msg_send.vehicle_x=100;
	  msg_send.vehicle_y=100;
//	  msg_send.ogm_data=postprocess.getOGM();
//	  std::cout<<"..........."<<msg_send.vehicle_x<<std::endl;
//	  std::cout<<"..........."<<msg_send.header.stamp<<std::endl;

	  for(int i=0;i<msg_send.ogmheight*msg_send.ogmwidth;i++)
	  {
//		  std::cout<<postprocess.getOGM()->ogm[i]<<" ";
		  msg_send.data.push_back(postprocess.getOGM()->ogm[i]);
	  }


	  pubStiffwaterOgm.publish(msg_send);
//	  memset(postprocess.getOGM(),'a',10000*sizeof(char));
	  loop_rate.sleep();
	  ros::spinOnce();
  }


//  ros::spin();
//
//  std::cout<<">>>>>>>>>>>>>>>>>>>>><<"<<std::endl;
  return 0;
}
