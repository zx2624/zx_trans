#include <common/common.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <ros/console.h>
#include <ros/package.h>
#include <glog/logging.h>
#include <sstream>
#include <fstream>

#include <pcl/common/transforms.h>
#include <boost/algorithm/string.hpp>
#include <boost/timer.hpp>
#include <boost/math/special_functions/round.hpp>
#include "boost/asio.hpp"
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/function.hpp>
#include <boost/date_time.hpp>
#include <boost/lambda/lambda.hpp>
#include <opencv2/opencv.hpp>
#include "util/xmlconf/xmlconf.h"
#include "rslidar/myhdl_grabber.h"
#include "util/playback/iv_data_playback.h"
#include "sensor_driver_msgs/startconfig.h"
typedef pcl::PointCloud<pcl::PointXYZI> Cloud;
typedef Cloud::ConstPtr CloudConstPtr;

struct CalibrationValue{
    double alfa ;
    double beta;
    double gama ;
    double x_offset;
    double y_offset ;
    double z_offset ;

};

struct CloudLimit{
  double angle_min;
  double angle_max;
};


class RsLidardata{
public:
  RsLidardata(const std::string& configstr):
    configstr_(configstr)
    , hdl_read_pcd_thread_ (NULL)
    , cloud_r_(new Cloud)
    , cloud_l_(new Cloud)
  {
    inited_ = false;
    init();
  };
  ~RsLidardata()
  {
    close();
  }

  void close ()
  {

    cloud_connection_r_.disconnect ();
    cloud_connection_l_.disconnect ();

    if(replay_ != 2)
      {
	grabber16_r_->stop();
	grabber16_l_->stop();
      }

  }

  void cloud_callback_r (const CloudConstPtr& cloud)
  {
      cloud_r_ = cloud;
      updated_ = true;
      stampr_ = cloud_r_->header.stamp/1000000.0;
  }

  void cloud_callback_l (const CloudConstPtr& cloud)
  {
      cloud_l_ = cloud;
      updated_ = true;
      stampl_ = cloud_l_->header.stamp/1000000.0;
  }

  void Record()
  {
    static int index=0;
    static std::string filename;
    if(cloud_r_ && cloud_l_)
    {
      playback_.BeginSaveLine();
      filename.clear();
      filename=playback_.MakeRecordFileName(index,".pcd");
      playback_<<2<<1<<playback_.SysTime(stampr_)<<filename;

      filename=playback_.GetRecordPath()+filename;
      pcl::io::savePCDFile(filename, *cloud_r_,true);
      index++;
      filename.clear();
      filename=playback_.MakeRecordFileName(index,".pcd");
      playback_<<2<<playback_.SysTime(stampl_)<<filename;
      filename=playback_.GetRecordPath()+filename;
      pcl::io::savePCDFile(filename, *cloud_l_,true);
      index++;
      playback_.EndSaveLine();
    }
    else if(cloud_r_)
    {
      playback_.BeginSaveLine();
      filename.clear();
      filename=playback_.MakeRecordFileName(index,".pcd");
      playback_<<1<<1<<stampr_<<filename;



      filename=playback_.GetRecordPath()+filename;
      pcl::io::savePCDFile(filename, *cloud_r_,true);
      index++;
      playback_.EndSaveLine();
    }
    else if(cloud_l_)
    {
      playback_.BeginSaveLine();
      filename.clear();
      filename=playback_.MakeRecordFileName(index,".pcd");
      playback_<<1<<2<<stampl_<<filename;

      filename=playback_.GetRecordPath()+filename;
      pcl::io::savePCDFile(filename, *cloud_l_,true);
      index++;
      playback_.EndSaveLine();
    }
  }

  void getdatafrompcd()
  {
    std::string filename;

    LOG(INFO)<<"getdatafrompcd";
    while(inited_&&playback_.BeginLoadLine()==true)
      {


	int count=0;
	playback_>>count;
	LOG(INFO)<<"count:"<<count;
	for(int i=0;i<count;i++)
	{
	    int num = 0;
	    playback_>>num;
	    double tempstamp;
	    if(num == 1)
	    {
		pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>);
		playback_>>tempstamp>>filename;
		std::cout<<filename<<std::endl;
		filename=playback_.GetPlaybackPath()+filename;
		pcl::io::loadPCDFile(filename,*laserCloud);
		pcl_conversions::toPCL(ros::Time::now(), laserCloud->header.stamp);//us
		cloud_callback_r(laserCloud);
//		    laserCloud.reset (new pcl::PointCloud<pcl::PointXYZI> ());
	    }
	    else if(num == 2)
	    {
		pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>);
		playback_>>tempstamp>>filename;
		std::cout<<filename<<std::endl;
		filename=playback_.GetPlaybackPath()+filename;
		pcl::io::loadPCDFile(filename,*laserCloud);
		pcl_conversions::toPCL(ros::Time::now(), laserCloud->header.stamp);//us
		cloud_callback_l(laserCloud);
//		    laserCloud.reset (new pcl::PointCloud<pcl::PointXYZI> ());
	    }
	}
	playback_.EndLoadLine();
      }
  }
  bool init()
  {
    lasernum_ = 16;
    if(!xmlconfig_.Parse(configstr_.c_str(), "iv_lidar16"))
    {
  	  std::cout<<"iv_lidar  is not exist in config xml  file"<<std::endl;
    }
    else
      {
	ConfigParam();
	float minDistanceThreshold = 1;
	float maxDistanceThreshold = 100;
	float velodyneheight = 1.2;

	float rigid_heightdiffthreshold = 0.2;

	playback_.Setup(xmlconfig_);
	if(playback_.PlaybackIsOn())
	{
		replay_=playback_mode;
	}
	else
	replay_=0;
	//replay_=1;

	if(replay_==1 || replay_==2)
	{
		boost::asio::ip::address listen_ip = boost::asio::ip::address::from_string(hdlIP_);
		unsigned short  listen_port = hdlPortr_;
		unsigned short listen_port2 = hdlPortl_;

		if(replay_==1)
		{
			if(!xmlconfig_.GetModuleParam("pcapFile_",pcapFile_))
			{
				std::cout<<"pcapFile_ is incorrect"<<std::endl;
			}
			else
			std::cout<<"pcapFile_ is "<<pcapFile_<<std::endl;

		}
		else
		  pcapFile_ = "";
		//对于第一个雷达的赋值，端口号和本地Ip
		grabber16_r_=new pcl::MyHDLGrabber(calibrationdir_r_,pcapFile_,lasernum_,true);
		grabber16_r_->setMaximumDistanceThreshold(maxDistanceThreshold);
		grabber16_r_->setMinimumDistanceThreshold(minDistanceThreshold);
		grabber16_r_->filterPackets(listen_ip,listen_port);
		//对于第二个雷达的赋值，端口号和本地ip
		grabber16_l_=new pcl::MyHDLGrabber(calibrationdir_l_,pcapFile_,lasernum_,false);
		grabber16_l_->setMaximumDistanceThreshold(maxDistanceThreshold);
		grabber16_l_->setMinimumDistanceThreshold(minDistanceThreshold);
		grabber16_l_->filterPackets(listen_ip,listen_port2);

	}
	else
	{

		std::cout<<"realtime start"<<std::endl;
		boost::asio::ip::address listen_ip = boost::asio::ip::address::from_string(hdlIP_);
		unsigned short  listen_portr = hdlPortr_;
		unsigned short  listen_portl = hdlPortl_;
		grabber16_r_=new pcl::MyHDLGrabber(listen_ip, listen_portr, calibrationdir_r_, lasernum_,false);
		grabber16_r_->setMaximumDistanceThreshold(maxDistanceThreshold);
		grabber16_r_->setMinimumDistanceThreshold(minDistanceThreshold);
		std::vector<functionPtr> tempfuc;
		tempfuc.push_back(grabber16_r_->gettrigger_cb());
		grabber16_l_=new pcl::MyHDLGrabber(listen_ip, listen_portl, calibrationdir_l_, lasernum_,true,&tempfuc);
		grabber16_l_->setMaximumDistanceThreshold(maxDistanceThreshold);
		grabber16_l_->setMinimumDistanceThreshold(minDistanceThreshold);

	}
	get_transform_matrix(calibvaluer_,transform_matrix_calibration_R2V_);
	get_transform_matrix(calibvaluel_,transform_matrix_calibration_L2V_);

	boost::function<void (const pcl::PointCloud<pcl::PointXYZI>::ConstPtr&)> cloud_cb_r
	    = boost::bind(&RsLidardata::cloud_callback_r,this,_1);
	cloud_connection_r_ = grabber16_r_->registerCallback(cloud_cb_r);
	boost::function<void (const pcl::PointCloud<pcl::PointXYZI>::ConstPtr&)> cloud_cb_l
	    = boost::bind(&RsLidardata::cloud_callback_l,this,_1);
	cloud_connection_l_ = grabber16_l_->registerCallback(cloud_cb_l);
        updated_ = false;
	inited_ = true;
        if(replay_ != 2)
          {
            grabber16_r_->start ();
            grabber16_l_->start ();
          }
        else
          {
            hdl_read_pcd_thread_ = new boost::thread (boost::bind (&RsLidardata::getdatafrompcd, this));
          }


	return true;
      }
    return false;
  }
  void ConfigParam()
  {
  	bool autodriving;

	if(!xmlconfig_.GetModuleParam("calibrationdir_r",calibrationdir_r_))
	{
		std::cout<<"calibrationdir_r_ is incorrect"<<std::endl;
	}
	else
	std::cout<<"calibrationdir_r_ is "<<calibrationdir_r_<<std::endl;

	if(!xmlconfig_.GetModuleParam("calibrationdir_l",calibrationdir_l_))
	{
		std::cout<<"calibrationdir_l_ is incorrect"<<std::endl;
	}
	else
	std::cout<<"calibrationdir_l_ is "<<calibrationdir_l_<<std::endl;

  	//对于config.xml里面的参数的赋值
  	if(!xmlconfig_.GetModuleParam("port_r",hdlPortr_))
	{
		std::cout<<"port num is incorrect"<<std::endl;
	}
	if(!xmlconfig_.GetModuleParam("port_l",hdlPortl_))
	{
		std::cout<<"port num is incorrect"<<std::endl;
	}


	if(!xmlconfig_.GetModuleParam("lidar16_ip",hdlIP_))
	{
		std::cout<<"ip num is incorrect"<<std::endl;
	}
	else
	std::cout<<"ip num is "<<hdlIP_<<std::endl;

	if(!xmlconfig_.GetModuleParam("playback_on",playback_on_))
	{
		std::cout<<"playback_on_ is incorrect"<<std::endl;
	}
	else
	std::cout<<"playback_on_ is "<<playback_on_<<std::endl;


	if(!xmlconfig_.GetModuleParam("playback_mode",playback_mode))
	{
		std::cout<<"playback_mode is incorrect"<<std::endl;
	}
	else
	std::cout<<"playback_mode is "<<playback_mode<<std::endl;


	if(!xmlconfig_.GetModuleParam("alfa_r",calibvaluer_.alfa))
	{
		std::cout<<"alfa_r is incorrect"<<std::endl;
	}
	else
	std::cout<<"alfa_r is "<<calibvaluer_.alfa<<std::endl;

	if(!xmlconfig_.GetModuleParam("beta_r",calibvaluer_.beta))
	{
		std::cout<<"beta_r is incorrect"<<std::endl;
	}
	else
	std::cout<<"beta_r is "<<calibvaluer_.beta<<std::endl;

	if(!xmlconfig_.GetModuleParam("gama_r",calibvaluer_.gama))
	{
		std::cout<<"gama_r is incorrect"<<std::endl;
	}
	else
	std::cout<<"gama_r is "<<calibvaluer_.gama<<std::endl;

	if(!xmlconfig_.GetModuleParam("x_offset_r",calibvaluer_.x_offset))
	{
		std::cout<<"x_offset_r is incorrect"<<std::endl;
	}
	else
	std::cout<<"x_offset_r is "<<calibvaluer_.x_offset<<std::endl;

	if(!xmlconfig_.GetModuleParam("y_offset_r",calibvaluer_.y_offset))
	{
		std::cout<<"y_offset_r is incorrect"<<std::endl;
	}
	else
	std::cout<<"y_offset_r is "<<calibvaluer_.y_offset<<std::endl;

	if(!xmlconfig_.GetModuleParam("z_offset_r",calibvaluer_.z_offset))
	{
		std::cout<<"z_offset_r is incorrect"<<std::endl;
	}
	else
	std::cout<<"z_offset_r is "<<calibvaluer_.z_offset<<std::endl;

	if(!xmlconfig_.GetModuleParam("alfa_l",calibvaluel_.alfa))
	{
		std::cout<<"alfa_l is incorrect"<<std::endl;
	}
	else
	std::cout<<"alfa_l is "<<calibvaluel_.alfa<<std::endl;

	if(!xmlconfig_.GetModuleParam("beta_l",calibvaluel_.beta))
	{
		std::cout<<"beta_l is incorrect"<<std::endl;
	}
	else
	std::cout<<"beta_l is "<<calibvaluel_.beta<<std::endl;

	if(!xmlconfig_.GetModuleParam("gama_l",calibvaluel_.gama))
	{
		std::cout<<"gama_l is incorrect"<<std::endl;
	}
	else
	std::cout<<"gama_l is "<<calibvaluel_.gama<<std::endl;

	if(!xmlconfig_.GetModuleParam("x_offset_l",calibvaluel_.x_offset))
	{
		std::cout<<"x_offset_l is incorrect"<<std::endl;
	}
	else
	std::cout<<"x_offset_l is "<<calibvaluel_.x_offset<<std::endl;

	if(!xmlconfig_.GetModuleParam("y_offset_l",calibvaluel_.y_offset))
	{
		std::cout<<"y_offset_l is incorrect"<<std::endl;
	}
	else
	std::cout<<"y_offset_l is "<<calibvaluel_.y_offset<<std::endl;

	if(!xmlconfig_.GetModuleParam("z_offset_l",calibvaluel_.z_offset))
	{
		std::cout<<"z_offset_l is incorrect"<<std::endl;
	}
	else
	std::cout<<"z_offset_l is "<<calibvaluel_.z_offset<<std::endl;

	if(!xmlconfig_.GetModuleParam("angle_min_r",cloudlimit_r_.angle_min))
	{
		std::cout<<"angle_min_r is incorrect"<<std::endl;
	}
	else
	std::cout<<"angle_min_r is "<<cloudlimit_r_.angle_min<<std::endl;

	if(!xmlconfig_.GetModuleParam("angle_min_l",cloudlimit_l_.angle_min))
	{
		std::cout<<"angle_min_l is incorrect"<<std::endl;
	}
	else
	std::cout<<"angle_min_l is "<<cloudlimit_l_.angle_min<<std::endl;

	if(!xmlconfig_.GetModuleParam("angle_max_r",cloudlimit_r_.angle_max))
	{
		std::cout<<"angle_max_r is incorrect"<<std::endl;
	}
	else
	std::cout<<"angle_max_r is "<<cloudlimit_r_.angle_max<<std::endl;

	if(!xmlconfig_.GetModuleParam("angle_max_l",cloudlimit_l_.angle_max))
	{
		std::cout<<"angle_max_l is incorrect"<<std::endl;
	}
	else
	std::cout<<"angle_max_l is "<<cloudlimit_l_.angle_max<<std::endl;

  }
  static void get_transform_matrix(const CalibrationValue& calibvalue,Eigen::Matrix4f& transform_matrix_calibration)
  {

  	double alfa = calibvalue.alfa*M_PI/180;
	double beta = calibvalue.beta*M_PI/180;
	double gama = calibvalue.gama*M_PI/180;//-164

	double x_offset = calibvalue.x_offset;
	double y_offset = calibvalue.y_offset;
	double z_offset = calibvalue.z_offset;


      transform_matrix_calibration = Eigen::Matrix4f::Identity();

      transform_matrix_calibration(0,0) = cos(beta)*cos(gama);
      transform_matrix_calibration(0,1) = sin(alfa)*sin(beta)*cos(gama) - cos(alfa)*sin(gama);
      transform_matrix_calibration(0,2) = cos(alfa)*sin(beta)*cos(gama) + sin(alfa)*sin(gama);
      transform_matrix_calibration(0,3) = x_offset;
      transform_matrix_calibration(1,0) = cos(beta)*sin(gama);
      transform_matrix_calibration(1,1) = sin(alfa)*sin(beta)*sin(gama) + cos(alfa)*cos(gama);
      transform_matrix_calibration(1,2) = cos(alfa)*sin(beta)*sin(gama) - sin(alfa)*cos(gama);
      transform_matrix_calibration(1,3) = y_offset;
      transform_matrix_calibration(2,0) = -sin(beta);
      transform_matrix_calibration(2,1) = sin(alfa)*cos(beta);
      transform_matrix_calibration(2,2) = cos(alfa)*cos(beta);
      transform_matrix_calibration(2,3) = z_offset;
      transform_matrix_calibration(3,0) = 0;
      transform_matrix_calibration(3,1) = 0;
      transform_matrix_calibration(3,2) = 0;
      transform_matrix_calibration(3,3) = 1.0;
  }



  static void reordercloud(Cloud& pointcloud,pcl::LaserData* indexmaptable,int  lasernum)
  {
    const Cloud tempcloud = pointcloud;
    for(int laser_j=0 ; laser_j<lasernum ;laser_j++)
	{
	  int oriindex_j=indexmaptable[laser_j].number;
	  for(int i=0;i<tempcloud.size()/lasernum;i++)
	    {
	      int index = i*lasernum+laser_j;
	      int oriindex = i*lasernum+oriindex_j;
	      pointcloud.at(index) = tempcloud.at(oriindex);
	    }
	}
  }
  static void addcloudinfo(Cloud& pointcloud,pcl::LaserData* indexmaptable,int  lasernum)
  {
    Cloud tempcloud;
    tempcloud.resize(lasernum);
    for(int laser_j=0 ; laser_j<lasernum ;laser_j++)
	{
	  int oriindex_j=indexmaptable[laser_j].number;
	  tempcloud.at(oriindex_j).x = laser_j+0.1;
	  tempcloud[oriindex_j].y = indexmaptable[laser_j].angle;
	  tempcloud[oriindex_j].range = -0.2;
	  tempcloud[oriindex_j].passibility = 1.0;
	}
    pointcloud += tempcloud;
  }

  void mixcloud()
  {
    totalcloud_ = processcloud_r_;
    totalcloud_ += processcloud_l_;
    pcl::PointXYZI temppoint;
    temppoint.x = calibvaluer_.x_offset;
    temppoint.y = calibvaluer_.y_offset;
    temppoint.z = calibvaluer_.z_offset;
    temppoint.azimuth=16.5; //线数
    temppoint.range = 0.5; //第二个点云的起点
    temppoint.passibility = 1.0;
    totalcloud_.push_back(temppoint);

    temppoint.x = calibvaluel_.x_offset;
    temppoint.y = calibvaluel_.y_offset;
    temppoint.z = calibvaluel_.z_offset;
    temppoint.azimuth=16.5;//线数
    temppoint.range = processcloud_r_.size()+0.5; //第二个点云的起点
    temppoint.passibility = 1.0;
    totalcloud_.push_back(temppoint);
    totalcloud_.header.stamp = processcloud_r_.header.stamp > processcloud_l_.header.stamp?
	processcloud_r_.header.stamp:processcloud_l_.header.stamp;
  }


bool isvalid()
{
  return  updated_&&fabs(stampr_ - stampl_)<0.05;
}

void resetcloudstate()
{
  updated_ = false;
}

static void  regionGrow(cv::Mat src,cv::Mat &matDst, cv::Point2i pt, float th)
{

   cv::Point2i ptGrowing;                      //待生长点位置
   int nGrowLable = 0;                             //标记是否生长过
   float nSrcValue = 0;                              //生长起点灰度值
   float nCurValue = 0;                              //当前生长点灰度值

   //生长方向顺序数据
  // int DIR[28][2] = {{-1,0}, {1,0}, {-2,0}, {2,0}, {-3,0}, {3,0}, {-4,0}, {4,0},{-5,0}, {5,0} ,{-6,0}, {6,0} ,{-7,0}, {7,0} ,{-8,0}, {8,0},{-9,0}, {9,0},{-10,0}, {10,0},{-11,0}, {11,0},{0,1}, {0,-1},{-1,1}, {-1,-1},{1,1}, {1,-1}};  //由近到远进行搜索
   int DIR[10][2] = {{-1,0}, {1,0}, {-2,0}, {2,0}, {-3,0}, {3,0}, {-4,0}, {4,0}, {0,1}, {0,-1}};  //由近到远进行搜索


   nSrcValue = src.at<float>(pt);            //记录生长点的灰度值
   matDst.at<uchar>(pt)=0;
   if (nSrcValue == 0)
   {
	matDst.at<uchar>(pt)=255;
       return;
   }

       //分别对八个方向上的点进行生长
       for (int i = 0; i<10; ++i)
       {
	   ptGrowing.x = pt.x + DIR[i][0];
	   ptGrowing.y = pt.y + DIR[i][1];
	   //检查是否是边缘点
	   if (ptGrowing.x < 0 || ptGrowing.y < 0 || ptGrowing.x > (src.cols-1) || (ptGrowing.y > src.rows -1))
	       continue;


	       nCurValue = src.at<float>(ptGrowing);
/*
	       if (abs(nSrcValue - nCurValue) < th)
		{
		   matDst.at<uchar>(pt)=255;
		   return;
		}

*/
	       if(nCurValue == 0)
	       {
		   continue;
	       }

	       if(nSrcValue <= 2  )//在阈值范围内则生长
		  {
		    if (fabs(nSrcValue - nCurValue) < 0.2)
		     {
			matDst.at<uchar>(pt) += 1;
		     }

		     if(matDst.at<uchar>(pt)>=3)           //
		      {
			  matDst.at<uchar>(pt)=255;                 //标记为白色,空间上的非孤立
			  return;
		      }
		  }
	       if(nSrcValue > 2 && nSrcValue <= 5   )
		 {
		   if (fabs(nSrcValue - nCurValue) < 0.5)
		    {
		       matDst.at<uchar>(pt) += 1;
		    }

		    if(matDst.at<uchar>(pt)>=2)           //
		     {
			 matDst.at<uchar>(pt)=255;                 //标记为白色,空间上的非孤立
			 return;
		     }
		  }
	       if(nSrcValue >5)
		  {

		   if (fabs(nSrcValue - nCurValue) < 1)
		    {
		       matDst.at<uchar>(pt) += 1;
		    }

		    if(matDst.at<uchar>(pt)>=1)           //
		     {
			 matDst.at<uchar>(pt)=255;                 //标记为白色,空间上的非孤立
			 return;
		     }
		  }



      }

 }




static void removeOutlier(pcl::PointCloud<pcl::PointXYZI> pointcloud,pcl::LaserData* indexmaptable,int  lasernum)
 {
   int col_count = pointcloud.size()/lasernum;
   pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_filter(new pcl::PointCloud<pcl::PointXYZI>);
   pointcloud_filter->clear();
   pointcloud_filter->height = lasernum;
   pointcloud_filter->width = col_count;
   pointcloud_filter->is_dense = false;
   pointcloud_filter->resize(pointcloud_filter->height * pointcloud_filter->width);

   cv::Mat mat_depth = cv::Mat(cv::Size( col_count,lasernum), CV_32FC1, cv::Scalar(0));
   for(int i=0;i<col_count;i++)
   {
      for(int j=0;j<lasernum;j++)
      {
	    int oriindex_j=indexmaptable[j].number;
	    int oriindex = i*lasernum+oriindex_j;
	    if(pointcloud.points[oriindex].range<0)
	    {
		    pointcloud_filter->at(i, j) = pointcloud.points[oriindex];
		    mat_depth.at<float>(j , i) = 0;
	    }
	    else
	    {
		    pointcloud_filter->at(i, j) = pointcloud.points[oriindex];
		    mat_depth.at<float>(j , i) = pointcloud.points[oriindex].range;
	    }
      }
   }

    cv::Mat matMask = cv::Mat::zeros(mat_depth.size(), CV_8UC1);   //创建一个空白区域，填充为黑色
    //int removeNum =0 , nannum = 0;
    for(int row = 0; row < mat_depth.rows; row++)
      {
         for(int col = 0 ; col < mat_depth.cols ; col++ )
        {
           float d = mat_depth.at<float>(row,col);
           if( d > 10)
           {
               matMask.at<uchar>(row,col) = (uchar)255;
               continue;
           }
           //if(d == 0)
           //{
           //    nannum++;
           //}
           cv::Point2i pt(col,row);
           regionGrow(mat_depth,matMask,pt,1);
        }
     }
    //ROS_INFO("NAN: %d", nannum);
     for(int row = 0; row < matMask.rows; row++)
      {
         for(int col = 0 ; col < matMask.cols ; col++ )
         {
             if(matMask.at<uchar>(row,col) != 255)
             {
                //removeNum++;
        	 pointcloud_filter->at(col,row).x =0.1;
        	 pointcloud_filter->at(col,row).y =0.1;
        	 pointcloud_filter->at(col,row).z =0.1;
        	 pointcloud_filter->at(col,row).azimuth = -1000;
        	 pointcloud_filter->at(col,row).range = -0.1;
        	 pointcloud_filter->at(col,row).passibility = 1.0;
                //pointcloud.at(col,row).intensity= 255;
             }
             int index = row + col*matMask.rows;  //行列倒过来
             pointcloud.points[index] = pointcloud_filter->at(col,row);

         }
       }
 }

 void precomputecloud(Cloud& pointcloud,CalibrationValue& calibvalue)
 {
  for (int i = 0; i < pointcloud.points.size(); i++)
  {
      //range < 0.5 ��Ч

    double error=0.000001;
    if(fabs(pointcloud.points[i].x-0.1)<error&&fabs(pointcloud.points[i].y-0.1)<error&&fabs(pointcloud.points[i].z-0.1)<error)
    {
	    pointcloud.points[i].azimuth = -1000;
	    pointcloud.points[i].range = -0.1;
	    pointcloud.points[i].passibility = 1.0;
    }
      else
      {
	  double tempdistance=sqrt(pointcloud.points[i].x * pointcloud.points[i].x +
		    pointcloud.points[i].y * pointcloud.points[i].y );
	  pointcloud.points[i].range = tempdistance;
	  float azimuth=atan2(pointcloud.points[i].y,pointcloud.points[i].x)*180/M_PI;
	  azimuth = azimuth + calibvalue.gama;
	  while(azimuth > 360.0) azimuth -=360.0;
	  while(azimuth <=0) azimuth +=360.0;
	  pointcloud.points[i].azimuth=azimuth;

	  pointcloud.points[i].passibility = 1.0;
      }
  }
 }

 static void filtercloudonvehicle(Cloud& pointcloud,const CalibrationValue& calibvalue,const CloudLimit& limit)
 {
  for (int i = 0; i < pointcloud.points.size(); i++)
  {
      //range < 0.5 ��Ч

      if(pointcloud.points[i].range>0&&((pointcloud.points[i].x > -fabs(1.5) && pointcloud.points[i].x < fabs(1.5) &&
	  pointcloud.points[i].y < calibvalue.y_offset+0.2 && pointcloud.points[i].y > -2.5)
	  ))
      {
	  pointcloud.points[i].range =- 0.01;
      }

      if((pointcloud.points[i].azimuth>limit.angle_min&&pointcloud.points[i].azimuth<limit.angle_max)
	  ||(pointcloud.points[i].azimuth - 360>limit.angle_min && pointcloud.points[i].azimuth - 360<limit.angle_max))
	{
	  pointcloud.points[i].range =- 0.1;
	  pointcloud.points[i].x = 0.1;
	  pointcloud.points[i].y = 0.1;
	  pointcloud.points[i].z = 0.1;
	}
//      else
//      {
//	float azimuth = processcloud->points[i].azimuth;
//	int mat_i=azimuth*10;
//	PolarPointDI temppolarpoint;
//	temppolarpoint.distance=processcloud->points[i].range;
//	temppolarpoint.index=i;
//	polaraxismat_[i%LASER_LAYER][mat_i]=temppolarpoint;
//      }
  }
 }

 void preprocess()
 {
   if(playback_.RecordIsOn())
     Record();
   processcloud_r_=*cloud_r_;
   processcloud_l_=*cloud_l_;
   precomputecloud(processcloud_r_,calibvaluer_);
   precomputecloud(processcloud_l_,calibvaluel_);
   if(0) //是否移除外点
     {
       removeOutlier(processcloud_r_,grabber16_r_->indexmaptable,lasernum_);
       removeOutlier(processcloud_l_,grabber16_l_->indexmaptable,lasernum_);
       pcl::transformPointCloud (processcloud_r_, processcloud_r_,transform_matrix_calibration_R2V_);
       pcl::transformPointCloud (processcloud_l_, processcloud_l_,transform_matrix_calibration_L2V_);
     }
   else
     {
       pcl::transformPointCloud (processcloud_r_, processcloud_r_,transform_matrix_calibration_R2V_);
       pcl::transformPointCloud (processcloud_l_, processcloud_l_,transform_matrix_calibration_L2V_);
       addcloudinfo(processcloud_r_,grabber16_r_->indexmaptable,lasernum_);
       addcloudinfo(processcloud_l_,grabber16_l_->indexmaptable,lasernum_);
//      reordercloud(processcloud_r_,grabber16_r_->indexmaptable,lasernum_);
//      reordercloud(processcloud_l_,grabber16_l_->indexmaptable,lasernum_);
     }

   filtercloudonvehicle(processcloud_r_,calibvaluer_,cloudlimit_r_);
   filtercloudonvehicle(processcloud_l_,calibvaluel_,cloudlimit_l_);
 }
 const Cloud& gettotalcloud()
 {
   return totalcloud_;
 }
private:
  IvDataPlayback playback_;
  XmlConf xmlconfig_;
  double stamp_;
  std::string filename_;
  int lasernum_;
  CalibrationValue calibvaluer_;
  CalibrationValue calibvaluel_;
  bool inited_;
  int replay_;
  bool show_window_;
  bool playback_on_;
  int playback_mode;
  bool record_on;
  bool oneframecalibmode_;
  bool saveoneframe_;
  std::string pcapFile_;
  std::string hdlCalibration_;
  std::string hdlIP_;
  int hdlPortr_;
  int hdlPortl_;
  std::string configstr_;
  pcl::MyHDLGrabber* grabber16_r_;
  pcl::MyHDLGrabber* grabber16_l_;
  bool updated_;
  CloudConstPtr cloud_r_;
  CloudConstPtr cloud_l_;
  Cloud processcloud_r_;
  Cloud processcloud_l_;
  Cloud totalcloud_;
  boost::signals2::connection cloud_connection_r_;
  boost::signals2::connection cloud_connection_l_;
  Eigen::Matrix4f transform_matrix_calibration_R2V_;
  Eigen::Matrix4f transform_matrix_calibration_L2V_;
  boost::thread *hdl_read_pcd_thread_;
  double stampr_;
  double stampl_;

  string calibrationdir_r_;
  string calibrationdir_l_;
  CloudLimit cloudlimit_r_;
  CloudLimit cloudlimit_l_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "getrslidardata");
  ros::NodeHandle nh;

  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  FLAGS_colorlogtostderr = true; //设置输出到屏幕的日志显示相应颜色

  ros::Publisher pubLaserCloud;
  pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>
                                 ("lidar_cloud_calibrated", 2);

  ros::ServiceClient configclient = nh.serviceClient<sensor_driver_msgs::startconfig>("startconfigsrv");
//    subConfig = node_handle_.subscribe<std_msgs::String>("startconfig",2, boost::bind(&Node::subStartConfigHandle,this,_1));
  sensor_driver_msgs::startconfig configsrv;

  while(!configclient.call(configsrv))
   {
     ros::Duration(0.01).sleep();
   }

  std::string startconfig = configsrv.response.configstr;

  RsLidardata rslidar(startconfig);
  ros::Rate rate(100);
  bool status = ros::ok();
  MyTime mytime;
  mytime.start();
  while(status)
    {
      if(rslidar.isvalid())
        {
    	  mytime.start();
	  rslidar.preprocess();
	  rslidar.mixcloud();
	  rslidar.resetcloudstate();
	  const Cloud& totalcloud= rslidar.gettotalcloud();
	  LOG(INFO)<<"totalcloud.size="<<totalcloud.size();
	  sensor_msgs::PointCloud2 cloudmsg;
	  pcl::toROSMsg(totalcloud, cloudmsg);
	  cloudmsg.header.frame_id = "vehicle_frame";
	  pubLaserCloud.publish(cloudmsg);
	  LOG(INFO)<<"pub pointcloud";
        }
      else
      {
    	  mytime.stop();
    	  if(mytime.gettime_s()>0.2)
    	  {
    		  int i1 = system("ping 192.168.1.201 -c 2 -i 0.2");
    		  int i2 = system("ping 192.168.1.202 -c 2 -i 0.2");
    		  if(i1!=0)
    			  std::cerr << "can't ping 192.168.1.201";
    		  if(i2!=0)
    			  std::cerr << "can't ping 192.168.1.202";
    	  }
      }
      rate.sleep();
      status = ros::ok();
    }

  return 1;

}
