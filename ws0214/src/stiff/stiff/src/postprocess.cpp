

#include <pcl/io/boost.h>
#include <boost/version.hpp>
#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/math/special_functions.hpp>
#include <boost/lambda/lambda.hpp>
#include "postprocess.h"

#define LOCAL_IP "192.168.0.112"
//#define LOCAL_IP "127.0.0.1"
#define FROMLADAR_LOCAL_PORT 9906



	void PostProcess::init(const std::string &correctionsfile)
	{


//		subLaserOdometry_ = nodehandle_.subscribe<nav_msgs::Odometry>
//										 ("lidar_odometry_to_init", 5, boost::bind(&PostProcess::laserOdometryHandler,this,_1));//需要雷达里程计信息时需要，否则可以注释掉

		subLaserCloudFullRes_ = nodehandle_.subscribe<sensor_msgs::PointCloud2>
										 ("lidar_cloud_calibrated", 1, boost::bind(&PostProcess::laserCloudHandler,this,_1));//经过筛选且转换之后的点云

		subLaserOdometry_ = nodehandle_.subscribe<sensor_driver_msgs::OdometrywithGps>
												 ("lidar_odometry_to_earth", 10, boost::bind(&PostProcess::laserOdometryHandler,this,_1));


		//file_.open("/home/jkj/catkin_ws/result.txt",std::ios::out);

		 loadCorrectionsFile (correctionsfile);
		 for(int index=0;index<HDL_MAX_NUM_LASERS;index++)
		  {
		      if(index<LASER_LAYER)
		      {
		          double angle=laser_corrections_[index].verticalCorrection;
		          //double tan_angle=tan(HDL_Grabber_toRadians(angle));
		          map_tanangle_index[angle]=index;     //build mapping
		      }
		  }
		 //="<<map_tanangle_index.size()<<std::endl;

		  int index=0;
		  for(std::map<double,int>::iterator iter=map_tanangle_index.begin() ; iter!=map_tanangle_index.end(); iter++)
		  {

		      indexmaptable[index].number=iter->second;
		      indexmaptable[index].angle=iter->first;
		      //cout<<index<<"\tlaserindex="<<iter->second<<"\tangle="<<iter->first<<"\ttanangle="<<tan(HDL_Grabber_toRadians(iter->first))<<endl;
		      index++;
		  }
//		  if(1)
//		      {
//		          boost::function0<void> fdisplay = boost::bind(&PostProcess::displayPointCloud,this);
//		          thread_displayPointCloud=new boost::thread(fdisplay);
//		      }

		  for(int i=0;i<LASER_LAYER;i++)
		     {
		         theorydis[i]=-z_offset_h/std::tan(indexmaptable[i].angle*M_PI/180);
//		         if(theorydis[i]>0&&theorydis[i]<60)
//		             anglerange[i]=2.5/theorydis[i]*180/M_PI;
//		         cout<<"theorydis["<<i<<"]="<< theorydis[i]<<endl;
		     }
		  stiffcloud_= Cloud::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
		  normalcloud_ = Cloud::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
//		  SetViewerPCL( cloud_viewer_);
//		  SetViewerEgoVehicleModel( cloud_viewer_);
		  processthread_ = new boost::thread(boost::bind(&PostProcess::process,this));

	}
	void PostProcess::coordinate_from_vehicle_to_velodyne(float x, float y , float z, float& newx, float& newy, float& newz)
	{
	    newz = z;
	    newx = x;
	    newy = y;
	}
	void PostProcess::keyboard_callback (const KeyboardEvent& event, void* cookie)
	{
	    if (event.keyUp ())
	    {
	        if(event.getKeyCode() ==  '1')
	            freeze_ = 1;
	        else if(event.getKeyCode() ==  '2')
	            freeze_ = 0;
	        return;
	    }
	}
	void PostProcess::mouse_callback (const MouseEvent& mouse_event, void* cookie)
	{
		if (mouse_event.getType () == MouseEvent::MouseButtonPress &&
			mouse_event.getButton () == MouseEvent::LeftButton)
		{
			cout << mouse_event.getX () << " , " << mouse_event.getY () << endl;
		}
	}

	void PostProcess::SetViewerPCL(boost::shared_ptr<PCLVisualizer> cloud_viewer_)
	{
		//��ʼ��PCL��ʾ��������ز���
		cloud_viewer_->addCoordinateSystem (3.0);
		cloud_viewer_->setBackgroundColor(0.75,0.75,0.75);//(1.0,1.0,1.0);// (255, 0, 0);
		cloud_viewer_->initCameraParameters ();
		cloud_viewer_->setCameraPosition (0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0);
		cloud_viewer_->setCameraClipDistances (1.0, 120.0);
		cloud_viewer_->registerMouseCallback(&PostProcess::mouse_callback, *this);
		cloud_viewer_->registerKeyboardCallback (&PostProcess::keyboard_callback, *this);
		std::cout<<"........................................................"<<std::endl;

	}
	void PostProcess::SetViewerEgoVehicleModel(boost::shared_ptr<PCLVisualizer> cloud_viewer_)
	{
		//�Զ���ʻ����Ѳ������//��ʾ���ڻ�����
		if(1)
		{
			//������
			float x1 = -1 , x2 = 1 , y1 = -1 , y2 = 3, z = 0;
			float newx1, newx2, newy1, newy2, newz;
			coordinate_from_vehicle_to_velodyne(x1,y1,z,newx1,newy1,newz);
			coordinate_from_vehicle_to_velodyne(x2,y2,z,newx2,newy2,newz);
			pcl::PointXYZI pt1, pt2, pt3, pt4;
			pt1.x = newx1 ;
			pt1.y = newy1 ;
			pt1.z = newz;
			pt2.x = newx1 ;
			pt2.y = newy2 ;
			pt2.z = newz;
			cloud_viewer_->addLine(pt1, pt2, "body1");

			pt1.x = newx2 ;
			pt1.y = newy2 ;
			pt1.z = newz;
			cloud_viewer_->addLine(pt1, pt2, "body2");

			pt2.x = newx2 ;
			pt2.y = newy1 ;
			pt2.z = newz;
			cloud_viewer_->addLine(pt1, pt2, "body3");

			pt1.x = newx1 ;
			pt1.y = newy1 ;
			pt1.z = newz;
			cloud_viewer_->addLine(pt1, pt2, "body4");

			pt1.x = 0 ;
			pt1.y = 3 ;
			pt1.z = 0;
			pt2.x = -1 ;
			pt2.y = 1.5 ;
			pt2.z = 0;
			cloud_viewer_->addLine(pt1, pt2, "body5");

			pt1.x = 0 ;
			pt1.y = 3 ;
			pt1.z = 0;
			pt2.x = 1 ;
			pt2.y = 1.5 ;
			pt2.z = 0;
			cloud_viewer_->addLine(pt1, pt2, "body6");

			pt1.x = -1 ;
			pt1.y = 1.5 ;
			pt1.z = 0;
			pt2.x = 1 ;
			pt2.y = 1.5 ;
			pt2.z = 0;
			cloud_viewer_->addLine(pt1, pt2, "body7");

			//���Ϸ�Χ
			if(0)
			{
				float x1 = -20 , x2 = 20 , y1 = -1 , y2 = 40, z = Z_MAX;
				float newx1, newx2, newy1, newy2, newz;
				coordinate_from_vehicle_to_velodyne(x1,y1,z,newx1,newy1,newz);
				coordinate_from_vehicle_to_velodyne(x2,y2,z,newx2,newy2,newz);
				pcl::PointXYZI pt1, pt2, pt3, pt4;
				pt1.x = newx1 ;
				pt1.y = newy1 ;
				pt1.z = newz;
				pt2.x = newx1 ;
				pt2.y = newy2 ;
				pt2.z = newz;
				cloud_viewer_->addLine(pt1, pt2, "upper1");

				pt1.x = newx2 ;
				pt1.y = newy2 ;
				pt1.z = newz;
				cloud_viewer_->addLine(pt1, pt2, "upper2");

				pt2.x = newx2 ;
				pt2.y = newy1 ;
				pt2.z = newz;
				cloud_viewer_->addLine(pt1, pt2, "upper3");

				pt1.x = newx1 ;
				pt1.y = newy1 ;
				pt1.z = newz;
				cloud_viewer_->addLine(pt1, pt2, "upper4");
			}

			//��������
			if (0)
			{
				char linename[20];
				for(int i = 0 ; i < 10 ; i++)
				{
					x1 = -20 ;
					x2 = 20 ;
					y1 = (i - 2) * 10 ;
					y2 = (i - 2) * 10;
					z = 0;
					coordinate_from_vehicle_to_velodyne(x1,y1,z,newx1,newy1,newz);
					coordinate_from_vehicle_to_velodyne(x2,y2,z,newx2,newy2,newz);
					pt1.x = min(newx1 , newx2) ;
					pt1.y = min(newy1 , newy2) ;
					pt1.z = newz;
					pt2.x = max(newx1 , newx2) ;
					pt2.y = max(newy1 , newy2) ;
					pt2.z = newz;
					memset(linename, 0 , 20);
					sprintf(linename , "lat%02d", i);
					cloud_viewer_->addLine(pt1, pt2, linename);
				}
				for(int i = 0 ; i < 5 ; i++)
				{
					x1 = i * 10 - 20;
					x2 = i * 10 - 20;
					y1 = -20 ;
					y2 = 70 ;
					z = 0;
					coordinate_from_vehicle_to_velodyne(x1,y1,z,newx1,newy1,newz);
					coordinate_from_vehicle_to_velodyne(x2,y2,z,newx2,newy2,newz);
					pt1.x = min(newx1 , newx2) ;
					pt1.y = min(newy1 , newy2) ;
					pt1.z = newz;
					pt2.x = max(newx1 , newx2) ;
					pt2.y = max(newy1 , newy2) ;
					pt2.z = newz;
					memset(linename, 0 , 20);
					sprintf(linename , "lng%02d" , i);
					cloud_viewer_->addLine(pt1, pt2, linename);


				}
			}
			if (1)
			{
				for (int i=1; i<5; i++)
				{
					pcl::ModelCoefficients coefficients/*(new pcl::ModelCoefficients)*/;
					coefficients.values.resize(3);
					coefficients.values[0] = 0;
					coefficients.values[1] = 0;
					coefficients.values[2] = i*10;


					char name[20];
					sprintf(name, "circle%d",i);
					cloud_viewer_->addCircle(coefficients,name,0);

				}
			}

		}
	}
	void PostProcess::ShowViewerCloudPoints( boost::shared_ptr<PCLVisualizer> cloud_viewer_, vector<Cloud::ConstPtr> cloud_show_,
		char cloudname[], double color_red_, double color_green_, double color_blue_ )
	{
		cloud_viewer_->removeAllPointClouds();
		if(cloud_show_.size()==3){


					pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloudHandler( cloud_show_[0], 255, 0, 0);
					if (!cloud_viewer_->updatePointCloud(cloud_show_[0],cloudHandler, cloudname))
					{
						//				std::cout<<"cloudgeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeet"<<std::endl;
						cloud_viewer_->addPointCloud(cloud_show_[0], cloudHandler, cloudname);
					}
//					pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloudHandler1( cloud_show_[1], 0,255,0);
//					if (!cloud_viewer_->updatePointCloud(cloud_show_[1],cloudHandler1, cloudname))
//					{
//						//				std::cout<<"cloudgeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeet"<<std::endl;
//						cloud_viewer_->addPointCloud(cloud_show_[1], cloudHandler1, cloudname);
//					}
//					pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloudHandler2( cloud_show_[2], 0,0,255);
//					if (!cloud_viewer_->updatePointCloud(cloud_show_[2],cloudHandler2, cloudname))
//					{
//						//				std::cout<<"cloudgeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeet"<<std::endl;
//						cloud_viewer_->addPointCloud(cloud_show_[2], cloudHandler2, cloudname);
//					}


		}

	}
	void PostProcess::ShowViewerCloudPoints( boost::shared_ptr<PCLVisualizer> cloud_viewer_, Cloud::ConstPtr cloud_show_,
		char cloudname[], double color_red_, double color_green_, double color_blue_ )
	{
		cloud_viewer_->removeAllPointClouds();

			if ( cloud_show_->size()>0 )
			{

				pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloudHandler( cloud_show_, color_red_, color_green_, color_blue_ );
				if (!cloud_viewer_->updatePointCloud(cloud_show_,cloudHandler, cloudname))
				{
					//				std::cout<<"cloudgeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeet"<<std::endl;
					cloud_viewer_->addPointCloud(cloud_show_, cloudHandler, cloudname);
				}
			}

	}
	void PostProcess::displayPointCloud(Cloud::Ptr cloud_show)//
	{
	    boost::shared_ptr<PCLVisualizer> cloud_viewer_(new PCLVisualizer ("stiff Cloud"));

	    cloud_viewer_->addCoordinateSystem (3.0);
	    cloud_viewer_->setBackgroundColor (0, 0, 0);
	    cloud_viewer_->initCameraParameters ();
	    cloud_viewer_->setCameraPosition (0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0);
	    cloud_viewer_->setCameraClipDistances (0.0, 100.0);
	    cloud_viewer_->registerKeyboardCallback (&PostProcess::keyboard_callback, *this);
	    //cloud_viewer_->registerMouseCallback(&PostProcess::mouse_callback,*this);
	    {
	        //画
	        float x1 = -1 , x2 = 1 , y1 = -1 , y2 = 3, z = 0;
	        float newx1, newx2, newy1, newy2, newz;
	        coordinate_from_vehicle_to_velodyne(x1,y1,z,newx1,newy1,newz);
	        coordinate_from_vehicle_to_velodyne(x2,y2,z,newx2,newy2,newz);
	        pcl::PointXYZI pt1, pt2, pt3, pt4;
	        pt1.x = newx1 ;
	        pt1.y = newy1 ;
	        pt1.z = newz;
	        pt2.x = newx1 ;
	        pt2.y = newy2 ;
	        pt2.z = newz;
	        cloud_viewer_->addLine(pt1, pt2, "body1");

	        pt1.x = newx2 ;
	        pt1.y = newy2 ;
	        pt1.z = newz;
	        cloud_viewer_->addLine(pt1, pt2, "body2");

	        pt2.x = newx2 ;
	        pt2.y = newy1 ;
	        pt2.z = newz;
	        cloud_viewer_->addLine(pt1, pt2, "body3");

	        pt1.x = newx1 ;
	        pt1.y = newy1 ;
	        pt1.z = newz;
	        cloud_viewer_->addLine(pt1, pt2, "body4");



	        //画上范围
	        if(0)
	        {
	            float x1 = -20 , x2 = 20 , y1 = -1 , y2 = 40, z = Z_MAX;
	            float newx1, newx2, newy1, newy2, newz;
	            coordinate_from_vehicle_to_velodyne(x1,y1,z,newx1,newy1,newz);
	            coordinate_from_vehicle_to_velodyne(x2,y2,z,newx2,newy2,newz);
	            pcl::PointXYZI pt1, pt2, pt3, pt4;
	            pt1.x = newx1 ;
	            pt1.y = newy1 ;
	            pt1.z = newz;
	            pt2.x = newx1 ;
	            pt2.y = newy2 ;
	            pt2.z = newz;
	            cloud_viewer_->addLine(pt1, pt2, "upper1");

	            pt1.x = newx2 ;
	            pt1.y = newy2 ;
	            pt1.z = newz;
	            cloud_viewer_->addLine(pt1, pt2, "upper2");

	            pt2.x = newx2 ;
	            pt2.y = newy1 ;
	            pt2.z = newz;
	            cloud_viewer_->addLine(pt1, pt2, "upper3");

	            pt1.x = newx1 ;
	            pt1.y = newy1 ;
	            pt1.z = newz;
	            cloud_viewer_->addLine(pt1, pt2, "upper4");
	        }

	        //画网格线
//	        char linename[20];
//	        for(int i = 0 ; i < 10 ; i++)
//	        {
//	            x1 = -20 ;
//	            x2 = 20 ;
//	            y1 = (i - 2) * 10 ;
//	            y2 = (i - 2) * 10;
//	            z = 0;
//	            coordinate_from_vehicle_to_velodyne(x1,y1,z,newx1,newy1,newz);
//	            coordinate_from_vehicle_to_velodyne(x2,y2,z,newx2,newy2,newz);
//	            pt1.x = min(newx1 , newx2) ;
//	            pt1.y = min(newy1 , newy2) ;
//	            pt1.z = newz;
//	            pt2.x = max(newx1 , newx2) ;
//	            pt2.y = max(newy1 , newy2) ;
//	            pt2.z = newz;
//	            memset(linename, 0 , 20);
//	            sprintf(linename , "lat%02d" , i);
//	            cloud_viewer_->addLine(pt1, pt2, linename);
//	        }
//
//	        for(int i = 0 ; i < 5 ; i++)
//	        {
//	            x1 = i * 10 - 20;
//	            x2 = i * 10 - 20;
//	            y1 = -20 ;
//	            y2 = 70 ;
//	            z = 0;
//	            coordinate_from_vehicle_to_velodyne(x1,y1,z,newx1,newy1,newz);
//	            coordinate_from_vehicle_to_velodyne(x2,y2,z,newx2,newy2,newz);
//	            pt1.x = min(newx1 , newx2) ;
//	            pt1.y = min(newy1 , newy2) ;
//	            pt1.z = newz;
//	            pt2.x = max(newx1 , newx2) ;
//	            pt2.y = max(newy1 , newy2) ;
//	            pt2.z = newz;
//	            memset(linename, 0 , 20);
//	            sprintf(linename , "lng%02d" , i);
//	            cloud_viewer_->addLine(pt1, pt2, linename);
//	        }
    }

	    while (!cloud_viewer_->wasStopped ())//&&!flag_close)
	    {

	    						        std::cout<<"getclouuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuud"<<std::endl;
	    		                        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> rigid_nopassablecloudHandler (cloud_show, 255, 0, 0);
	    		                        if (!cloud_viewer_->updatePointCloud (cloud_show, rigid_nopassablecloudHandler, "normalcloud"))
	    		                            cloud_viewer_->addPointCloud (cloud_show, rigid_nopassablecloudHandler, "normalcloud");
	        if(1)
	        {
//	            if(freeze_==1)
//	                cloud_viewer_->spinOnce();
//	            else
	            {
	                //display
	                // if(velodyne_pointcloud->points.size()>0)
	                if(cloudupdate)
	                {

	                    cloudupdate=false;
	                  //  boost::mutex::scoped_lock lock(displaymutex);

	                    cloud_viewer_->removeAllPointClouds();
	                    //��ͨ������
	                    if(stiffcloud_->points.size()>0)
	                    {
	                        //��ɫ

	                        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> passablecloudHandler (stiffcloud_, 0, 255, 0);
	                        if (!cloud_viewer_->updatePointCloud (stiffcloud_, passablecloudHandler, "stiffcloud")){
	                            cloud_viewer_->addPointCloud (stiffcloud_, passablecloudHandler, "stiffcloud");
	                        }
	                    }

	                    if(cloud_show->points.size()>0)
	                    {
//	                        std::cout<<"getclouuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuud"<<std::endl;
//	                        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> rigid_nopassablecloudHandler (cloud_show, 255, 0, 0);
//	                        if (!cloud_viewer_->updatePointCloud (cloud_show, rigid_nopassablecloudHandler, "normalcloud")){
//	                            cloud_viewer_->addPointCloud (cloud_show, rigid_nopassablecloudHandler, "normalcloud");
	                        }
	                    }


	                    //       if(velodyne_pointcloud)
	                cloud_viewer_->spinOnce();



	                }
//	                if (!grabber_H_.isRunning())
//	                    cloud_viewer_->spin ();


	                ////��ʾ********************/

	#ifdef SIMULATION
	                boost::this_thread::sleep (boost::posix_time::microseconds (50000));
	#else
	                boost::this_thread::sleep (boost::posix_time::microseconds (100));
	#endif

	        }
	    }
	    //    if (replay_ == 1)
	    //        grabber_H_.resume();

	    //flag_close=true;
	    cloud_viewer_->close();
	}

	void PostProcess::SendData(OGMData<unsigned char>& ogmdata)  //udp通信 发送端例程
	{
		BoostUdp sendogmdata(LOCAL_IP,FROMLADAR_LOCAL_PORT);
		sendogmdata.connectRemoteEndpoint("192.168.0.111",9905);
	//	sendogmdata.connectRemoteEndpoint("127.0.0.1",9905);
		int lenth=2000;
		int package = ogmdata.ogmcell_size/lenth;
		int package_total;

		char ogm_total_data[ogmdata.ogmcell_size];
		int headernum = 3;
		char header[headernum];
		//memcpy(ogm_total_data,ogmdata.ogm,1);
		if(ogmdata.ogmcell_size - lenth * package>0)
		{
			package_total = package + 1;
		}
		for(int i =0;i < package;i++)
		{
			int ogmstart = lenth*i;
			char senddata[lenth+headernum];
			header[0]='a';
			header[1]=i;
			header[2]=package_total;
			memcpy(senddata,header,headernum);
			memcpy(senddata+headernum,ogmdata.ogm+ogmstart,lenth);
			sendogmdata.send(senddata,sizeof(senddata));
		}
		if(ogmdata.ogmcell_size - lenth * package>0)
		{
				int lenth_new = ogmdata.ogmcell_size - lenth * package;
				char senddata[lenth_new];
				header[0] = 0x88;
				header[1] = package;
				header[2] = package_total;
				memcpy(senddata,header,headernum);
				memcpy(senddata+headernum,ogmdata.ogm+lenth*package,lenth_new);
				sendogmdata.send(senddata,sizeof(senddata));
		}
	}

	void PostProcess::laserOdometryHandler(const sensor_driver_msgs::OdometrywithGps::ConstPtr& laserOdometry)  //雷达里程计
	{
		double timeOdometry = laserOdometry->odometry.header.stamp.toSec();
	  static double last_stamp = -1;
	//  static geometry_msgs::Quaternion last_geoQuat;
	  static transform::Rigid3d lasttransformodometry;
	//  static float last_trans[6];
	//  double roll, pitch, yaw;
	  geometry_msgs::Quaternion geoQuat = laserOdometry->odometry.pose.pose.orientation;

	  Eigen::Quaterniond roatation(geoQuat.w,geoQuat.x,geoQuat.y,geoQuat.z);
	  Eigen::Vector3d translation(laserOdometry->odometry.pose.pose.position.x,
			  laserOdometry->odometry.pose.pose.position.y,
			  laserOdometry->odometry.pose.pose.position.z);

	  transform::Rigid3d transformodometry(translation,roatation);

	  lidarOdoms_.Push(common::make_unique<TimePosePair>(timeOdometry,transformodometry));

	}



	void PostProcess::laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullRes2) //点云数据
	{
		double timeLaserCloudFullRes = laserCloudFullRes2->header.stamp.toSec();
		timestamp_=laserCloudFullRes2->header.stamp;
//		std::cout<<"lasertime" << fixed << setprecision(9)<<timestamp_<<std::endl;

//		LOG(INFO)<<std::fixed<<std::setprecision(3)<<"cloudtime:"<<timeLaserCloudFullRes;
//		LOG(INFO)<<"starttime"<<ros::Time::now().toSec() - timeLaserCloudFullRes;
	    lidarCloudMsgs_.Push(laserCloudFullRes2);
	    if(lidarCloudMsgs_.Size()>1)
		  lidarCloudMsgs_.Pop();

	}

	void PostProcess::analysisCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr inputcloud,
			std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& outputclouds,std::vector<pcl::PointXYZI>& lidarpropertys)
	{

		  //////////////////////////总的点云中可能包含了几组独立的点云数据，对发过来的点云进行处理，将每一组点云都提取出来////////////////////////////////////////
		  int cloudnum = inputcloud->size() % 16;//包含的点云包数目
		  vector<int> startnum;

		  for(int i =0;i<cloudnum;i++)
		  {
			  pcl::PointXYZI originpoint;
			  int flag = (*inputcloud)[inputcloud->size()-cloudnum+i].range;//每一包点云的第一个点的位置
			  (*inputcloud)[inputcloud->size()-cloudnum+i].range = -0.5;
			  originpoint.x = (*inputcloud)[inputcloud->size()-cloudnum+i].x;//每一包点云中对应的雷达在车体坐标系的x
			  originpoint.y = (*inputcloud)[inputcloud->size()-cloudnum+i].y;////每一包点云中对应的雷达在车体坐标系的y
			  originpoint.z = (*inputcloud)[inputcloud->size()-cloudnum+i].z;////每一包点云中对应的雷达在车体坐标系的z
			  originpoint.intensity = (*inputcloud)[inputcloud->size()-cloudnum+i].azimuth;//每一包点云中对应的雷达线束
			  startnum.push_back(flag);
			  lidarpropertys.push_back(originpoint);
		  }
		  for(int i = 0;i < startnum.size();i++)
		  {
			int length;
			pcl::PointCloud<pcl::PointXYZI>::Ptr lasercloudptr(new pcl::PointCloud<pcl::PointXYZI>);//每一包点云

			if(i == startnum.size()-1)
			{
				length = inputcloud->size() - cloudnum - startnum.at(i);
			}
			else
			{
				length = startnum.at(i+1) - startnum.at(i);
			}

			lasercloudptr->insert(lasercloudptr->begin(),inputcloud->begin()+startnum.at(i),inputcloud->begin()+startnum.at(i)+length);
			outputclouds.push_back(lasercloudptr);

		  }

	}

	void PostProcess::loadCorrectionsFile (const std::string& correctionsFile)
	{
		boost::property_tree::ptree pt;
		  try
		  {
		    read_xml (correctionsFile, pt, boost::property_tree::xml_parser::trim_whitespace);
		  }
		  catch (boost::exception const&)
		  {
		    PCL_ERROR ("[pcl::MyHDLGrabber::loadCorrectionsFile] Error reading calibration file %s!\n", correctionsFile.c_str ());
		    return;
		  }

		  BOOST_FOREACH (boost::property_tree::ptree::value_type &v, pt.get_child ("boost_serialization.DB.points_"))
		  {
		    if (v.first == "item")
		    {
		      boost::property_tree::ptree points = v.second;
		      BOOST_FOREACH(boost::property_tree::ptree::value_type &px, points)
		      {
		        if (px.first == "px")
		        {
		          boost::property_tree::ptree calibrationData = px.second;
		          int index = -1;
		          double azimuth = 0, vertCorrection = 0, distCorrection = 0,
		                 vertOffsetCorrection = 0, horizOffsetCorrection = 0;

		          BOOST_FOREACH (boost::property_tree::ptree::value_type &item, calibrationData)
		          {
		            if (item.first == "id_")
		              index = atoi (item.second.data ().c_str ());
		            if (item.first == "rotCorrection_")
		              azimuth = atof (item.second.data ().c_str ());
		            if (item.first == "vertCorrection_")
		              vertCorrection = atof (item.second.data ().c_str ());
		            if (item.first == "distCorrection_")
		              distCorrection = atof (item.second.data ().c_str ());
		            if (item.first == "vertOffsetCorrection_")
		              vertOffsetCorrection = atof (item.second.data ().c_str ());
		            if (item.first == "horizOffsetCorrection_")
		              horizOffsetCorrection = atof (item.second.data ().c_str ());
		          }
		          if (index != -1)
		          {
		            laser_corrections_[index].azimuthCorrection = azimuth;
		            laser_corrections_[index].verticalCorrection = vertCorrection;
		            laser_corrections_[index].distanceCorrection = distCorrection / 100.0;
		            laser_corrections_[index].verticalOffsetCorrection = vertOffsetCorrection / 100.0;
		            laser_corrections_[index].horizontalOffsetCorrection = horizOffsetCorrection / 100.0;

		            laser_corrections_[index].cosVertCorrection = std::cos (HDL_Grabber_toRadians(laser_corrections_[index].verticalCorrection));
		            laser_corrections_[index].sinVertCorrection = std::sin (HDL_Grabber_toRadians(laser_corrections_[index].verticalCorrection));

		          }
		        }
		      }
		    }
		  }
	}
	void PostProcess::countogmpoints(vector<pcl::PointCloud<pcl::PointXYZI>::ConstPtr> cloud)
	{
		memset(point_count_ogm_.ogm , 0 , point_count_ogm_.ogmcell_size*sizeof(int));//每次都要置零,避免填充错乱
		memset(maxz_ogm_.ogm , 0 , maxz_ogm_.ogmcell_size*sizeof(int));
//		memset(ogm_msg_.ogm,0,ogm_msg_.ogmcell_size*sizeof(unsigned int));
		memset(ogm_msg_.ogm,0,ogm_msg_.ogmcell_size*sizeof(unsigned char));

		float ogm_y_offset = 20.0f;
		//为每个栅格赋值点云数量
		for(int cloudi=0;cloudi<cloud.size();cloudi++){
//			cout<<"cloudi sieze if "<<cloud[cloudi]->points.size()<<"    ";
			for (int i = 0; i < cloud[cloudi]->points.size(); i++)
			{

				float x = cloud[cloudi]->points[i].x,
						y = cloud[cloudi]->points[i].y,
						z = cloud[cloudi]->points[i].z;

				float newy = y + ogm_y_offset;//ogm_y_offset
				//					if(cloud[cloudi]->points[i].intensity < 15.0f)//排除噪声干扰，只考虑反射率较高的点
				//					                continue;
				if(x>-1.6&&x<1.6&&y>-1&&y<3){		//排除车身周围
					continue;
				}

				if((x >=-point_count_ogm_.ogmwidth / 2  && x <= point_count_ogm_.ogmwidth / 2) &&//这里的条件是否需要再考虑一下
						(newy >=0 && newy < point_count_ogm_.ogmheight) &&
						( z <=  Z_MAX))
				{
					int col = boost::math::round(x / point_count_ogm_.ogmresolution) + ( point_count_ogm_.ogmwidth_cell - 1 ) / 2;
					int row = boost::math::round(newy / point_count_ogm_.ogmresolution) ;

					if((row >=0 && row < point_count_ogm_.ogmheight_cell)
							&& (col >=0 && col < point_count_ogm_.ogmwidth_cell))
					{
						int index = row * point_count_ogm_.ogmwidth_cell + col;
						point_count_ogm_.ogm[index]++;
					}
				}

			}

			//test
			//		for(int i=point_count_ogm_.ogmwidth_cell/2;i<point_count_ogm_.ogmwidth_cell/2+6/point_count_ogm_.ogmresolution;i++){
			//			int index=20/point_count_ogm_.ogmresolution*point_count_ogm_.ogmwidth_cell+i;
			//			std::cout<<point_count_ogm_.ogm[index]<<" ";
			//		}
			//		std::cout<<std::endl;
			/////////////////////////////////
			//为每个栅格赋值maxz
			for (int i = 0; i < cloud[cloudi]->points.size(); i++)
			{
				float x = cloud[cloudi]->points[i].x,
						y = cloud[cloudi]->points[i].y,
						z = cloud[cloudi]->points[i].z;
				float newy = y + ogm_y_offset;
				//			if(cloud[cloudi]->points[i].intensity < 15.0f)//排除噪声干扰，只考虑反射率较高的点
				//			                continue;
				if(x>-1.5&&x<1.5&&y>-1&&y<4){		//排除车身周围------------------------------------zx
					continue;
				}

				//			            newy = y + maxz_ogm_.ogmheight/2;
				if((x >=-maxz_ogm_.ogmwidth / 2  && x <= maxz_ogm_.ogmwidth / 2) &&
						(newy >=0 && newy < maxz_ogm_.ogmheight) &&
						(z >= - 2 && z <=  Z_MAX))
				{
					int col = boost::math::round(x / maxz_ogm_.ogmresolution) + ( maxz_ogm_.ogmwidth_cell - 1 ) / 2;
					int row = boost::math::round(newy / maxz_ogm_.ogmresolution) ;

					if((row >=0 && row < maxz_ogm_.ogmheight_cell)					//加了_cell的才是栅格zx
							&& (col >=0 && col < maxz_ogm_.ogmwidth_cell))
					{
						int index = row * maxz_ogm_.ogmwidth_cell + col;
						if( maxz_ogm_.ogm[index] < z)
							maxz_ogm_.ogm[index] = z;
					}
				}
			}
			///test
			//		for(int i=maxz_ogm_.ogmwidth_cell/2;i<maxz_ogm_.ogmwidth_cell/2+15/maxz_ogm_.ogmresolution;i++){
			//				int index=35/maxz_ogm_.ogmresolution*maxz_ogm_.ogmwidth_cell+i;
			//				std::cout<<maxz_ogm_.ogm[index]<<" ";
			//			}
			//			std::cout<<std::endl;
			//测试代码,看每个格子的点数/高度
			//		for (int i=20;i<30;i++)
			//		{
			//			for(int j=20;j<40;j++)//车体右侧
			//				{
			////				if(maxz_ogm_.ogm[i*point_count_ogm_.ogmwidth_cell+j]>1){
			//				std::cout<<".."<<maxz_ogm_.ogm[i*point_count_ogm_.ogmwidth_cell+j];
			////				cout<<".."<<j<<".."<<i<<endl;
			////				}
			//			}
			//			std::cout<<std::endl;
			//		}
			//		for(int i=13;i<21;i++){
			//			std::cout<<point_count_ogm_.ogm[35*point_count_ogm_.ogmwidth_cell+i]<<"  ";
			//		}
			//		std::cout<<std::endl;
		}

		//下面将考虑通过检测比较大片的无点区域判断是否为悬崖zx
		for(int i=20/point_count_ogm_.ogmresolution+3;i<45/point_count_ogm_.ogmresolution;i++)//
		{
//			std::cout<<"......11111111111111.................."<<std::endl;
			int end_right=point_count_ogm_.ogmwidth_cell/2+16/point_count_ogm_.ogmresolution;
			for(int j=point_count_ogm_.ogmwidth_cell/2+4/point_count_ogm_.ogmresolution;j<end_right;j++)//车体右侧
			{

				int count=0;
				int index=i*point_count_ogm_.ogmwidth_cell+j;
															//用于记录连续没有点云的栅格数
								while(point_count_ogm_.ogm[index]<6&&count<end_right-j)//0221这里的10之前是16zx
								{
//									std::cout<<"......11111111111111.................."<<std::endl;
									count++;
									index++;
								}
//								int tempindex=index;
//								int index_img=(point_count_ogm_.ogmheight_cell-i-1)*point_count_ogm_.ogmwidth_cell+j;//这是在图像上的索引，用于测试
								index=4*i*ogm_msg_.ogmwidth_cell+j*4;//栅格地图上悬崖起始点索引
								int index2=(4*i+1)*ogm_msg_.ogmwidth_cell+4*j;
								int index3=(4*i+2)*ogm_msg_.ogmwidth_cell+4*j;
								int index4=(4*i+3)*ogm_msg_.ogmwidth_cell+4*j;
								if(count>GRID_THRESH)								//可调参数
								{
									float thresh_flat=0,maxz1=-10,maxz2=-10;//用来排除地面点云间隙造成的误检
									if(j+count==end_right-1){thresh_flat=-1;}//如果无点区域直接出界
									else{
										for(int k=4*i;k<4*i+4;k++){
											for(int l=4*j-4;l<4*j;l++){
												int small_i=k*maxz_ogm_.ogmwidth_cell+l;
												if(maxz_ogm_.ogm[small_i]>maxz1){
													maxz1=maxz_ogm_.ogm[small_i];
													if(abs(maxz1)>0.1) break;
												}
											}
											if(abs(maxz1)>0.1) break;
										}
										for(int k=4*i;k<4*i+4;k++){
											for(int l=4*(j+count);l<4*(j+count+1);l++){
												int small_i=k*maxz_ogm_.ogmwidth_cell+l;
												if(maxz_ogm_.ogm[small_i]>maxz2){
													maxz2=maxz_ogm_.ogm[small_i];
													if(abs(maxz2)>0.1) break;
												}
											}
											if(abs(maxz2)>0.1) break;
										}
										thresh_flat=(maxz1!=0&&maxz2!=0)?FLAT_THRESH:(-1);
									}
									if((pathClear(4*i,j*4)&&pathClear((i+count/2)*4,4*j))&&pathClear((i+count)*4+3,4*j)&&abs(maxz1-maxz2)>thresh_flat)	//判断一下由雷达到无点区域中点之间有无障碍
									{
//										std::cout<<"<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>"<<std::endl;
										memset(&ogm_msg_.ogm[index],5,count*4*sizeof(unsigned char));
										memset(&ogm_msg_.ogm[index2],5,count*4*sizeof(unsigned char));
										memset(&ogm_msg_.ogm[index3],5,count*4*sizeof(unsigned char));
										memset(&ogm_msg_.ogm[index4],5,count*4*sizeof(unsigned char));

//										for(int d=0;d<count;d++)
//										{
//											std::cout<<ogm_msg_.ogm[index+d]<<"  ";
//										}
//										std::cout<<"........................"<<std::endl;
										vecright_.push_back(point_count_ogm_.ogmheight_cell-i-1);
										vecright_.push_back(j);
										vecright_.push_back(count);

									}
								}
								if(count>0)
								j+=count-1;
			}

			for(int j=point_count_ogm_.ogmwidth_cell/2-20/point_count_ogm_.ogmresolution;j<point_count_ogm_.ogmwidth_cell/2-4/point_count_ogm_.ogmresolution;j++)//车体左侧
			{

				int count=0;
				int index=i*point_count_ogm_.ogmwidth_cell+j;
															//用于记录连续没有点云的栅格数
								while(point_count_ogm_.ogm[index]<6&&count<(point_count_ogm_.ogmwidth_cell/2-4/point_count_ogm_.ogmresolution-j))//0221这里的10之前是16zx
								{
//									std::cout<<"................"<<count+i<<std::endl;
									count++;
									index++;

								}
								int tempindex=index;
//								int index_img=(point_count_ogm_.ogmheight_cell-i-1)*point_count_ogm_.ogmwidth_cell+j;//这是在图像上的索引，用于测试
								index=4*i*ogm_msg_.ogmwidth_cell+j*4;//栅格地图上悬崖起始点索引
								int index2=(4*i+1)*ogm_msg_.ogmwidth_cell+4*j;
								int index3=(4*i+2)*ogm_msg_.ogmwidth_cell+4*j;
								int index4=(4*i+3)*ogm_msg_.ogmwidth_cell+4*j;
								if(count>GRID_THRESH)								//可调参数
								{
									float thresh_flat=0,maxz1=-10,maxz2=-10;//用来排除地面点云间隙造成的误检
									if(j==point_count_ogm_.ogmwidth_cell/2-20/point_count_ogm_.ogmresolution){thresh_flat=-1;}
									else{
										for(int k=4*i;k<4*i+4;k++){
											for(int l=4*j-1;l>4*j-5;l--){
												int small_i=k*maxz_ogm_.ogmwidth_cell+l;
												if(maxz_ogm_.ogm[small_i]>maxz1){
													maxz1=maxz_ogm_.ogm[small_i];
													if(abs(maxz1)>0.1) break;
												}
											}
											if(abs(maxz1)>0.1) break;
										}
										for(int k=4*i;k<4*i+4;k++){
											for(int l=4*(j+count);l<4*(j+count+1);l++){
												int small_i=k*maxz_ogm_.ogmwidth_cell+l;
												if(maxz_ogm_.ogm[small_i]>maxz2){
													maxz2=maxz_ogm_.ogm[small_i];
													if(abs(maxz2)>0.1) break;
												}
											}
											if(abs(maxz2)>0.1) break;
										}
										thresh_flat=(maxz1!=0&&maxz2!=0)?FLAT_THRESH:(-1);
									}
									if((pathClear(4*i,j*4)&&pathClear((i+count/2)*4,4*j))&&pathClear((i+count)*4+3,4*j)&&abs(maxz1-maxz2)>thresh_flat)	//&&abs(maxz1-maxz2)>thresh_flat
									{
//										cout<<"the maxz is  "<<maxz2<<"   "<<maxz1<<endl;
//										cout<<"absolute value is "<<abs(maxz1-maxz2)<<endl;
										memset(&ogm_msg_.ogm[index],5,count*4*sizeof(unsigned char));
										memset(&ogm_msg_.ogm[index2],5,count*4*sizeof(unsigned char));
										memset(&ogm_msg_.ogm[index3],5,count*4*sizeof(unsigned char));
										memset(&ogm_msg_.ogm[index4],5,count*4*sizeof(unsigned char));
//										for(int d=0;d<count;d++)
//										{
//											std::cout<<ogm_msg_.ogm[index+d]<<"  ";
//										}
//										std::cout<<"........................"<<std::endl;
										vecright_.push_back(point_count_ogm_.ogmheight_cell-i-1);
										vecright_.push_back(j);
										vecright_.push_back(count);

									}
								}
								if(count>0)
									j+=count-1;


			}
//
		}
		//test
//		for(int i=13;i<21;i++){
//			int index=35*point_count_ogm_.ogmwidth_cell+i;
//			int count=0;
//			while(point_count_ogm_.ogm[index]==0){
//				count++;
//				index++;
//			}
//			if(count>2){
//				if(pathClear(35,i)){
//					std::cout<<"yesyesyes"<<std::endl;
//				}
//				else cout<<"nonono"<<std::endl;
//			}
//		}
		//test done
		for(int j=point_count_ogm_.ogmwidth_cell/2-4/point_count_ogm_.ogmresolution;j<point_count_ogm_.ogmwidth_cell/2+4/point_count_ogm_.ogmresolution;j++)//车体上下范围
		{

//			for(int i=0;i<20/point_count_ogm_.ogmresolution;i++)//-3/point_count_ogm_.ogmresolution
//			{
//				int count=0;
//				//排除车体周围范围
//				if(j>point_count_ogm_.ogmwidth_cell/2-3/point_count_ogm_.ogmresolution&&j<point_count_ogm_.ogmwidth_cell/2+3/point_count_ogm_.ogmresolution
//						&&i<point_count_ogm_.ogmheight_cell/2+2/point_count_ogm_.ogmresolution&&i>point_count_ogm_.ogmheight_cell/2-4/point_count_ogm_.ogmresolution)
//					continue;
//				int index=i*point_count_ogm_.ogmwidth_cell+j;
//				while(point_count_ogm_.ogm[index]==0&&count<6&&count<point_count_ogm_.ogmheight_cell/2-4/point_count_ogm_.ogmresolution-i)//-point_count_ogm_.ogmheight_cell/2//&&count<point_count_ogm_.ogmheight_cell-i
//				{
//					count++;
//					index+=point_count_ogm_.ogmwidth_cell;
//				}
//				index=i*2*ogm_msg_.ogmwidth_cell+j*2;//这是在栅格坐标系下的索引zx
////				int index_img=(point_count_ogm_.ogmheight_cell-i-1)*point_count_ogm_.ogmwidth_cell+j;//这是在图像上的索引，用于测试
//				if(count>GRID_THRESH)								//可调参数
//				{
//					if(pathClear(i,j))	//判断一下由雷达到无点区域中点之间有无障碍  pathClear(j,i+count/2)
//					{
//						for(int k=0;k<count;k++)
//						{
//							memset(&ogm_msg_.ogm[index],5,2*sizeof(unsigned char));
//							memset(&ogm_msg_.ogm[index+ogm_msg_.ogmwidth_cell],5,2*sizeof(unsigned char));
////							std::cout<<ogm_msg_.ogm[index]<<" "<<ogm_msg_.ogm[index+ogm_msg_.ogmwidth_cell]<<std::endl;
////							ogm_msg_.ogm[index]=5;;
////							ogm_msg_.ogm[index+1]=5;
//							index+=2*ogm_msg_.ogmwidth_cell;			//在图像和栅格坐标系下分别是+= 和-=
//						}
////										std::cout<<"........................"<<std::endl;
//						vecup_.push_back(point_count_ogm_.ogmheight_cell-i-1);
//						vecup_.push_back(j);
//						vecup_.push_back(count);
//
//					}
//				}
//				if(count>0) i+=count-1;
//
//			}
			for(int i=20/point_count_ogm_.ogmresolution+6/point_count_ogm_.ogmresolution;i<50/point_count_ogm_.ogmresolution;i++)
						{
							int thresh;
							if(i<45) thresh=GRID_THRESH;
							else if(i<55) thresh=GRID_THRESH2;
							else thresh=GRID_THRESH3;
							int count=0;
							if(j>point_count_ogm_.ogmwidth_cell/2-3/point_count_ogm_.ogmresolution&&j<point_count_ogm_.ogmwidth_cell/2+3/point_count_ogm_.ogmresolution
									&&i<point_count_ogm_.ogmheight_cell/2+2/point_count_ogm_.ogmresolution&&i>point_count_ogm_.ogmheight_cell/2-4/point_count_ogm_.ogmresolution)
								continue;
							int index=i*point_count_ogm_.ogmwidth_cell+j;
							while(point_count_ogm_.ogm[index]==0&&count<6&&count<50/point_count_ogm_.ogmresolution-i)//&&count<point_count_ogm_.ogmheight_cell-i
							{
								count++;
								index+=point_count_ogm_.ogmwidth_cell;
							}
//							index=i*point_count_ogm_.ogmwidth_cell+j;//复位zx
							index=i*4*ogm_msg_.ogmwidth_cell+j*4;
//							int index_img=(point_count_ogm_.ogmheight_cell-i-1)*point_count_ogm_.ogmwidth_cell+j;//这是在图像上的索引，用于测试
							if(count>thresh)								//可调参数
							{
								if(pathClear(4*i,4*j))	//判断一下由雷达到无点区域中点之间有无障碍  pathClear(j,i+count/2)
								{
									for(int k=0;k<count;k++)
									{
//										ogm_msg_.ogm[index]=1;
//										index+=point_count_ogm_.ogmwidth_cell; //在图像和栅格坐标系下分别是+= 和-=
										memset(&ogm_msg_.ogm[index],5,4*sizeof(unsigned char));
										memset(&ogm_msg_.ogm[index+ogm_msg_.ogmwidth_cell],5,4*sizeof(unsigned char));
										memset(&ogm_msg_.ogm[index+2*ogm_msg_.ogmwidth_cell],5,4*sizeof(unsigned char));
										memset(&ogm_msg_.ogm[index+3*ogm_msg_.ogmwidth_cell],5,4*sizeof(unsigned char));
//										std::cout<<ogm_msg_.ogm[index]<<" "<<ogm_msg_.ogm[index+ogm_msg_.ogmwidth_cell]<<std::endl;
										//							ogm_msg_.ogm[index]=5;;
										//							ogm_msg_.ogm[index+1]=5;
										index+=4*ogm_msg_.ogmwidth_cell;			//在图像和栅格坐标系下分别是+= 和-=
									}
			//										std::cout<<"........................"<<std::endl;
									vecup_.push_back(point_count_ogm_.ogmheight_cell-i-1);
									vecup_.push_back(j);
									vecup_.push_back(count);

								}
							}
							if(count>0) i+=count-1;

						}
		}


	}
	void PostProcess::countogmpoints(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
	{
		memset(point_count_ogm_.ogm , 0 , point_count_ogm_.ogmcell_size*sizeof(int));//每次都要置零,避免填充错乱
		memset(maxz_ogm_.ogm , 0 , maxz_ogm_.ogmcell_size*sizeof(int));
//		memset(ogm_msg_.ogm,0,ogm_msg_.ogmcell_size*sizeof(unsigned int));
		memset(ogm_msg_.ogm,0,ogm_msg_.ogmcell_size*sizeof(unsigned char));

		float ogm_y_offset = 20.0f;
		//为每个栅格赋值点云数量
		for (int i = 0; i < cloud->points.size(); i++)
		{

		            float x = cloud->points[i].x,
		                  y = cloud->points[i].y,
		                  z = cloud->points[i].z;

		            float newy = y + ogm_y_offset;//ogm_y_offset
//					if(cloud->points[i].intensity < 15.0f)//排除噪声干扰，只考虑反射率较高的点
//					                continue;
					if(x>-1.6&&x<1.6&&y>-1&&y<3){		//排除车身周围
						continue;
					}

		            if((x >=-point_count_ogm_.ogmwidth / 2  && x <= point_count_ogm_.ogmwidth / 2) &&//这里的条件是否需要再考虑一下
		                    (newy >=0 && newy < point_count_ogm_.ogmheight) &&
		                    ( z <=  Z_MAX))
		            {
		                int col = boost::math::round(x / point_count_ogm_.ogmresolution) + ( point_count_ogm_.ogmwidth_cell - 1 ) / 2;
		                int row = boost::math::round(newy / point_count_ogm_.ogmresolution) ;





		            if((row >=0 && row < point_count_ogm_.ogmheight_cell)
		                                  && (col >=0 && col < point_count_ogm_.ogmwidth_cell))
		            {
		            	int index = row * point_count_ogm_.ogmwidth_cell + col;
		            	point_count_ogm_.ogm[index]++;
		            }
		            }

		}

		//test
//		for(int i=point_count_ogm_.ogmwidth_cell/2;i<point_count_ogm_.ogmwidth_cell/2+6/point_count_ogm_.ogmresolution;i++){
//			int index=20/point_count_ogm_.ogmresolution*point_count_ogm_.ogmwidth_cell+i;
//			std::cout<<point_count_ogm_.ogm[index]<<" ";
//		}
//		std::cout<<std::endl;
		/////////////////////////////////
		//为每个栅格赋值maxz
		for (int i = 0; i < cloud->points.size(); i++)
		{
			float x = cloud->points[i].x,
			      y = cloud->points[i].y,
			      z = cloud->points[i].z;
			float newy = y + ogm_y_offset;
//			if(cloud->points[i].intensity < 15.0f)//排除噪声干扰，只考虑反射率较高的点
//			                continue;
			if(x>-1.5&&x<1.5&&y>-1&&y<4){		//排除车身周围------------------------------------zx
				continue;
			}

//			            newy = y + maxz_ogm_.ogmheight/2;
			            if((x >=-maxz_ogm_.ogmwidth / 2  && x <= maxz_ogm_.ogmwidth / 2) &&
			                    (newy >=0 && newy < maxz_ogm_.ogmheight) &&
			                    (z >= - 2 && z <=  Z_MAX))
			            {
			                int col = boost::math::round(x / maxz_ogm_.ogmresolution) + ( maxz_ogm_.ogmwidth_cell - 1 ) / 2;
			                int row = boost::math::round(newy / maxz_ogm_.ogmresolution) ;

			                if((row >=0 && row < maxz_ogm_.ogmheight_cell)					//加了_cell的才是栅格zx
			                        && (col >=0 && col < maxz_ogm_.ogmwidth_cell))
			                {
			                    int index = row * maxz_ogm_.ogmwidth_cell + col;
			                    if( maxz_ogm_.ogm[index] < z)
			                        maxz_ogm_.ogm[index] = z;
			                }
			            }
		}
		///test
//		for(int i=maxz_ogm_.ogmwidth_cell/2;i<maxz_ogm_.ogmwidth_cell/2+15/maxz_ogm_.ogmresolution;i++){
//				int index=35/maxz_ogm_.ogmresolution*maxz_ogm_.ogmwidth_cell+i;
//				std::cout<<maxz_ogm_.ogm[index]<<" ";
//			}
//			std::cout<<std::endl;
//测试代码,看每个格子的点数/高度
//		for (int i=20;i<30;i++)
//		{
//			for(int j=20;j<40;j++)//车体右侧
//				{
////				if(maxz_ogm_.ogm[i*point_count_ogm_.ogmwidth_cell+j]>1){
//				std::cout<<".."<<maxz_ogm_.ogm[i*point_count_ogm_.ogmwidth_cell+j];
////				cout<<".."<<j<<".."<<i<<endl;
////				}
//			}
//			std::cout<<std::endl;
//		}
//		for(int i=13;i<21;i++){
//			std::cout<<point_count_ogm_.ogm[35*point_count_ogm_.ogmwidth_cell+i]<<"  ";
//		}
//		std::cout<<std::endl;
		//下面将考虑通过检测比较大片的无点区域判断是否为悬崖zx
		for(int i=20/point_count_ogm_.ogmresolution+3;i<45/point_count_ogm_.ogmresolution;i++)//
		{
//			std::cout<<"......11111111111111.................."<<std::endl;
			int end_right=point_count_ogm_.ogmwidth_cell/2+16/point_count_ogm_.ogmresolution;
			for(int j=point_count_ogm_.ogmwidth_cell/2+4/point_count_ogm_.ogmresolution;j<end_right;j++)//车体右侧
			{

				int count=0;
				int index=i*point_count_ogm_.ogmwidth_cell+j;
															//用于记录连续没有点云的栅格数
								while(point_count_ogm_.ogm[index]<6&&count<end_right-j)//0221这里的10之前是16zx
								{
//									std::cout<<"......11111111111111.................."<<std::endl;
									count++;
									index++;
								}
//								int tempindex=index;
//								int index_img=(point_count_ogm_.ogmheight_cell-i-1)*point_count_ogm_.ogmwidth_cell+j;//这是在图像上的索引，用于测试
								index=4*i*ogm_msg_.ogmwidth_cell+j*4;//栅格地图上悬崖起始点索引
								int index2=(4*i+1)*ogm_msg_.ogmwidth_cell+4*j;
								int index3=(4*i+2)*ogm_msg_.ogmwidth_cell+4*j;
								int index4=(4*i+3)*ogm_msg_.ogmwidth_cell+4*j;
								if(count>GRID_THRESH)								//可调参数
								{
									float thresh_flat=0,maxz1=-10,maxz2=-10;//用来排除地面点云间隙造成的误检
									if(j+count==end_right-1){thresh_flat=-1;}//如果无点区域直接出界
									else{
										for(int k=4*i;k<4*i+4;k++){
											for(int l=4*j-4;l<4*j;l++){
												int small_i=k*maxz_ogm_.ogmwidth_cell+l;
												if(maxz_ogm_.ogm[small_i]>maxz1){
													maxz1=maxz_ogm_.ogm[small_i];
													if(abs(maxz1)>0.1) break;
												}
											}
											if(abs(maxz1)>0.1) break;
										}
										for(int k=4*i;k<4*i+4;k++){
											for(int l=4*(j+count);l<4*(j+count+1);l++){
												int small_i=k*maxz_ogm_.ogmwidth_cell+l;
												if(maxz_ogm_.ogm[small_i]>maxz2){
													maxz2=maxz_ogm_.ogm[small_i];
													if(abs(maxz2)>0.1) break;
												}
											}
											if(abs(maxz2)>0.1) break;
										}
										thresh_flat=(maxz1!=0&&maxz2!=0)?FLAT_THRESH:(-1);
									}
									if((pathClear(4*i,j*4)&&pathClear((i+count/2)*4,4*j))&&pathClear((i+count)*4+3,4*j)&&abs(maxz1-maxz2)>thresh_flat)	//判断一下由雷达到无点区域中点之间有无障碍
									{
//										std::cout<<"<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>"<<std::endl;
										memset(&ogm_msg_.ogm[index],5,count*4*sizeof(unsigned char));
										memset(&ogm_msg_.ogm[index2],5,count*4*sizeof(unsigned char));
										memset(&ogm_msg_.ogm[index3],5,count*4*sizeof(unsigned char));
										memset(&ogm_msg_.ogm[index4],5,count*4*sizeof(unsigned char));

//										for(int d=0;d<count;d++)
//										{
//											std::cout<<ogm_msg_.ogm[index+d]<<"  ";
//										}
//										std::cout<<"........................"<<std::endl;
										vecright_.push_back(point_count_ogm_.ogmheight_cell-i-1);
										vecright_.push_back(j);
										vecright_.push_back(count);

									}
								}
								if(count>0)
								j+=count-1;
			}

			for(int j=point_count_ogm_.ogmwidth_cell/2-20/point_count_ogm_.ogmresolution;j<point_count_ogm_.ogmwidth_cell/2-4/point_count_ogm_.ogmresolution;j++)//车体左侧
			{

				int count=0;
				int index=i*point_count_ogm_.ogmwidth_cell+j;
															//用于记录连续没有点云的栅格数
								while(point_count_ogm_.ogm[index]<6&&count<(point_count_ogm_.ogmwidth_cell/2-4/point_count_ogm_.ogmresolution-j))//0221这里的10之前是16zx
								{
//									std::cout<<"................"<<count+i<<std::endl;
									count++;
									index++;

								}
								int tempindex=index;
//								int index_img=(point_count_ogm_.ogmheight_cell-i-1)*point_count_ogm_.ogmwidth_cell+j;//这是在图像上的索引，用于测试
								index=4*i*ogm_msg_.ogmwidth_cell+j*4;//栅格地图上悬崖起始点索引
								int index2=(4*i+1)*ogm_msg_.ogmwidth_cell+4*j;
								int index3=(4*i+2)*ogm_msg_.ogmwidth_cell+4*j;
								int index4=(4*i+3)*ogm_msg_.ogmwidth_cell+4*j;
								if(count>GRID_THRESH)								//可调参数
								{
									float thresh_flat=0,maxz1=-10,maxz2=-10;//用来排除地面点云间隙造成的误检
									if(j==point_count_ogm_.ogmwidth_cell/2-20/point_count_ogm_.ogmresolution){thresh_flat=-1;}
									else{
										for(int k=4*i;k<4*i+4;k++){
											for(int l=4*j-1;l>4*j-5;l--){
												int small_i=k*maxz_ogm_.ogmwidth_cell+l;
												if(maxz_ogm_.ogm[small_i]>maxz1){
													maxz1=maxz_ogm_.ogm[small_i];
													if(abs(maxz1)>0.1) break;
												}
											}
											if(abs(maxz1)>0.1) break;
										}
										for(int k=4*i;k<4*i+4;k++){
											for(int l=4*(j+count);l<4*(j+count+1);l++){
												int small_i=k*maxz_ogm_.ogmwidth_cell+l;
												if(maxz_ogm_.ogm[small_i]>maxz2){
													maxz2=maxz_ogm_.ogm[small_i];
													if(abs(maxz2)>0.1) break;
												}
											}
											if(abs(maxz2)>0.1) break;
										}
										thresh_flat=(maxz1!=0&&maxz2!=0)?FLAT_THRESH:(-1);
									}
									if((pathClear(4*i,j*4)&&pathClear((i+count/2)*4,4*j))&&pathClear((i+count)*4+3,4*j)&&abs(maxz1-maxz2)>thresh_flat)	//&&abs(maxz1-maxz2)>thresh_flat
									{
//										cout<<"the maxz is  "<<maxz2<<"   "<<maxz1<<endl;
//										cout<<"absolute value is "<<abs(maxz1-maxz2)<<endl;
										memset(&ogm_msg_.ogm[index],5,count*4*sizeof(unsigned char));
										memset(&ogm_msg_.ogm[index2],5,count*4*sizeof(unsigned char));
										memset(&ogm_msg_.ogm[index3],5,count*4*sizeof(unsigned char));
										memset(&ogm_msg_.ogm[index4],5,count*4*sizeof(unsigned char));
//										for(int d=0;d<count;d++)
//										{
//											std::cout<<ogm_msg_.ogm[index+d]<<"  ";
//										}
//										std::cout<<"........................"<<std::endl;
										vecright_.push_back(point_count_ogm_.ogmheight_cell-i-1);
										vecright_.push_back(j);
										vecright_.push_back(count);

									}
								}
								if(count>0)
									j+=count-1;


			}
//
		}
		//test
//		for(int i=13;i<21;i++){
//			int index=35*point_count_ogm_.ogmwidth_cell+i;
//			int count=0;
//			while(point_count_ogm_.ogm[index]==0){
//				count++;
//				index++;
//			}
//			if(count>2){
//				if(pathClear(35,i)){
//					std::cout<<"yesyesyes"<<std::endl;
//				}
//				else cout<<"nonono"<<std::endl;
//			}
//		}
		//test done
		for(int j=point_count_ogm_.ogmwidth_cell/2-4/point_count_ogm_.ogmresolution;j<point_count_ogm_.ogmwidth_cell/2+4/point_count_ogm_.ogmresolution;j++)//车体上下范围
		{

//			for(int i=0;i<20/point_count_ogm_.ogmresolution;i++)//-3/point_count_ogm_.ogmresolution
//			{
//				int count=0;
//				//排除车体周围范围
//				if(j>point_count_ogm_.ogmwidth_cell/2-3/point_count_ogm_.ogmresolution&&j<point_count_ogm_.ogmwidth_cell/2+3/point_count_ogm_.ogmresolution
//						&&i<point_count_ogm_.ogmheight_cell/2+2/point_count_ogm_.ogmresolution&&i>point_count_ogm_.ogmheight_cell/2-4/point_count_ogm_.ogmresolution)
//					continue;
//				int index=i*point_count_ogm_.ogmwidth_cell+j;
//				while(point_count_ogm_.ogm[index]==0&&count<6&&count<point_count_ogm_.ogmheight_cell/2-4/point_count_ogm_.ogmresolution-i)//-point_count_ogm_.ogmheight_cell/2//&&count<point_count_ogm_.ogmheight_cell-i
//				{
//					count++;
//					index+=point_count_ogm_.ogmwidth_cell;
//				}
//				index=i*2*ogm_msg_.ogmwidth_cell+j*2;//这是在栅格坐标系下的索引zx
////				int index_img=(point_count_ogm_.ogmheight_cell-i-1)*point_count_ogm_.ogmwidth_cell+j;//这是在图像上的索引，用于测试
//				if(count>GRID_THRESH)								//可调参数
//				{
//					if(pathClear(i,j))	//判断一下由雷达到无点区域中点之间有无障碍  pathClear(j,i+count/2)
//					{
//						for(int k=0;k<count;k++)
//						{
//							memset(&ogm_msg_.ogm[index],5,2*sizeof(unsigned char));
//							memset(&ogm_msg_.ogm[index+ogm_msg_.ogmwidth_cell],5,2*sizeof(unsigned char));
////							std::cout<<ogm_msg_.ogm[index]<<" "<<ogm_msg_.ogm[index+ogm_msg_.ogmwidth_cell]<<std::endl;
////							ogm_msg_.ogm[index]=5;;
////							ogm_msg_.ogm[index+1]=5;
//							index+=2*ogm_msg_.ogmwidth_cell;			//在图像和栅格坐标系下分别是+= 和-=
//						}
////										std::cout<<"........................"<<std::endl;
//						vecup_.push_back(point_count_ogm_.ogmheight_cell-i-1);
//						vecup_.push_back(j);
//						vecup_.push_back(count);
//
//					}
//				}
//				if(count>0) i+=count-1;
//
//			}
			for(int i=20/point_count_ogm_.ogmresolution+6/point_count_ogm_.ogmresolution;i<50/point_count_ogm_.ogmresolution;i++)
						{
							int thresh;
							if(i<45) thresh=GRID_THRESH;
							else if(i<55) thresh=GRID_THRESH2;
							else thresh=GRID_THRESH3;
							int count=0;
							if(j>point_count_ogm_.ogmwidth_cell/2-3/point_count_ogm_.ogmresolution&&j<point_count_ogm_.ogmwidth_cell/2+3/point_count_ogm_.ogmresolution
									&&i<point_count_ogm_.ogmheight_cell/2+2/point_count_ogm_.ogmresolution&&i>point_count_ogm_.ogmheight_cell/2-4/point_count_ogm_.ogmresolution)
								continue;
							int index=i*point_count_ogm_.ogmwidth_cell+j;
							while(point_count_ogm_.ogm[index]==0&&count<6&&count<50/point_count_ogm_.ogmresolution-i)//&&count<point_count_ogm_.ogmheight_cell-i
							{
								count++;
								index+=point_count_ogm_.ogmwidth_cell;
							}
//							index=i*point_count_ogm_.ogmwidth_cell+j;//复位zx
							index=i*4*ogm_msg_.ogmwidth_cell+j*4;
//							int index_img=(point_count_ogm_.ogmheight_cell-i-1)*point_count_ogm_.ogmwidth_cell+j;//这是在图像上的索引，用于测试
							if(count>thresh)								//可调参数
							{
								if(pathClear(4*i,4*j))	//判断一下由雷达到无点区域中点之间有无障碍  pathClear(j,i+count/2)
								{
									for(int k=0;k<count;k++)
									{
//										ogm_msg_.ogm[index]=1;
//										index+=point_count_ogm_.ogmwidth_cell; //在图像和栅格坐标系下分别是+= 和-=
										memset(&ogm_msg_.ogm[index],5,4*sizeof(unsigned char));
										memset(&ogm_msg_.ogm[index+ogm_msg_.ogmwidth_cell],5,4*sizeof(unsigned char));
										memset(&ogm_msg_.ogm[index+2*ogm_msg_.ogmwidth_cell],5,4*sizeof(unsigned char));
										memset(&ogm_msg_.ogm[index+3*ogm_msg_.ogmwidth_cell],5,4*sizeof(unsigned char));
//										std::cout<<ogm_msg_.ogm[index]<<" "<<ogm_msg_.ogm[index+ogm_msg_.ogmwidth_cell]<<std::endl;
										//							ogm_msg_.ogm[index]=5;;
										//							ogm_msg_.ogm[index+1]=5;
										index+=4*ogm_msg_.ogmwidth_cell;			//在图像和栅格坐标系下分别是+= 和-=
									}
			//										std::cout<<"........................"<<std::endl;
									vecup_.push_back(point_count_ogm_.ogmheight_cell-i-1);
									vecup_.push_back(j);
									vecup_.push_back(count);

								}
							}
							if(count>0) i+=count-1;

						}
		}


	}
	bool  PostProcess::pathClear(int height,int width)
	{
		int height0=20/maxz_ogm_.ogmresolution,width0=20/maxz_ogm_.ogmresolution;//起点坐标zx
		float k=(float)(height-height0)/(width-width0);
		float d=-0.5;
		int flag=0;
		//test
//		int heighttest=55,widthtest=32;
//		float ktest=(float)(heighttest-height0)/(widthtest-width0);
//		while(height0<heighttest)
//		{
////			if(maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0]>PATHTHRESH||maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0+1]>PATHTHRESH||maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0-1]>PATHTHRESH){
////				return false;
////			}
//			std::cout<<maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0]<<" ";
//			height0++;
//			d+=1/ktest;
//			if(d>0){width0++;d-=1;}
//		}
//		std::cout<<endl;
		//test done
		//if(width0>width||height0>height||k>1) return false;//暂时先只判断车体右侧
		if(width0<=width&&height0<=height&&0<k&&k<=1){flag=1;}
		if(width0<=width&&height0<=height&&k>1){ flag=2;}
		if(width0>=width&&height0<=height&&k<=-1){flag=3;}
		if(width0>=width&&height0<=height&&k>-1&&k<0){flag=4;}
		if(width0>=width&&height0>=height&&k>0&&k<1){flag=5;}
		if(width0>=width&&height0>=height&&k>=1){ flag=6;}
		if(width0<=width&&height0>height&&k<=-1){flag=7;}
		if(width0<=width&&height0>height&&k>-1&&k<0) {flag=8;}
		if(width<width0&&k==0){flag=9;}
		if(width>width0&&k==0){flag=10;}
//		std::cout<<"....."<<flag<<"..."<<height0<<"..."<<height<<"..."<<width0<<"..."<<width<<"...."<<k<<std::endl;
		int height_next=0;
		int width_next=0;
		switch (flag){

			case 1:
			while(width0<width)
			{

     		if(maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0]>PATHTHRESH||maxz_ogm_.ogm[(height0+1)*maxz_ogm_.ogmwidth_cell+width0]>PATHTHRESH||maxz_ogm_.ogm[(height0-1)*maxz_ogm_.ogmwidth_cell+width0]>PATHTHRESH)
				{return false;}//判断无数点是否由于障碍物产生zx,此处阈值可更改
     		width0++;
     		d+=k;
     		if(d>0) {height0++;d-=1;}
			}return true;
//				return true;
			case 2:
//			std::cout<<"..."<<"this is case2"<<std::endl;
			while(height0+2<height)
			{
				if(maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0]>PATHTHRESH||maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0+1]>PATHTHRESH||maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0-1]>PATHTHRESH){
					return false;
				}
				height0++;
				d+=1/k;
				if(d>0){width0++;d-=1;}
			}return true;
			case 3:
			while(height0<height)
			{
				if(maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0]>PATHTHRESH||maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0+1]>PATHTHRESH||maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0-1]>PATHTHRESH){
									return false;
								}
				height0++;
				d-=1/k;
				if(d>0) {width0--;d-=1;}
			}return true;
			case 4:
			while(width<width0)
			{

				if(maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0]>PATHTHRESH||maxz_ogm_.ogm[(height0+1)*maxz_ogm_.ogmwidth_cell+width0]>PATHTHRESH||maxz_ogm_.ogm[(height0-1)*maxz_ogm_.ogmwidth_cell+width0]>PATHTHRESH){
													return false;
												}
				width0--;
				d-=k;
				if(d>0) {height0++;d-=1;}

			}return true;
			case 5:
			while(width<width0)
			{

				if(maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0]>PATHTHRESH||maxz_ogm_.ogm[(height0+1)*maxz_ogm_.ogmwidth_cell+width0]>PATHTHRESH||maxz_ogm_.ogm[(height0-1)*maxz_ogm_.ogmwidth_cell+width0]>PATHTHRESH){
										return false;}
				width0--;
				d+=k;
				if(d>0){height0--;d-=1;}
			}return true;
			case 6:
			while(height<height0)
			{
				if(maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0]>PATHTHRESH||maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0+1]>PATHTHRESH||maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0-1]>PATHTHRESH){
											return false;}
											height0--;
											d+=1/k;
											if(d>0) {width0--;d-=1;}

			}return true;
			case 7:
//				std::cout<<"..."<<"this is case8"<<std::endl;
			while(height<height0)
			{
				int index=height0*maxz_ogm_.ogmwidth_cell+width0;
//				std::cout<<maxz_ogm_.ogm[index]<<" ";
				if(maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0]>PATHTHRESH||maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0+1]>PATHTHRESH||maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0-1]>PATHTHRESH){
											return false;}
				height0--;
				d-=1/k;
				if(d>0){width0++;d-=1;}

			}return true;//std::cout<<std::endl;
			case 8:
			while(width0<width)
			{

				if(maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0]>PATHTHRESH||maxz_ogm_.ogm[(height0+1)*maxz_ogm_.ogmwidth_cell+width0]>PATHTHRESH||maxz_ogm_.ogm[(height0-1)*maxz_ogm_.ogmwidth_cell+width0]>PATHTHRESH){
															return false;}
				width0++;
				d-=k;
				if(d>0){height0--;d-=1;}


			}return true;
			case 9:
			while(width<width0)
			{
				if(maxz_ogm_.ogm[height*maxz_ogm_.ogmwidth_cell+width]>PATHTHRESH){
															return false;}
				width++;
			}return true;
			case 10:
			while(width>width0)
			{
				if(maxz_ogm_.ogm[height*maxz_ogm_.ogmwidth_cell+width]>PATHTHRESH){
					return false;}
				width--;
			}return true;
		}

	}
//	std::condition_variable con;
	void PostProcess::showOGM(const char* windowname ,const OGMData<int>& ogmdata,vector<int> vecside,vector<int> vecupdown)
	{
		cvNamedWindow(windowname,0);
		IplImage *slopemat = cvCreateImage(cvSize(ogmdata.ogmwidth_cell,ogmdata.ogmheight_cell),IPL_DEPTH_8U,1);
		cvZero(slopemat);
		int heightnum = ogmdata.ogmheight_cell;
		int widthnum = ogmdata.ogmwidth_cell;
		for(int j=0;j<ogmdata.ogmheight_cell;j++)
		{
			unsigned char* pdata = (unsigned char*)(slopemat->imageData + (ogmdata.ogmheight_cell - 1 - j)* slopemat->widthStep);
			for(int i=0 ;i < ogmdata.ogmwidth_cell ; i++)
			{
				unsigned char val = ogmdata.ogm[i + j*ogmdata.ogmwidth_cell];//val为每个栅格的点云数量
				if(val > 0)
				{
					pdata[i]=abs(val*10);//以val×10作为像素值

				}

			}

		}
////test
//		for(int i=0;i<6;i++){
//			cvLine(slopemat,cvPoint(0,63-i*10),cvPoint(50,63-i*10),cvScalar(255));
//		}
//		矩形
//			cvRectangle(slopemat,cvPoint(30,30),cvPoint(35,35),cvScalar(255));
//		矩形
//		cvLine(slopemat,cvPoint(0,87-45),cvPoint(50,87-45),cvScalar(255));
			int height0=25,width0=25;
			int height=64,width=15;
//			cvLine(slopemat,cvPoint(width0,height0),cvPoint(width,height),cvScalar(255));
			float k=(float)(height-height0)/(width-width0);
			float d=-0.5;
			while(height>height0)
			{
				unsigned char* pdata = (unsigned char*)(slopemat->imageData + (87-height0-1)* slopemat->widthStep);
				pdata[width0]=255;
//				if(maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0]>PATHTHRESH||maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0+1]>PATHTHRESH||maxz_ogm_.ogm[height0*maxz_ogm_.ogmwidth_cell+width0-1]>PATHTHRESH){
//					return false;}
				height0++;
				d-=1/k;
				if(d>0){width0--;d-=1;}

			}
////test done
//			std::cout<<ogmdata.ogmwidth_cell/2<<std::endl;
    		cvLine(slopemat,cvPoint(0,ogmdata.ogmheight_cell-1-20/ogmdata.ogmresolution),cvPoint(ogmdata.ogmwidth_cell,ogmdata.ogmheight_cell-1-20/ogmdata.ogmresolution),cvScalar(255));
	    	cvLine(slopemat,cvPoint(ogmdata.ogmwidth_cell/2,0),cvPoint(ogmdata.ogmwidth_cell/2,ogmdata.ogmheight_cell),cvScalar(255));

	    	cvLine(slopemat,cvPoint(ogmdata.ogmwidth_cell/2-4/ogmdata.ogmresolution,0),cvPoint(ogmdata.ogmwidth_cell/2-4/ogmdata.ogmresolution,170),cvScalar(255));
	    	cvLine(slopemat,cvPoint(ogmdata.ogmwidth_cell/2+4/ogmdata.ogmresolution,0),cvPoint(ogmdata.ogmwidth_cell/2+4/ogmdata.ogmresolution,170),cvScalar(255));
//	    	cvLine(slopemat,cvPoint(ogmdata.ogmwidth_cell/2+3/ogmdata.ogmresolution,0),cvPoint(ogmdata.ogmwidth_cell/2+3/ogmdata.ogmresolution,99),cvScalar(255));
//	    	cvLine(slopemat,cvPoint(ogmdata.ogmwidth_cell/2-3/ogmdata.ogmresolution,0),cvPoint(ogmdata.ogmwidth_cell/2-3/ogmdata.ogmresolution,99),cvScalar(255));
//	    	cvLine(slopemat,cvPoint(0,ogmdata.ogmwidth_cell/2+6/ogmdata.ogmresolution),cvPoint(99,ogmdata.ogmwidth_cell/2+6/ogmdata.ogmresolution),cvScalar(255));
//	    	cvLine(slopemat,cvPoint(0,ogmdata.ogmwidth_cell/2-5/ogmdata.ogmresolution),cvPoint(99,ogmdata.ogmwidth_cell/2-5/ogmdata.ogmresolution),cvScalar(255));
//test
	    for(int i=0;i<vecside.size();)
	    {
//	    	std::cout<<".............................."<<std::endl;
	    	cvLine(slopemat,cvPoint(vecside[i+1],vecside[i]),cvPoint(vecside[i+1]+vecside[i+2],vecside[i]),cvScalar(255));
	    	i+=3;
	    }
	    for(int i=0;i<vecupdown.size();)
	    {
//	    	std::cout<<".............................."<<std::endl;
	    	cvLine(slopemat,cvPoint(vecupdown[i+1],vecupdown[i]),cvPoint(vecupdown[i+1],vecupdown[i]+vecupdown[i+2]),cvScalar(255));
	    	i+=3;
	    }


	    cvShowImage(windowname,slopemat);
	    cvWaitKey(10);
	    cvReleaseImage(&slopemat);

	}
	void PostProcess::radiusDetection(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
	{
//		std::cout<<"radiusdetectionenterrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr"<<std::endl;
		int col_count = cloud->size()/LASER_LAYER;
//		float x_thresh;
//		float y_thresh;
		float z_thresh=-0.1;//TODO:根据不同距离设置不同到阈值
		float x_diff=0,y_diff=0,z_diff=0;//这个声明在外面会更好一些吗？zx
		for(int i=0;i<col_count;i++)
		{
//			std::cout<<"radiusdetectionenterrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr"<<std::endl;
			for(int j=0;j<40;j++)//一组到64或32根线处理
			{
//				std::cout<<"radiusdetectionenterrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr"<<std::endl;
				int actual_num=indexmaptable[j].number;
				int actual_num_next=indexmaptable[j+1].number;//获取点对的index方便索引
				int actual_index=i*LASER_LAYER+actual_num;
				int actual_index_next=i*LASER_LAYER+actual_num_next;
				float x1=cloud->points[actual_index].x;
				float x2=cloud->points[actual_index_next].x;
//				x_diff=x1-x2;
				y_diff=(cloud->points[actual_index_next].y-cloud->points[actual_index].y);
				x_diff=(cloud->points[actual_index_next].x-cloud->points[actual_index].x);
				z_diff=(cloud->points[actual_index_next].z-cloud->points[actual_index].z);//获取点三个坐标的变化
//				std::cout<<".........."<<y_diff<<".........."<<x_diff<<".........."<<z_diff<<".........."<<std::endl;
//				printf("................%f",x_diff);
				if(/*x_diff>x_thresh||y_diff>y_thresh||*/z_diff<z_thresh)
				{
//					std::cout<<"radiusdetectionenterrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr"<<std::endl;
					stiffcloud_->points.push_back(cloud->points[actual_index]);
				}
			}
		}
	}
	void PostProcess::radiusPointpair(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
	{
		int col_count = cloud->size() / LASER_LAYER;
		float x_thresh=6;
		float y_thresh=6;
		float z_thresh=1.2;
		float x1,x2,y1,y2,z1,z2,x_diff,y_diff,z_diff;
		for(int i=0;i<55;i++)	//防止溢出，另外也用不了64根线
		{
			int actual_i=indexmaptable[i].number;
			for(int j=0;j<col_count;j++)
			{
				int index1=j*LASER_LAYER+actual_i;
				int index2=index1+LASER_LAYER;
				x1=cloud->points[index1].x;
				x2=cloud->points[index2].x;
				y1=cloud->points[index1].y;
				y2=cloud->points[index2].y;
				z1=cloud->points[index1].z;
				z2=cloud->points[index2].z;
				x_diff=abs(x2-x1);
				y_diff=abs(y2-y1);
				z_diff=abs(z2-z1);

				if(x_diff>x_thresh&&y_diff>y_thresh&&z_diff>z_thresh)
				{
					stiffcloud_->points.push_back(cloud->points[index1]);
					stiffcloud_->points.push_back(cloud->points[index2]);
				}
			}
		}
	}
	void PostProcess::circleradiusDetection(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
	{

		int col_count = cloud->size() / LASER_LAYER;
	    float delta_dis_thresh = 2;//adjustable param
	    float delta_dis_thresh2=delta_dis_thresh/2;
	    int max_min_counter_thresh=2;
	    int cloud_window_size = 10;//adjustable param cloud_window_size
//	    for(int i=0;i<64;i++)
//	    {
//	    	for(int j=0;j<64;j++)
//	    	{
//	    		stiffcloud_->points.push_back(cloud->points[i*64+j]);
//	    	}
//	    }

	    for(int dis_i = 0 ; dis_i < LASER_LAYER ; dis_i++)
	    {
	        if(theorydis[dis_i] > 40 || theorydis[dis_i]<0)				//理论值吗?zx
	            continue;
	        //float verticalangle = laserverticalangle[i] * M_PI / 180 + beta_h;
	        //float verticalangle=indexmaptable[dis_i].angle*M_PI/180+beta_h;			//incexmaptable中的angle是否是垂直角度?
	        int i=indexmaptable[dis_i].number;

	        float temptheorydis=theorydis[dis_i];
	        float temptheoryz=0;
	        bool flag_matchtheory=false;
	        // if(verticalangle < verticalangle_thresh)
	        {

	            float disminold=100;
	            float dismaxold=-100;
	            int disminposold=0;
	            int dismaxposold=0;
	            for(int j = 0 ; j < col_count - cloud_window_size; j+=2)								//这个应该是要在每一圈内做文章?
	            {
	                int ori_index = (j) * LASER_LAYER + i;
	                if(cloud->points[ori_index].y<4&&cloud->points[ori_index].y>-4&&cloud->points[ori_index].x>-2.5&&cloud->points[ori_index].x<2.5) //back exist false detection , so need delete it
	                    continue;

	                if(cloud->points[ori_index].y< -20||cloud->points[ori_index].x< -30||cloud->points[ori_index].x > 30) //back exist false detection , so need delete it
	                    continue;



	                float dismin=100;
	                float dismax=-100;
	                int disminpos=0;
	                int dismaxpos=0;
	                int window_maxz=-100;
	                int window_minz=100;
	                //get min max
	                for(int window_j=0;window_j<cloud_window_size;window_j+=1)
	                {
	                    int index = (j+ window_j) * LASER_LAYER + i;
	                    if(cloud->points[index].y<4&&cloud->points[index].y>-4&&cloud->points[index].x>-2.5&&cloud->points[index].x<2.5) //back exist false detection , so need delete it
	                        continue;
	                    float temprange=cloud->points[index].range;
	                    float tempz=cloud->points[index].z;
	                    if(temprange < 0.5)
	                        continue;

	                    if(temprange<dismin)
	                    {
	                        dismin=temprange;
	                        disminpos=window_j;
	                    }
	                    if(temprange>dismax)
	                    {
	                        dismax=temprange;
	                        dismaxpos=window_j;
	                    }

	                    if(tempz<window_minz)
	                    {
	                        window_minz=tempz;
	                    }

	                    if(tempz>window_maxz)
	                    {
	                        window_maxz=tempz;
	                    }

	                }
	                //count num
	                if(dismax-dismin>delta_dis_thresh&&dismax-dismin>delta_dis_thresh*0.1*dismin||dismax-dismin>delta_dis_thresh*3)//dismax-dismin>delta_dis_thresh????????zx
	                {
	                    int mincounter=0;
	                    int maxcounter=0;
	                    int zerocounter=0;
	                    for(int window_j=0;window_j<cloud_window_size;window_j++)
	                    {
	                        int index = (j+ window_j) * LASER_LAYER + i;
	                        if(cloud->points[index].y<4&&cloud->points[index].y>-4&&cloud->points[index].x>-2.5&&cloud->points[index].x<2.5) //back exist false detection , so need delete it
	                        {
	                            zerocounter++;
	                            continue;
	                        }
	                        float temprange=cloud->points[index].range;
	                        if(temprange < 0.5)
	                        {
	                            zerocounter++;
	                            continue;

	                        }

	                        if(temprange-dismin<delta_dis_thresh2)
	                        {
	                            mincounter++;
	                        }
	                        if(dismax-temprange<delta_dis_thresh2)
	                        {
	                            maxcounter++;
	                        }



	                    }

	                    if(mincounter>=max_min_counter_thresh&&maxcounter>=max_min_counter_thresh&&zerocounter<max_min_counter_thresh*2)
	                    {
	                        for(int window_j=0;window_j<cloud_window_size;window_j++)
	                        {
	                            int index = (j+ window_j) * LASER_LAYER + i;
	                            float temprange=cloud->points[index].range;
	                            if(temprange-dismin<delta_dis_thresh2)//加了z方向的条件zx20180130
	                            {
	#ifdef USE_OMP
	                                omp_set_lock(&omplock); //获得互斥器
	#endif
	                                cloud->points[index].passibility = 0.0;
	                                stiffcloud_->points.push_back(cloud->points[index]);
	#ifdef USE_OMP
	                                omp_unset_lock(&omplock); //释放互斥器
	#endif
	                            }
	                        }
	                    }
	                }


	            }
	        }
	    }
	    cloudupdate=true;
	}
	void PostProcess::process()
	{

		while(!processthreadfinished_)
		{


			const sensor_msgs::PointCloud2ConstPtr cloudmsg = lidarCloudMsgs_.PopWithTimeout(common::FromSeconds(0.1));
			if(cloudmsg == nullptr)
				continue;
			double timeLaserCloudFullRes = cloudmsg->header.stamp.toSec();
			pcl::PointCloud<pcl::PointXYZI>::Ptr tempcloud(new pcl::PointCloud<pcl::PointXYZI>);//当前帧点云（雷达里程计坐标系）
			pcl::fromROSMsg(*cloudmsg, *tempcloud);//获取当前帧点云数据
			std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> outputclouds;
			std::vector<pcl::PointXYZI> lidarpropertys;
//			analysisCloud(tempcloud,outputclouds,lidarpropertys);

//			对齐时间戳 拼接点云
#ifdef FUSE
			if(lworldclouds_.size()>=5)
				lworldclouds_.pop_front();
			vtotalcloud_.clear();
			vtotalcloud_.push_back(tempcloud);//将当前点云赋给totalcloud_
//			totalclouds_->clear();
//			*totalclouds_+=(*tempcloud);
			LOG(INFO)<<"wait lidarodom";
			auto timeposepair = lidarOdoms_.Pop();
			if(timeposepair==nullptr)
				continue;
			if(timeposepair->first-timeLaserCloudFullRes>0.005)
			{
				lidarOdoms_.Push_Front(std::move(timeposepair));
				continue;
			}

			while((timeposepair->first-timeLaserCloudFullRes)<-0.005)
				timeposepair = lidarOdoms_.Pop();

			LOG(INFO)<<"got lidarodom";
			if(timeposepair==nullptr)
				continue;
			if(fabs(timeposepair->first-timeLaserCloudFullRes)<0.005){
				transform::Rigid3d transformodometry = timeposepair->second;
				transform::Rigid3d transformodometry_inverse = transformodometry.inverse();
				int n=1;
				for(std::list<pcl::PointCloud<pcl::PointXYZI>::Ptr>::iterator it = lworldclouds_.begin();it != lworldclouds_.end();it++)
				{
					n++;
					pcl::PointCloud<pcl::PointXYZI>::Ptr tempcloud(new pcl::PointCloud<pcl::PointXYZI>);
					pcl::transformPointCloud(*(*it),*tempcloud,transformodometry_inverse.translation()
							,transformodometry_inverse.rotation());
					vtotalcloud_.push_back(tempcloud) ;
//					*totalclouds_+=(*tempcloud);
				}
//				cout<<"size now is "<<vtotalcloud_.size()<<endl;
				pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZI>);
				pcl::transformPointCloud(*tempcloud,*tmp,transformodometry.translation()
											 ,transformodometry.rotation());
//				pcl::PointCloud<pcl::PointXYZI>::Ptr ttt=tmp;
//				cout<<"n now is "<<n<<endl;
				lworldclouds_.push_back(tmp);
				if(0){
					char cloud_name[50];
					memset( cloud_name, 0 , 50);
					sprintf( cloud_name, "passablecloud");
					ShowViewerCloudPoints(cloud_viewer_, vtotalcloud_,
							cloud_name, 0, 0, 255);
					cloud_viewer_->spinOnce();
				}
//				MyTime mytime;
//				mytime.start();
				countogmpoints(vtotalcloud_);
//				showOGM("grid",point_count_ogm_,vecright_,vecup_);
				vecright_.clear();							//TODO:检测下size
				vecup_.clear();
//				mytime.show_s();
			}

//			对齐时间戳 拼接点云

//			LOG(INFO)<<"cloud num:"<<lidarpropertys.size();
//			for(int i=0;i<lidarpropertys.size();i++)
//			{
//				LOG(INFO)<<i<<"\tpoint num:"<<outputclouds[i]->size();
//				LOG(INFO)<<i<<"\tlidar pos:"<<lidarpropertys[i].x<<" "<<lidarpropertys[i].y<<" "<<lidarpropertys[i].z<<" ";
//				LOG(INFO)<<i<<"\tlidar layernum:"<<int(lidarpropertys[i].intensity);
//			}
//			lidarOdoms_.Pop(); //需要雷达里程计信息时需要，否则可以注释掉
			//就用64线的先

#else
			{
                            if(tempcloud)
                            {

                                //					  	  std::cout<<"the point size is  "<<outputclouds[0]->points.size()<<std::endl;
//                                timer t;

                                char cloud_name[50];
                                memset( cloud_name, 0 , 50);
                                sprintf( cloud_name, "passablecloud");
                                ShowViewerCloudPoints(cloud_viewer_, tempcloud,
                                                      cloud_name, 0, 0, 255);



                                //				            circleradiusDetection(outputclouds[0]);
                                //				            radiusDetection(outputclouds[0]);
                                //					  	  	radiusPointpair(outputclouds[0]);
                                //				            char stiff_name[50];
                                //				            memset( stiff_name, 0 , 50);
                                //		                    sprintf( stiff_name, "stiffcloud");
                                //				            ShowViewerCloudPoints(cloud_viewer_, stiffcloud_,
                                //				            stiff_name, 255, 0, 0);
                                MyTime mytime;
                                mytime.start();
                                countogmpoints(tempcloud);
                                showOGM("grid",point_count_ogm_,vecright_,vecup_);
                                vecright_.clear();							//TODO:检测下size
                                vecup_.clear();


                                cloud_viewer_->spinOnce();//这个一定记得取消注释

                                mytime.show_s ();

                            }
			}
#endif



	}
}



