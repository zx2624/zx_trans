/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012 The MITRE Corporation
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <pcl/console/print.h>
#include <pcl/io/boost.h>
#include <boost/version.hpp>
#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/math/special_functions.hpp>
#include <boost/lambda/lambda.hpp>
#include "myhdl_grabber.h"
#ifdef HAVE_PCAP
#include <pcap.h>
#endif // #ifdef HAVE_PCAP
//UDP发送数据需要的文件
#include <string.h>  
#include <sys/socket.h>  
#include <netinet/in.h>  
#include <arpa/inet.h>  
#include <netdb.h>  
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <ros/package.h>

const boost::asio::ip::address pcl::MyHDLGrabber::HDL_DEFAULT_NETWORK_ADDRESS = boost::asio::ip::address::from_string ("192.168.1.204");//目标的ip地址，最后好像也没用到
double *pcl::MyHDLGrabber::cos_lookup_table_ = NULL;
double *pcl::MyHDLGrabber::sin_lookup_table_ = NULL;

using boost::asio::ip::udp;

///////////////////////////////////////////////////////////////////////////////new
pcl::MyHDLGrabber::MyHDLGrabber (const std::string& correctionsFile,
                             const std::string& pcapFile, int laser_layer, bool masterflag,std::vector<functionPtr>* functions)
  : hdl_data_ ()
  , udp_listener_endpoint_ (HDL_DEFAULT_NETWORK_ADDRESS, HDL_DATA_PORT)
  , source_address_filter_ ()
  , source_port_filter_ (443)
  , hdl_read_socket_service_ ()
  , hdl_read_socket_ (NULL)
  , pcap_file_name_ (pcapFile)
  , queue_consumer_thread_ (NULL)
  , hdl_read_packet_thread_ (NULL)
  , current_sweep_xyzi_ (new pcl::PointCloud<pcl::PointXYZI> ())
  , last_azimuth_ (65000)
  , sweep_xyzi_signal_ ()
  , min_distance_threshold_(0.0)
  , max_distance_threshold_(10000.0)
  , read_packet_thread_paused_(false)
  , masterflag_(masterflag)
  , functions_(functions)
{

        LASER_LAYER = 16;//确定是16线程序
        initialize (correctionsFile);
}

///////////////////////////////////////////////////////////////////////////////new
pcl::MyHDLGrabber::MyHDLGrabber (const boost::asio::ip::address& ipAddress,
                             const unsigned short int port, 
                             const std::string& correctionsFile, int laser_layer,bool masterflag, std::vector<functionPtr>* functions)
  : hdl_data_ ()
  ,port_(port)
  , udp_listener_endpoint_ (ipAddress, port)
  , source_address_filter_ ()
  , source_port_filter_ (443)
  , hdl_read_socket_service_ ()
  , hdl_read_socket_ (NULL)
  , pcap_file_name_ ()
  , queue_consumer_thread_ (NULL)
  , hdl_read_packet_thread_ (NULL)
  , current_sweep_xyzi_ (new pcl::PointCloud<pcl::PointXYZI> ())
  , last_azimuth_ (65000)
  , sweep_xyzi_signal_ ()
  , min_distance_threshold_(0.0)
  , max_distance_threshold_(10000.0)
  , read_packet_thread_paused_(false)
  , masterflag_(masterflag)
  , functions_(functions)
{


      LASER_LAYER = 16;

    initialize (correctionsFile);
}

/////////////////////////////////////////////////////////////////////////////
pcl::MyHDLGrabber::~MyHDLGrabber () throw ()
{
  stop ();

  disconnect_all_slots<sig_cb_RS_hdl_sweep_point_cloud_xyzi> ();//这个RS代表速腾聚创的16线雷达
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::MyHDLGrabber::initialize (const std::string& correctionsFile)//初始化
{
//if(port_==6699)
//  sendmsgtolidar1();//给激光雷达1发送数据激活其工作
//else
//  sendmsgtolidar2();//给激光雷达2发送数据激活其工作
  file.open("grabbertime.txt",std::ios::out);
  if (cos_lookup_table_ == NULL && sin_lookup_table_ == NULL)
  {
    cos_lookup_table_ = static_cast<double *> (malloc (HDL_NUM_ROT_ANGLES * sizeof (*cos_lookup_table_)));
    sin_lookup_table_ = static_cast<double *> (malloc (HDL_NUM_ROT_ANGLES * sizeof (*sin_lookup_table_)));
    for (unsigned int i = 0; i < HDL_NUM_ROT_ANGLES; i++)
    {
      double rad = (M_PI / 180.0) * (static_cast<double> (i) / 100.0);
      cos_lookup_table_[i] = std::cos (rad);
      sin_lookup_table_[i] = std::sin (rad);
    }
  }

  loadCorrectionsFile (correctionsFile);

  //jkj polar detect 2016/10/22


  for(int index=0;index<HDL_MAX_NUM_LASERS;index++)
  {
      if(index<LASER_LAYER)
      {
          double angle=laser_corrections_[index].verticalCorrection;
          //double tan_angle=tan(HDL_Grabber_toRadians(angle));
          map_tanangle_index[angle]=index;     //build mapping
      }
  }
  std::cout<<"lasernum="<<map_tanangle_index.size()<<std::endl;

    int index=0;
  for(std::map<double,int>::iterator iter=map_tanangle_index.begin() ; iter!=map_tanangle_index.end(); iter++)
  {

      indexmaptable[index].number=iter->second;
      indexmaptable[index].angle=iter->first;
      cout<<index<<"\tlaserindex="<<iter->second<<"\tangle="<<iter->first<<"\ttanangle="<<tan(HDL_Grabber_toRadians(iter->first))<<endl;
      index++;
  }
//jkj end

  for (int i = 0; i < HDL_MAX_NUM_LASERS; i++)
  {
    HDLLaserCorrection correction = laser_corrections_[i];
    laser_corrections_[i].sinVertOffsetCorrection = correction.verticalOffsetCorrection
                                       * correction.sinVertCorrection;
    laser_corrections_[i].cosVertOffsetCorrection = correction.verticalOffsetCorrection
                                       * correction.cosVertCorrection;
  }
  sweep_xyzi_signal_ =createSignal<sig_cb_RS_hdl_sweep_point_cloud_xyzi> ();
  current_sweep_xyzi_.reset (new pcl::PointCloud<pcl::PointXYZI>);

  trigger_=false;
  mutilidar_signal_ = createSignal<void()> ();
  if(masterflag_&&functions_)
  {
      for(int i=0;i<functions_->size();i++)
	{
	  if(mutilidar_signal_)
		mutilidar_signal_->connect(*(functions_->at(i).get()));
	  else
	  {
		  std::cerr<<"not found signal"<<std::endl;
	  }
	}
  }


  for (int i = 0; i < HDL_MAX_NUM_LASERS; i++)
    laser_rgb_mapping_[i].r = laser_rgb_mapping_[i].g = laser_rgb_mapping_[i].b = 0;

  if (laser_corrections_[32].distanceCorrection == 0.0)
  {
    for (int i = 0; i < 16; i++)
    {
      laser_rgb_mapping_[i * 2].b = static_cast<uint8_t> (i * 6 + 64);
      laser_rgb_mapping_[i * 2 + 1].b = static_cast<uint8_t> ( (i + 16) * 6 + 64);
    }
  }
  else
  {
    for (int i = 0; i < 16; i++)
    {
      laser_rgb_mapping_[i * 2].b = static_cast<uint8_t> (i * 3 + 64);
      laser_rgb_mapping_[i * 2 + 1].b = static_cast<uint8_t> ( (i + 16) * 3 + 64);
    }
    for (int i = 0; i < 16; i++)
    {
      laser_rgb_mapping_[i * 2 + 32].b = static_cast<uint8_t> (i * 3 + 160);
      laser_rgb_mapping_[i * 2 + 33].b = static_cast<uint8_t> ( (i + 16) * 3 + 160);
    }
  }
}

///////////////////////////////////////////////////////////////////////////////new
void
pcl::MyHDLGrabber::loadCorrectionsFile (const std::string& correctionsFile)//读取参数矫正文件的函数
{
  if (correctionsFile.empty ())
  {
	  if(LASER_LAYER == 16)
		loadVLP16Corrections();

    return;
  }
  else if(LASER_LAYER == 16)
  {
  loadVLP16Corrections(correctionsFile);
  return;
  }

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

void
pcl::MyHDLGrabber::loadVLP16Corrections (string pcap_file)
{
  std::string pkgPath = ros::package::getPath("sensor_driver");
  pcap_file = pkgPath + "/" +pcap_file;
  string anglefile = pcap_file + "/configuration_data/angle.csv";
  string channelnumfile = pcap_file + "/configuration_data/ChannelNum.csv";
  std::cout<<anglefile<<std::endl;
  std::cout<<channelnumfile<<std::endl;
  std::fstream angle(anglefile.c_str(),ios::in);
  std::fstream channelnum(channelnumfile.c_str(),ios::in);

  double vlp16VerticalCorrections[16];
  double distancecorrection[16];
  for(int i=0;i<16;i++)
	angle>>vlp16VerticalCorrections[i];

  for(int i=0;i<16;i++)
	channelnum>>distancecorrection[i];

  for (int i = 0; i < 16; i++)
  {
    laser_corrections_[i].azimuthCorrection = 0.0;
    laser_corrections_[i].distanceCorrection =distancecorrection[i];
    laser_corrections_[i].horizontalOffsetCorrection = 0.0;
    laser_corrections_[i].verticalOffsetCorrection = 0.0;
    laser_corrections_[i].verticalCorrection = vlp16VerticalCorrections[i];
    laser_corrections_[i].sinVertCorrection = std::sin (HDL_Grabber_toRadians(vlp16VerticalCorrections[i]));
    laser_corrections_[i].cosVertCorrection = std::cos (HDL_Grabber_toRadians(vlp16VerticalCorrections[i]));
  }
  for (int i = 16; i < HDL_MAX_NUM_LASERS; i++)
  {//根据协议可以知道每一个block发送两组数据
    laser_corrections_[i].azimuthCorrection = 0.0;
    laser_corrections_[i].distanceCorrection = 0.0;
    laser_corrections_[i].horizontalOffsetCorrection = 0.0;
    laser_corrections_[i].verticalOffsetCorrection = 0.0;
    laser_corrections_[i].verticalCorrection = 0.0;
    laser_corrections_[i].sinVertCorrection = 0.0;
    laser_corrections_[i].cosVertCorrection = 1.0;
  }
}

///////////////////////////////////////////////////////////////////////////////new
void
pcl::MyHDLGrabber::loadVLP16Corrections ()
{
  double vlp16VerticalCorrections01[] = {-15.0662,  -13.0887,  -11.0584,  -9.0554,  -7.0369,  -5.0362,  -3.0410,  -1.0384,  15.0428,  13.0276,  11.0101,  9.0100,  6.9839,  4.9935,  2.9767,  0.9847};//对于角度的确切值
  double distancecorrection01[]={444,  449,  442,  456,  444,  438,  433,  442,  447,  449,  446,  443,  446,  435,  450,  438};//对于位置的修正值
  double vlp16VerticalCorrections02[] = {-14.9994,  -13.0106,  -10.9894,  -9.0100,  -7.0086,  -4.9900,  -2.9946,  -1.0276,  15.0194,  12.9970,  11.0101,  9.0309,  7.0298,  5.0113,  2.9803,  0.9775};
  double distancecorrection02[]={445,  434,  446,  437,  441,  450,  445,  454,  457,  458,  448,  456,  450,  443,  436,  436};
  double * vlp16VerticalCorrections;
  double * distancecorrection;
  if(port_==9901)//left
  {
	  vlp16VerticalCorrections = vlp16VerticalCorrections01;
	  distancecorrection = distancecorrection01;
  }
  else
  {
	  vlp16VerticalCorrections = vlp16VerticalCorrections02;
	  distancecorrection = distancecorrection02;
  }
  for (int i = 0; i < 16; i++)
  {
    laser_corrections_[i].azimuthCorrection = 0.0;
    laser_corrections_[i].distanceCorrection =distancecorrection[i];
    laser_corrections_[i].horizontalOffsetCorrection = 0.0;
    laser_corrections_[i].verticalOffsetCorrection = 0.0;
    laser_corrections_[i].verticalCorrection = vlp16VerticalCorrections[i];
    laser_corrections_[i].sinVertCorrection = std::sin (HDL_Grabber_toRadians(vlp16VerticalCorrections[i]));
    laser_corrections_[i].cosVertCorrection = std::cos (HDL_Grabber_toRadians(vlp16VerticalCorrections[i]));
  }
  for (int i = 16; i < HDL_MAX_NUM_LASERS; i++)
  {//根据协议可以知道每一个block发送两组数据
    laser_corrections_[i].azimuthCorrection = 0.0;
    laser_corrections_[i].distanceCorrection = 0.0;
    laser_corrections_[i].horizontalOffsetCorrection = 0.0;
    laser_corrections_[i].verticalOffsetCorrection = 0.0;
    laser_corrections_[i].verticalCorrection = 0.0;
    laser_corrections_[i].sinVertCorrection = 0.0;
    laser_corrections_[i].cosVertCorrection = 1.0;
  }
}


/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
void
pcl::MyHDLGrabber::processRSPackets ()//处理激光雷达发过来的数据包
{

	topointtime.start();
  while (true)
  {
    unsigned char *data;

    topointtime.stop();

    //usleep(1);
	if(!read_packet_thread_paused_)
	{
		if (!hdl_data_.dequeue (data))
		{
			std::cout<<"dequeue error"<<std::endl;
			return;
		}

		//std::cout<<"toPointClouds"<<std::endl;
		toPointClouds (reinterpret_cast<HDLDataPacket *> (data));//数据从队列中转移到数据包中

		free (data);
	}
	else
	{
		usleep(1);
	}
  }
}

///////////////////////////////////////////////////////////////////////////////new
void
pcl::MyHDLGrabber::toPointClouds (HDLDataPacket *dataPacket)//将读取的数据转换成点云
{
  static uint32_t scanCounter = 0;
  static uint32_t sweepCounter = 0;
 //if (sizeof (HDLLaserReturn) != 3)
   //return;

  //time_t  time_;
  //time(&time_);
  //time_t velodyneTime = (time_ & 0x00000000ffffffffl) << 32 | dataPacket->gpsTimestamp;
//这个激光雷达的时间没有加上去，所以就先注释了
  scanCounter++;

  //std::cout<<"to pointclouds"<<std::endl;
  for (int i = 0; i < HDL_FIRING_PER_PKT; ++i)
  {
    HDLFiringData firingData = dataPacket->firingData[i];

	if(LASER_LAYER == 16)
	{
		//������16����azimuth
		unsigned short this_azimuth = firingData.rotation_1*256+firingData.rotation_2;//本时刻的水平角度
		//this_azimuth += 9000.0;
		unsigned short interpolated_rotationalPosition = firingData.rotation_1*256+firingData.rotation_2;
		if(interpolated_rotationalPosition < last_azimuth_)
						interpolated_rotationalPosition += (this_azimuth + 36000 - last_azimuth_) / 2;
					else
						interpolated_rotationalPosition += (this_azimuth - last_azimuth_) / 2;

					//对水平角度的设置

					if (masterflag_&&this_azimuth < last_azimuth_)
					{
						if(current_sweep_xyzi_->size() > 0 )
						{
							current_sweep_xyzi_->is_dense = false;
							current_sweep_xyzi_->header.seq = sweepCounter;

							sweepCounter++;
							if(mutilidar_signal_&&mutilidar_signal_->num_slots()>0)
								mutilidar_signal_->operator ()();
							fireCurrentSweep ();

						}
						current_sweep_xyzi_.reset (new pcl::PointCloud<pcl::PointXYZI> ());
					}
					if(trigger_)
					{
						trigger_ = false;
						if(current_sweep_xyzi_->size() > 0 )
						{
							current_sweep_xyzi_->is_dense = false;
							current_sweep_xyzi_->header.seq = sweepCounter;

							sweepCounter++;
							fireCurrentSweep ();

						}
						current_sweep_xyzi_.reset (new pcl::PointCloud<pcl::PointXYZI> ());

					}


		for (int j = 0; j < 16; j++)
		{
			PointXYZI xyzi;
			computeXYZI (xyzi, this_azimuth, firingData.laserReturns[j], laser_corrections_[j]);
			xyzi.passibility = 1.0;
			last_azimuth_ = this_azimuth;

			current_sweep_xyzi_->push_back (xyzi);
		}

			//�ڶ���16�㣬ֻ��interpolated_azimuth��Чʱ�ż���
			if(interpolated_rotationalPosition < 36000)
			{
				for (int j = 16; j<32; j++)
				{
					PointXYZI xyzi;
					computeXYZI (xyzi, interpolated_rotationalPosition, firingData.laserReturns[j], laser_corrections_[j - 16]);
					xyzi.passibility = 1.0;

					current_sweep_xyzi_->push_back (xyzi);
				}
			}	

	}
		
  }
}

///////////////////////////////////////////////////////////////////////////////new
void
pcl::MyHDLGrabber::computeXYZI (pcl::PointXYZI& point, int azimuth, 
                              HDLLaserReturn laserReturn, HDLLaserCorrection correction)
{//具体的对点的修正和赋值
  double cosAzimuth, sinAzimuth;
  double disresolution = 0.01;//距离的单位是cm
  double distanceM = (laserReturn.distance_1*256+laserReturn.distance_2) * disresolution;
  distanceM-= correction.distanceCorrection * disresolution;
  if (distanceM < min_distance_threshold_ || distanceM > max_distance_threshold_) {
    //point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN();
	point.x = point.y = point.z = 0.1;
  point.intensity = static_cast<float> (laserReturn.intensity);
	point.azimuth = static_cast<float> (azimuth) / 100.0;
    point.range = - 0.1;
    return;
  }

  if (correction.azimuthCorrection == 0)
  {
    cosAzimuth = cos_lookup_table_[azimuth];
    sinAzimuth = sin_lookup_table_[azimuth];
  }
  else
  {
    //double azimuthInRadians = HDL_Grabber_toRadians ((static_cast<double> (azimuth) / 100.0) - correction.azimuthCorrection);
//    cosAzimuth = std::cos (azimuthInRadians);
//    sinAzimuth = std::sin (azimuthInRadians);

	  double azimuthInRadians = HDL_Grabber_toRadians ((static_cast<double> (azimuth) / 100.0) - correction.azimuthCorrection);
	  cosAzimuth = std::cos (azimuthInRadians);
	  sinAzimuth = std::sin (azimuthInRadians);

  }

  double xyDistance = distanceM * correction.cosVertCorrection - correction.sinVertOffsetCorrection;
  	float intensityVal = laserReturn.intensity;

  	point.x = static_cast<float> (xyDistance * sinAzimuth - correction.horizontalOffsetCorrection * cosAzimuth);
  	point.y = static_cast<float> (xyDistance * cosAzimuth + correction.horizontalOffsetCorrection * sinAzimuth);
  	point.z = static_cast<float> (distanceM * correction.sinVertCorrection + correction.cosVertOffsetCorrection);
  	point.intensity = static_cast<float> (intensityVal);
  	point.azimuth = static_cast<float> (azimuth) / 100.0;
  	point.range = static_cast<float>(distanceM);
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::MyHDLGrabber::fireCurrentSweep ()
{
  pcl_conversions::toPCL(ros::Time::now(), current_sweep_xyzi_->header.stamp);//us
  if (sweep_xyzi_signal_->num_slots () > 0)
    sweep_xyzi_signal_->operator() (current_sweep_xyzi_); 
}

/////////////////////////////////////////////////////////////////////////////
bool
pcl::MyHDLGrabber::enqueueHDLPacket (const unsigned char *data,
    std::size_t bytesReceived)//jkj 2017/1/2
{
  if (bytesReceived == 1206)//根据协议，去除之前的42字节，对于剩下的字节数进行判断
  {
    unsigned char *dup = static_cast<unsigned char *> (malloc (bytesReceived * sizeof(unsigned char)));
    memcpy (dup, data, bytesReceived * sizeof(unsigned char));

    hdl_data_.enqueue (dup);
    return true;
  }
  return false;
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::MyHDLGrabber::start ()
{
  terminate_read_packet_thread_ = false;


  if (isRunning ())
    return;
//创建线程
  queue_consumer_thread_ = new boost::thread (boost::bind (&MyHDLGrabber::processRSPackets, this));

  if (pcap_file_name_.empty ())
  {
    try
    {
      try {
		  hdl_read_socket_ = new udp::socket (hdl_read_socket_service_, udp_listener_endpoint_);
	  }
	  catch (std::exception bind) {
		  delete hdl_read_socket_;
		  hdl_read_socket_ = new udp::socket (hdl_read_socket_service_, udp::endpoint(boost::asio::ip::address_v4::any(), udp_listener_endpoint_.port()));
	  }
      hdl_read_socket_service_.run ();
    }
    catch (std::exception &e)
    {
		PCL_ERROR ("[pcl::MyHDLGrabber::start] Unable to bind to socket! %s\n", e.what());
        return;
    }
    hdl_read_packet_thread_ = new boost::thread (boost::bind (&MyHDLGrabber::readPacketsFromSocket, this));
  }
  else
  {
#ifdef HAVE_PCAP
    hdl_read_packet_thread_ = new boost::thread(boost::bind(&MyHDLGrabber::readPacketsFromPcap, this));
#endif // #ifdef HAVE_PCAP
  }
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::MyHDLGrabber::stop ()
{
  terminate_read_packet_thread_ = true;
  hdl_data_.stopQueue ();

  if (hdl_read_packet_thread_ != NULL)
  {
    hdl_read_packet_thread_->interrupt ();
    hdl_read_packet_thread_->join ();
    delete hdl_read_packet_thread_;
    hdl_read_packet_thread_ = NULL;
  }
  if (queue_consumer_thread_ != NULL)
  {
    queue_consumer_thread_->join ();
    delete queue_consumer_thread_;
    queue_consumer_thread_ = NULL;
  }

  if (hdl_read_socket_ != NULL)
  {
    delete hdl_read_socket_;
    hdl_read_socket_ = NULL;
  }
}

void pcl::MyHDLGrabber::pause()
{
	read_packet_thread_paused_ = true;
}

void pcl::MyHDLGrabber::resume()
{
	read_packet_thread_paused_ = false;
}

bool pcl::MyHDLGrabber::getpausestate()
{
	return read_packet_thread_paused_;
}

/////////////////////////////////////////////////////////////////////////////
bool
pcl::MyHDLGrabber::isRunning () const
{
	return (!hdl_data_.isEmpty() || (hdl_read_packet_thread_ != NULL && 
         !hdl_read_packet_thread_->timed_join (boost::posix_time::milliseconds (10))));
}

/////////////////////////////////////////////////////////////////////////////
std::string
pcl::MyHDLGrabber::getName () const
{
  return (std::string ("RS High Definition Laser (HDL) Grabber"));
}

/////////////////////////////////////////////////////////////////////////////
float
pcl::MyHDLGrabber::getFramesPerSecond () const
{
  return (0.0f);
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::MyHDLGrabber::filterPackets (const boost::asio::ip::address& ipAddress,
                                const unsigned short port)
{
  source_address_filter_ = ipAddress;
  source_port_filter_ = port;
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::MyHDLGrabber::setLaserColorRGB (const pcl::RGB& color,
                                   unsigned int laserNumber)
{
  if (laserNumber >= HDL_MAX_NUM_LASERS)
    return;

  laser_rgb_mapping_[laserNumber] = color;
}

/////////////////////////////////////////////////////////////////////////////
bool
pcl::MyHDLGrabber::isAddressUnspecified (const boost::asio::ip::address& ipAddress)
{
#if BOOST_VERSION>=104700
  return (ipAddress.is_unspecified ());
#else
  if (ipAddress.is_v4 ())
    return (ipAddress.to_v4 ().to_ulong() == 0);

  return (false);
#endif
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::MyHDLGrabber::setMaximumDistanceThreshold(float &maxThreshold) {
  max_distance_threshold_ = maxThreshold;
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::MyHDLGrabber::setMinimumDistanceThreshold(float &minThreshold) {
  min_distance_threshold_ = minThreshold;
}

/////////////////////////////////////////////////////////////////////////////
float
pcl::MyHDLGrabber::getMaximumDistanceThreshold() {
  return(max_distance_threshold_);
}

/////////////////////////////////////////////////////////////////////////////
float
pcl::MyHDLGrabber::getMinimumDistanceThreshold() {
  return(min_distance_threshold_);
}

/////////////////////////////////////////////////////////////////////////////
void
pcl::MyHDLGrabber::readPacketsFromSocket ()
{
  unsigned char data[1500];//实际上每次收到的数据都是1248
  unsigned char msg_Lidar[1248];//发送给雷达的数据
  udp::endpoint sender_endpoint;

  while (!terminate_read_packet_thread_ && hdl_read_socket_->is_open())
  {
    size_t length = hdl_read_socket_->receive_from (boost::asio::buffer (data, 1500), sender_endpoint);
    for(int i = 0; i + 8 < length; i++)
   {
    	if ((data[i] == 0x55) && (data[i + 1] == 0xAA) && (data[i + 2] == 0x05) && (data[i + 3] == 0x0A) && (data[i + 4] == 0x5A) && (data[i + 5] == 0xA5) && (data[i + 6] == 0x50) && (data[i + 7] == 0xA0))//对于数据的一个校验，根据通信手册，数据包前8位是固定的
    	{
    		memcpy(msg_Lidar,data+i+42,1206);//其实i是等于0的
    		if (isAddressUnspecified (source_address_filter_) ||  (source_address_filter_ == sender_endpoint.address () && source_port_filter_ == sender_endpoint.port ()))
	  {
      enqueueHDLPacket (msg_Lidar, 1206);
	  }
    	}
    }
  }
}

/////////////////////////////////////////////////////////////////////////////
#ifdef HAVE_PCAP
void
pcl::MyHDLGrabber::readPacketsFromPcap ()
{
  struct pcap_pkthdr *header;
  const unsigned char *data;
  char errbuff[PCAP_ERRBUF_SIZE];

  //std::cout<<"readPacketsFromPcap\t"<<pcap_file_name_.c_str ()<<std::endl;
  pcap_t *pcap = pcap_open_offline (pcap_file_name_.c_str (), errbuff);

  struct bpf_program filter;
  std::ostringstream stringStream;

  stringStream << "udp ";
  if (!isAddressUnspecified(source_address_filter_))
  {
    stringStream << " and src port " << source_port_filter_ << " and src host " << source_address_filter_.to_string();
  }

  // PCAP_NETMASK_UNKNOWN should be 0xffffffff, but it's undefined in older PCAP versions
  if (pcap_compile (pcap, &filter, stringStream.str ().c_str(), 0, 0xffffffff) == -1)
  {
    PCL_WARN ("[pcl::MyHDLGrabber::readPacketsFromPcap] Issue compiling filter: %s.\n", pcap_geterr (pcap));
  }
  else if (pcap_setfilter(pcap, &filter) == -1)
  {
    PCL_WARN ("[pcl::MyHDLGrabber::readPacketsFromPcap] Issue setting filter: %s.\n", pcap_geterr (pcap));
  }

  MyTime totaltime;
  long long lastactualtime;

  struct timeval lasttime;
  long long uSecDelay;

  long long autualtime=0;

  lasttime.tv_sec = 0;
  timecounter.start();
  int packetcounter=0;
  int returnValue=-1;
  do{
  	returnValue = pcap_next_ex(pcap, &header, &data);
  }while(	!enqueueHDLPacket(data + 42, header->len - 42));
  totaltime.start();
  long long start_tv_sec =header->ts.tv_sec;
  long long start_tv_usec =header->ts.tv_usec;

  while (returnValue >= 0 && !terminate_read_packet_thread_)
  {
    if (lasttime.tv_sec == 0)
    {
      lasttime.tv_sec = header->ts.tv_sec;
      lasttime.tv_usec = header->ts.tv_usec;
    }
    if (lasttime.tv_usec > header->ts.tv_usec)
    {
      lasttime.tv_usec -= 1000000;
      lasttime.tv_sec++;
    }
    packetcounter++;

//    uSecDelay = ((header->ts.tv_sec - lasttime.tv_sec) * 1000000) +
//                (header->ts.tv_usec - lasttime.tv_usec);
    totaltime.stop();
//    timecounter.stop();

    autualtime=static_cast<long long>(totaltime.gettime_s()*1000000);
    uSecDelay=(header->ts.tv_sec-start_tv_sec)*1000000+header->ts.tv_usec-start_tv_usec-autualtime;

//    autualtime+=timecounter.gettime_s()*1000000;
   // timecounter.start();
//    file<<"packetcounter="<<packetcounter<<"\ttotaltime="<<totaltime.gettime_s()<<"\tuSecDelay="<<uSecDelay<<"\tautualtime="<<autualtime<<std::endl;
//    if(uSecDelay>autualtime)
//    {
//        uSecDelay=uSecDelay-autualtime;
//        autualtime=0;
//    }
//    else
//    {
//    	autualtime=autualtime-uSecDelay;
//    	uSecDelay=0;
//    }

    if(uSecDelay>0)
    boost::this_thread::sleep(boost::posix_time::microseconds(uSecDelay));


//    lasttime.tv_sec = header->ts.tv_sec;
//    lasttime.tv_usec = header->ts.tv_usec;

    // The ETHERNET header is 42 bytes long; unnecessary
do{
	returnValue = pcap_next_ex(pcap, &header, &data);
}while(	!enqueueHDLPacket(data + 42, header->len - 42));

  }
}
#endif //#ifdef HAVE_PCAP

int sendmsgtolidar1()//给雷达发送数据并将其激活,这个内容需要改ZhuBC，因为端口号需要改
{
    int socket_descriptor; //套接口描述字
    int portnum1 = 6699;
    char buf[49]={0xAA,0x00,0xFF,0x11,0x22,0x22,0xAA,0xAA,0x04,0xB0,0xC0,0xA8,0x01,0x71,0xC0,0xA8,0x01,0xCC,0x00,0x1C,0x23,0x17,0x4A,0xCC,0x1A,0x2B,0x1A,0x15,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x11,0x03,0x0A,0x09,0x2D,0x1E,0x00,0x64,0x00,0xC8,
0x00,0x0F,0xF0};
    boost::asio::io_service io_service;
    udp::endpoint local_endpoint(udp::v4(), 6699);//本地端口号
    boost::asio::ip::address addr = boost::asio::ip::address::from_string("192.168.1.204");
    udp::endpoint remote_endpoint(addr, 6677);//目标地址和目标端口号
    udp::socket socket(io_service, local_endpoint);

    socket.send_to(boost::asio::buffer(buf, strlen(buf)),remote_endpoint);



    return (0);

}

int sendmsgtolidar2()//给雷达发送数据并将其激活
{
    int socket_descriptor; //套接口描述字
    char buf[49]={0xAA,0x00,0xFF,0x11,0x22,0x22,0xAA,0xAA,0x04,0xB0,0xC0,0xA8,0x01,0x71,0xC0,0xA8,0x01,0xC8,0x00,0x1C,0x23,0x17,0x4A,0xCC,0x1A,0x20,0x19,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x11,0x03,0x0A,0x09,0x2D,0x1E,0x00,0x64,0x00,0xC8,
0x00,0x0F,0xF0};
    boost::asio::io_service io_service;
    udp::endpoint local_endpoint(udp::v4(), 6688);//本地端口号，由于LM42和LM48的端口号是一样的，所以端口号肯定是要改的，LM48的本地IP是192.168.1.102
    boost::asio::ip::address addr = boost::asio::ip::address::from_string("192.168.1.200");
    udp::endpoint remote_endpoint(addr, 6655);//目标地址和目标端口号
    udp::socket socket(io_service, local_endpoint);

    socket.send_to(boost::asio::buffer(buf, strlen(buf)),remote_endpoint);



    return (0);

}
