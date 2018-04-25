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
//xiao add 0106
#define PCL_NO_PRECOMPILE

#include "pcl/pcl_config.h"

#ifndef PCL_IO_HDL_GRABBER_H_
#define PCL_IO_HDL_GRABBER_H_

#include <pcl/io/grabber.h>
#include <pcl/io/impl/synchronized_queue.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/asio.hpp>
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h> //fps calculations
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/boost.h>
#include <pcl/visualization/mouse_event.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/impl/angles.hpp>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/visualization/impl/pcl_visualizer.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <glog/logging.h>

#include "mytime.h"
struct hdlPoint
{
    PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
    float intensity;
    float laserID;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
}EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (hdlPoint,           // here we assume a XYZ + "test" (as fields)
        (float, x, x)
        (float, y, y)
        (float, z, z)
        (float, intensity, intensity)
        (float, laserID, laserID)
        )


#define HAVE_PCAP
#define HDL_Grabber_toRadians(x) ((x) * M_PI / 180.0)

#ifndef _WINDOWS
    typedef unsigned char           UCHAR;
    typedef unsigned char          UINT8;
    typedef unsigned short int UINT16;
    typedef unsigned int UINT;
    typedef long long int LONGLONG;
    typedef unsigned char uchar;

#define strncpy_s(dest,len,src,count) strncpy(dest,src,count)
#define sprintf_s(dest,len,format,args...) sprintf(dest,format,args)
#define sscanf_s sscanf
#define strcpy_s(dst,len,src) strcpy(dst,src)
#define _strdup(src) strdup(src)
#define strtok_s(tok,del,ctx) strtok(tok,del)
#endif



    namespace pcl
{
    struct CalibrationValue{
        double alfa;
        double beta;
        double gama;
        double x_offset;
        double y_offset;
        double z_offset;

    };

    void get_transform_matrix(const CalibrationValue& calibvalue,Eigen::Matrix4f& transform_matrix_calibration);


    /** \brief Grabber for the Velodyne High-Definition-Laser (HDL)
     * \author Keven Ring <keven@mitre.org>
     * \ingroup io
     */

    struct LaserData{
        int number;
        double angle;
    };


    class /*PCL_EXPORTS*/ VelodyneGrabber : public Grabber
    {
        typedef unsigned char uchar;
        public:
        /** \brief Signal used for a single sector
         *         Represents 1 corrected packet from the HDL Velodyne
         */
        //typedef void (sig_cb_velodyne_hdl_scan_point_cloud_xyz) (
        //    const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> >&,
        //    float, float);
        /** \brief Signal used for a single sector
         *         Represents 1 corrected packet from the HDL Velodyne.  Each laser has a different RGB
         */
        //typedef void (sig_cb_velodyne_hdl_scan_point_cloud_xyzrgb) (
        //    const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA> >&,
        //    float, float);
        /** \brief Signal used for a single sector
         *         Represents 1 corrected packet from the HDL Velodyne with the returned intensity.
         */
        //typedef void (sig_cb_velodyne_hdl_scan_point_cloud_xyzi) (
        //    const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI> >&,
        //    float startAngle, float);
        /** \brief Signal used for a 360 degree sweep
         *         Represents multiple corrected packets from the HDL Velodyne
         *         This signal is sent when the Velodyne passes angle "0"
         */
        //typedef void (sig_cb_velodyne_hdl_sweep_point_cloud_xyz) (
        //    const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> >&);
        /** \brief Signal used for a 360 degree sweep
         *         Represents multiple corrected packets from the HDL Velodyne with the returned intensity
         *         This signal is sent when the Velodyne passes angle "0"
         */
        typedef void (sig_cb_velodyne_hdl_sweep_point_cloud_xyzi) (
                const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI> >& , int id);

        //typedef void (sig_cb_velodyne_hdl_sweep_point_cloud_xyzil) (
        //	const boost::shared_ptr<const pcl::PointCloud<hdlPoint> >&);
        /** \brief Signal used for a 360 degree sweep
         *         Represents multiple corrected packets from the HDL Velodyne
         *         This signal is sent when the Velodyne passes angle "0".  Each laser has a different RGB
         */
        //typedef void (sig_cb_velodyne_hdl_sweep_point_cloud_xyzrgb) (
        //    const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA> >&);

        /** \brief Constructor taking an optional path to an HDL corrections file.  The Grabber will listen on the default IP/port for data packets [192.168.3.255/2368]
         * \param[in] correctionsFile Path to a file which contains the correction parameters for the HDL.  This parameter is mandatory for the HDL-64, optional for the HDL-32
         * \param[in] pcapFile Path to a file which contains previously captured data packets.  This parameter is optional
         *///new
        VelodyneGrabber (const std::string& correctionsFile = "",
                const std::string& pcapFile = "", int laser_layer = 16
				,int id = 0,boost::shared_ptr<std::vector<int> > synchornizationcontainor = boost::make_shared<std::vector<int> >(std::vector<int>()));

        /** \brief Constructor taking a pecified IP/port and an optional path to an HDL corrections file.
         * \param[in] ipAddress IP Address that should be used to listen for HDL packets
         * \param[in] port UDP Port that should be used to listen for HDL packets
         * \param[in] correctionsFile Path to a file which contains the correction parameters for the HDL.  This field is mandatory for the HDL-64, optional for the HDL-32
         *///new
        VelodyneGrabber (const boost::asio::ip::address& ipAddress,
                const unsigned short port, const std::string& correctionsFile = "", int laser_layer = 16
				,int id = 0,boost::shared_ptr<std::vector<int> > synchornizationcontainor = boost::make_shared<std::vector<int> >(std::vector<int>()));

        /** \brief virtual Destructor inherited from the Grabber interface. It never throws. */
        virtual ~VelodyneGrabber () throw ();

        /** \brief Starts processing the Velodyne packets, either from the network or PCAP file. */
        virtual void start ();

        /** \brief Stops processing the Velodyne packets, either from the network or PCAP file */
        virtual void stop ();

        void pause();

        void resume();

        bool getpausestate();

        /** \brief Obtains the name of this I/O Grabber
         *  \return The name of the grabber
         */
        virtual std::string getName () const;

        /** \brief Check if the grabber is still running.
         *  \return TRUE if the grabber is running, FALSE otherwise
         */
        virtual bool isRunning () const;

        /** \brief Returns the number of frames per second.
        */
        virtual float getFramesPerSecond () const;

        /** \brief Allows one to filter packets based on the SOURCE IP address and PORT
         *         This can be used, for instance, if multiple HDL LIDARs are on the same network
         */
        void filterPackets (const boost::asio::ip::address& ipAddress,
                const unsigned short port = 443);

        /** \brief Allows one to customize the colors used for each of the lasers.
        */
        void setLaserColorRGB (const pcl::RGB& color, unsigned int laserNumber);

        /** \brief Any returns from the HDL with a distance less than this are discarded.
         *         This value is in meters
         *         Default: 0.0
         */
        void setMinimumDistanceThreshold(float & minThreshold);

        /** \brief Any returns from the HDL with a distance greater than this are discarded.
         *         This value is in meters
         *         Default: 10000.0
         */
        void setMaximumDistanceThreshold(float & maxThreshold);

        /** \brief Returns the current minimum distance threshold, in meters
        */

        float getMinimumDistanceThreshold();

        /** \brief Returns the current maximum distance threshold, in meters
        */
        float getMaximumDistanceThreshold();

        long getframecount();

        void setCalibration(const CalibrationValue& value);
        void setDataFromExtern()
        {
        	externdatamode_ = true;
        }

        inline bool externenqueueHDLPacket (const unsigned char *data,
                std::size_t bytesReceived)
        {
        	if(externdatamode_)
        		return enqueueHDLPacket(data,bytesReceived);
        	else
        		return false;
        }

        protected:
        static const int HDL_DATA_PORT = 2368;
        static const int HDL_NUM_ROT_ANGLES = 36001;
        static const int HDL_LASER_PER_FIRING = 32;
        static const int HDL_MAX_NUM_LASERS = 64;
        static const int HDL_FIRING_PER_PKT = 12;
        static const boost::asio::ip::address HDL_DEFAULT_NETWORK_ADDRESS;

        enum HDLBlock
        {
            BLOCK_0_TO_31 = 0xeeff, BLOCK_32_TO_63 = 0xddff
        };

#pragma pack(push, 1)
        typedef struct HDLLaserReturn
        {
            unsigned short distance;
            unsigned char intensity;
        } HDLLaserReturn;
#pragma pack(pop)

        struct HDLFiringData
        {
            unsigned short blockIdentifier;
            unsigned short rotationalPosition;
            HDLLaserReturn laserReturns[HDL_LASER_PER_FIRING];
        };

        struct HDLDataPacket
        {
            HDLFiringData firingData[HDL_FIRING_PER_PKT];
            unsigned int gpsTimestamp;
            unsigned char blank1;
            unsigned char blank2;
        };

        struct HDLLaserCorrection
        {
            double azimuthCorrection;
            double verticalCorrection;
            double distanceCorrection;
            double verticalOffsetCorrection;
            double horizontalOffsetCorrection;
            double sinVertCorrection;
            double cosVertCorrection;
            double sinVertOffsetCorrection;
            double cosVertOffsetCorrection;

            ////xiao add 0105
            uchar minIntensity;
            uchar maxIntensity;
            double focalSlope;
            int focalDistance;
        };

        private:
        static double *cos_lookup_table_;
        static double *sin_lookup_table_;
        pcl::SynchronizedQueue<unsigned char *> hdl_data_;
        boost::asio::ip::udp::endpoint udp_listener_endpoint_;
        boost::asio::ip::address source_address_filter_;
        unsigned short source_port_filter_;
        boost::asio::io_service hdl_read_socket_service_;
        boost::asio::ip::udp::socket *hdl_read_socket_;
        std::string pcap_file_name_;
        boost::thread *queue_consumer_thread_;
        boost::thread *hdl_read_packet_thread_;
        HDLLaserCorrection laser_corrections_[HDL_MAX_NUM_LASERS];
        bool terminate_read_packet_thread_;
        bool read_packet_thread_paused_;
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI> >
            current_sweep_xyzi_;
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI> >
            current_calibrated_xyzi_;
        unsigned int last_azimuth_;
        boost::signals2::signal<sig_cb_velodyne_hdl_sweep_point_cloud_xyzi>* sweep_xyzi_signal_;
        int framecount;
        ros::NodeHandle nh;
        ros::Publisher pubImudata;
        ros::Publisher pubPointCloud;
        CalibrationValue calibrationvalue_;
        Eigen::Matrix4f calibration_transform_matrix_;
        bool calibrated_;
        pcl::RGB laser_rgb_mapping_[HDL_MAX_NUM_LASERS];
        float min_distance_threshold_;
        float max_distance_threshold_;
        int LASER_LAYER;
        boost::shared_ptr<std::vector<int> > synchornizationcontainor_;
        int id_;
        bool externdatamode_;
        void processVelodynePackets ();
        bool enqueueHDLPacket (const unsigned char *data,
                std::size_t bytesReceived);
        void initialize (const std::string& correctionsFile);
        void loadCorrectionsFile (const std::string& correctionsFile);

        void loadVLP16Corrections ();
        void loadHDL32Corrections ();
        void readPacketsFromSocket ();
#ifdef HAVE_PCAP
        MyTime timecounter;
        void readPacketsFromPcap();
        void readPacketsFromPcapold();
        void readPacketsFromPcapwithins ();
#endif //#ifdef HAVE_PCAP
        struct struct_FONSData
       	  {
       		  double dHeading;
       		  double dPitch;
       		  double dRoll;

       		  double dAccx;
       		  double dAccy;
       		  double dAccz;

       		  double dLat;
       		  double dLng;
       		  double dAltitude;

       		  double dState;
       		  char GPSState;

       		  UCHAR				ucStarNum;
       		  UCHAR				ucFONSState;

       		  UCHAR				ucStaThreshold;		//ECU通信状态阈值
       		  UCHAR				ucStaTimes;			//ECU通信状态计数
       		  UCHAR				ucStaOK;			//ECU通信状态

       		  double				dTimeStamp;			//IMU时间戳

       		  double				m_AHRSHeading;			//IMU发来的Heading
       		  double				dIPitch;			//IMU发来的Pitch
       		  double				dIRoll;				//IMU发来的Roll


       		  double				dGyroX;				//IMU发来的绕X向Angular rate
       		  double				dGyroY;				//IMU发来的绕Y向Angular rate
       		  double				dGyroZ;				//IMU发来的绕Z向Angular rate


       		  double				dVdd;				//IMU发来的电压
       		  double				dTemp;				//IMU发来的平均温度



       		  UCHAR				ucOStaThreshold;		//IMU通信状态阈值
       		  UCHAR				ucOStaTimes;			//IMU通信状态计数
       		  UCHAR				ucOStaOK;			//IMU通信状态


       		  //*****新加  fons ASCII 码协议部分
       		  double              dRunTime;           //FONS 上电运行时间
       		  double              dUTC;                //FONS utc世界协调时间
       		  double              dEastV;             // FONS  东向速度
       		  double              dNorthV;            // FONS  北向速度
       		  double              dAltitudeV;         // 天向速度

       	  };
       	struct_FONSData m_sFONSData;
       	unsigned int last_time_;
        uint32_t scanCounter_;
        uint32_t sweepCounter_;
        void OctansData(const unsigned char * OctansData,std::size_t n);
        void toPointClouds (HDLDataPacket *dataPacket);
        void fireCurrentSweep ();
        void computeXYZI (pcl::PointXYZI& pointXYZI, int azimuth,
                HDLLaserReturn laserReturn, HDLLaserCorrection correction);
        void calibratePoint (pcl::PointXYZI& point);
        bool isAddressUnspecified (const boost::asio::ip::address& ip_address);

        public:
        std::map<double,int> map_tanangle_index;//sort by angle
        LaserData indexmaptable[HDL_MAX_NUM_LASERS];
    };
}

#endif /* PCL_IO_HDL_GRABBER_H_ */
