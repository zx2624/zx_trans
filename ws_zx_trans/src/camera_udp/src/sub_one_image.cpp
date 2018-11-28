#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <unistd.h>//Linux系统下网络通讯的头文件集合
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#include <malloc.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <stdarg.h>
#include <fcntl.h>
#include <fcntl.h>
#include <mutex>
#include <thread>
#include "detection_result/detection_result_msg.h"
#include "object_detector_msgs/laser_electronic_result.h"
#include "sensor_driver_msgs/GpswithHeading.h"
#include "control_msgs/GetECUReport.h"
using namespace std;
using namespace cv;
int PORT = 6667;
int PORT2 = 9999;
string IP = "192.168.8.119";
//std::vector<cv::Mat> srcImage;
std::mutex mtx_0;
std::mutex mtx_1;
std::mutex mtx_2;
std::mutex mtx_gps;
std::mutex mtx_status;
float longitude = 0, altitude = 0, latitude = 0;
float speed = 0;
unsigned char gear;
sockaddr_in server_vedio;
int socket_vedio;

sockaddr_in socket_add_result;
int socket_result;

VideoCapture capture(0);//打开摄像头

Mat image_0,image_1,image_2;
uchar encodeImg[6553555];
double lasttime=-1;
double lasttime_1 = -1;
unsigned char flag_channel='2';

socklen_t len;
//class ImageDeal
//{
//	ros::NodeHandle& _nh;
//	ros::Subscriber _sub;
//
//public:
//	cv::Mat _srcImage;
//	ImageDeal(ros::NodeHandle& nh):_nh(nh)
//	{
//		_sub=_nh.subscribe("/image", 1, &ImageDeal::imageCb, this);
//
//	}
//	~ImageDeal()
//	{
//		cv::destroyAllWindows();
//	}
//
//	void imageCb(const sensor_msgs::ImageConstPtr& orgmsg)
//	{
//		mtx.lock();
//		_srcImage = (cv_bridge::toCvShare(orgmsg)->image).clone();
//		mtx.unlock();
//		//		if(_srcImage.empty())
//		//			std::cout << "Can't get image!" << std::endl;
//		//		else
//		//			std::cout << "Get one image!" << std::endl;
//	}
//};
void probecb(const object_detector_msgs::laser_electronic_resultConstPtr& msg){

	mtx_0.lock();
	image_0 = (cv_bridge::toCvCopy(msg->image_data)->image).clone();
	mtx_0.unlock();
}
void thread33(){
	//recv
	while(1)
	{
		unsigned char buf[1024];
		int n = recvfrom(socket_vedio, buf, sizeof(buf), 0,(sockaddr *)& server_vedio, &len);//接受缓存
		if (n > 0) {
			if(buf[0]=='0'||buf[0]=='1'||buf[0]=='2')
			{
				flag_channel = buf[0];
				//			if(flag_channel == '1')
				//				cout<<"flag is 1 @@@@@@@@@@@@@@@@@@@@@@@"<<endl;
				cout << "read command from server: " << flag_channel << endl;
			}
		}
	}
}
void computercam(){
	while(ros::ok()){
		double timenow = ros::Time::now().toSec();
		if(lasttime != -1 && timenow - lasttime < 0.1){
//			lasttime = timenow;
			capture>>image_1;
			continue;
		}
		lasttime = timenow;
		mtx_1.lock();
		if(capture.isOpened())
			capture>>image_1;
//		cv::resize(image_1,image_1,cv::Size(960,640));
		mtx_1.unlock();
	}
}
void thread22(const sensor_msgs::ImageConstPtr& orgmsg){
	mtx_1.lock();
	image_1=(cv_bridge::toCvShare(orgmsg)->image).clone();
	//	if(capture.isOpened())
	//		capture>>image_1;
	cv::resize(image_1,image_1,cv::Size(960,640));
	mtx_1.unlock();
}
void leftimageCb(const sensor_msgs::ImageConstPtr& orgmsg){
	mtx_0.lock();
	image_0 = (cv_bridge::toCvCopy(orgmsg)->image).clone();
	mtx_0.unlock();
}
void imageCb(const sensor_msgs::ImageConstPtr& orgmsg)
{
	mtx_2.lock();
	//	cout<<"屏幕图像获取成功。。。。。"<<endl;
	image_2 = (cv_bridge::toCvShare(orgmsg)->image).clone();
	cv::resize(image_2,image_2,cv::Size(image_2.cols/2,image_2.rows/2));
	//	imshow("screen",image_2);
	//	waitKey(3);
	mtx_2.unlock();

}
void float_to_uchar(float b, unsigned char* sendbuf){
	int a = b * 100000;
	sendbuf[0] = a & 0xff;
	sendbuf[1] = (a & 0xff00) >> 8;
	sendbuf[2] = (a & 0xff0000) >> 16;
	sendbuf[3] = (a & 0xff000000) >> 24;
}
void process(){
	while(ros::ok()){
		double timenow=ros::Time::now().toSec();
		lasttime = timenow;
		Mat imgsend;
		if(flag_channel == '0'){
			//			cout<<"flag is 0"<<endl;
			mtx_0.lock();
			imgsend = image_0.clone();
			mtx_0.unlock();
		}else if(flag_channel == '1'){
			//			cout<<"flag is 1"<<endl;
			mtx_1.lock();
			imgsend = image_1.clone();
			mtx_1.unlock();
		}else if(flag_channel == '2'){
			//						cout<<"flag is 2"<<endl;
			mtx_2.lock();
			imgsend = image_2.clone();
			mtx_2.unlock();
		}else{
			cout<<"wtf"<<endl;

		}
		if(!imgsend.empty()){//!imgsend.empty()
			std::vector<uchar> data_encode;
			std::vector<int> quality;
			quality.push_back(CV_IMWRITE_JPEG_QUALITY);
			quality.push_back(10);//进行50%的压缩
			imencode(".jpg", imgsend, data_encode,quality);//将图像编码
			int nSize=data_encode.size();

			for (int i = 0; i < nSize; i++)
			{
				encodeImg[i] = data_encode[i];
				//				cout<<i<<"   "<<nSize<<endl;
			}
			sendto(socket_vedio, encodeImg, nSize, 0, (const sockaddr*)& server_vedio, sizeof(server_vedio));
			memset(&encodeImg, 0, sizeof(encodeImg));  //初始化结构体
			//			imshow("rsult",imgsend);
			//			waitKey(5);
		}else{
			if(!image_2.empty()){
				std::vector<uchar> data_encode;
				std::vector<int> quality;
				quality.push_back(CV_IMWRITE_JPEG_QUALITY);
				quality.push_back(90);//进行50%的压缩
				mtx_2.lock();
				imencode(".jpg", image_2, data_encode,quality);//将图像编码
				mtx_2.unlock();
				int nSize=data_encode.size();

				//			cout<<"size is "<<nSize<<endl;
				for (int i = 0; i < nSize; i++)
				{
					encodeImg[i] = data_encode[i];
					//				cout<<i<<"   "<<nSize<<endl;
				}
				sendto(socket_vedio, encodeImg, nSize, 0, (const sockaddr*)& server_vedio, sizeof(server_vedio));
				memset(&encodeImg, 0, sizeof(encodeImg));  //初始化结构体
			}
		}
		if(latitude > 10){
			unsigned char send_buf_gps[12];
			mtx_gps.lock();
			float_to_uchar(longitude, send_buf_gps);
			float_to_uchar(latitude, send_buf_gps + 4);
			float_to_uchar(altitude, send_buf_gps + 8);
			mtx_gps.unlock();
			sendto(socket_vedio, send_buf_gps, 12, 0, (const sockaddr*)& server_vedio, sizeof(server_vedio));
		}
		//发送车辆信息
		unsigned char send_buf_status[6];
		send_buf_status[0] = 231;//第一个字节==231代表传输的是状态信息
		mtx_status.lock();
		float_to_uchar(speed, send_buf_status + 1);
		send_buf_status[5] = (unsigned char) gear;//第六个字节代表档位
		mtx_status.unlock();
		sendto(socket_vedio, send_buf_status, 6, 0, (const sockaddr*)& server_vedio, sizeof(server_vedio));

		usleep(100000);
	}


}
void sendResult(){
	while(1){
		if(!image_2.empty()){//!imgsend.empty()
			std::vector<uchar> data_encode;
			std::vector<int> quality;
			quality.push_back(CV_IMWRITE_JPEG_QUALITY);
			quality.push_back(10);//进行50%的压缩
			mtx_2.lock();
			imencode(".jpg", image_2, data_encode,quality);//将图像编码
			mtx_2.unlock();
			int nSize=data_encode.size();

			//			cout<<"size is "<<nSize<<endl;
			for (int i = 0; i < nSize; i++)
			{
				encodeImg[i] = data_encode[i];
				//				cout<<i<<"   "<<nSize<<endl;
			}
			sendto(socket_result, encodeImg, nSize, 0, (const sockaddr*)& socket_add_result, sizeof(socket_add_result));
			memset(&encodeImg, 0, sizeof(encodeImg));  //初始化结构体
			//			imshow("rsult",imgsend);
			//			waitKey(5);
		}
		sleep(1);
	}


}
void gpscallback(const sensor_driver_msgs::GpswithHeadingConstPtr& gpsmsg){
	mtx_gps.lock();
	latitude = gpsmsg->gps.latitude;
	longitude = gpsmsg->gps.longitude;
	altitude = gpsmsg->gps.altitude;
	mtx_gps.unlock();

}
void statuscallback(const control_msgs::GetECUReportConstPtr& status_msg){
	std::cout << "got ecudata -- " << std::endl;
	mtx_status.lock();
	speed = (float) status_msg->speed.velocity.linear.x;
	gear = (char)status_msg->shift_cur.gear;
	mtx_status.unlock();

}
int main(int argc, char ** argv)
{
	ros::init(argc, argv, "sub_one_image");
	ros::NodeHandle nh;
	ros::param::get("ip",IP);
	ros::param::get("~port",PORT);
	ros::param::get("~port2",PORT2);
	cout<<"========视频传输客户端IP=========="<<IP<<endl;
	cout<<"========视频传输客户端PORT=========="<<PORT<<endl;
	cout<<"========PORT2=========="<<PORT2<<endl;
	//	ImageDeal imgd(nh);
	int count = 0;
	cv::Mat srcImage;
	if ((socket_vedio = socket(AF_INET, SOCK_DGRAM, 0)) < 0)    //创建socket句柄，采用UDP协议
	{
		printf("create socket error: %s(errno: %d)\n", strerror(errno), errno);
		return -1;
	}
	memset(&server_vedio, 0, sizeof(server_vedio));  //初始化结构体
	server_vedio.sin_family = AF_INET;           //设置通信方式
	//	server_vedio.sin_port = htons(PORT);         //设置端口号
	server_vedio.sin_addr.s_addr = inet_addr(IP.c_str());
	server_vedio.sin_port = htons(PORT);//设置需要发送的IP和端口号
	bind(socket_vedio, (sockaddr*)&server_vedio, sizeof(server_vedio));//绑定端口号


	if ((socket_result = socket(AF_INET, SOCK_DGRAM, 0)) < 0)    //创建socket句柄，采用UDP协议
	{
		printf("create socket error: %s(errno: %d)\n", strerror(errno), errno);
		return -1;
	}
	memset(&socket_add_result, 0, sizeof(socket_add_result));  //初始化结构体
	socket_add_result.sin_family = AF_INET;           //设置通信方式
	//	socket_add_result.sin_port = htons(PORT);         //设置端口号
	socket_add_result.sin_addr.s_addr = inet_addr(IP.c_str());
	socket_add_result.sin_port = htons(PORT2);//设置需要发送的IP和端口号
	bind(socket_result, (sockaddr*)&socket_add_result, sizeof(socket_add_result));//绑定端口号


	//todo;获取侦察视频
	ros::Subscriber sub_front=nh.subscribe("/image", 1,thread22);//前视摄像头数据
	ros::Subscriber sub_screeb = nh.subscribe("screen_image_topic", 1, imageCb);//屏幕截图
//	ros::Subscriber sub_probe = nh.subscribe("realtime_video_topic", 1, probecb);//侦察视频
	ros::Subscriber sub_left = nh.subscribe("/left_image", 1, leftimageCb);
	ros::Subscriber sub_gps = nh.subscribe("/sensor_fusion_output", 1, gpscallback);
	ros::Subscriber sub_status = nh.subscribe("ecudatareport", 1, statuscallback);
//	std::thread thread1{computercam};
	std::thread thread3{thread33};//接收命令线程
	std::thread thread4{process};//整体处理发送线程
	//	std::thread thread5{sendResult};
	//	thread1.join();
	ros::spin();
	close(socket_vedio);

}
