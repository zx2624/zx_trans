#include <ros/ros.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <WinSock2.h>
//#pragma comment(lib,"WS2_32.lib")
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
#include <thread>
#include <mutex>
#include "sensor_driver_msgs/GpswithHeading.h"
#include "control_msgs/GetECUReport.h"
using namespace cv;
using namespace std;

int PORT = 6667;
int socket_result;
sockaddr_in addr_result;
Mat image_result;
int socket_vedio;
sockaddr_in m_servaddr;
Mat image_vedio;
Mat imshoww;
sockaddr_in client;
socklen_t len;

sockaddr_in client_result;
socklen_t len_result;
char* name2="result";
char* name1="result22";

std::mutex mtx_0;
std::mutex mtx_1;
std::mutex mtx_gps;
float altitude = 0, latitude = 0, longitude = 0;
float speed = 0;
int gear = -1;

void command(){
	while(1){
		// cout<<"thread11"<<endl;
		cout << "Input the video channel: ";
		unsigned char a;
		cin>>a;
		sendto(socket_vedio, &a, 2, 0, (const sockaddr*)& client, len);
		usleep(100000);
	}
}
void uchar_to_float(unsigned char* buf, float& longi, float& lati,float& alti){
	longi = int ( buf[0] | buf[1] << 8 | buf[2] <<16 | buf[3] << 24) * 0.00001;
	lati = int ( buf[4] | buf[5] << 8 | buf[6] <<16 | buf[7] << 24) * 0.00001;
	alti = int ( buf[8] | buf[9] << 8 | buf[10] <<16 | buf[11] << 24) * 0.00001;
}
void receive(){
	while(1){
		unsigned char buf[65536];
		std::vector<uchar> decode;
		int n = recvfrom(socket_vedio, buf, sizeof(buf), 0,(sockaddr *)& client,&len);//接受缓存
		int pos = 0;
		if(n > 12){
			while (pos < n)
			{
				decode.push_back(buf[pos++]);//存入vector
			}
			mtx_0.lock();
			image_vedio = imdecode(decode, CV_LOAD_IMAGE_COLOR);
			mtx_0.unlock();
		}
		if(n == 12){
			uchar_to_float(buf, longitude,latitude,altitude);
		}
		if(buf[0] == 231){//第一个字节231代表车辆信息
			speed = int ( buf[1] | buf[2] << 8 | buf[3] <<16 | buf[4] << 24) * 0.00001;
			gear = int(buf[5]);
//			std::cout << "got status " << speed << "  " << shift_cur << std::endl;
		}

	}
	close(socket_vedio);
}

void resultReceive(){
	while(1){
		Mat img;
		char buf[65536];
		std::vector<uchar> decode;
		int n = recvfrom(socket_result, buf, sizeof(buf), 0,(sockaddr *)& client_result,&len_result);//接受缓存
		int pos = 0;
		while (pos < n)
		{
			decode.push_back(buf[pos++]);//存入vector
		}
		mtx_1.lock();
		image_result = imdecode(decode, CV_LOAD_IMAGE_COLOR);
		mtx_1.unlock();
		if(!img.empty()){
//			cout<<"got image result ..  "<<endl;

		}else{
//			cout<<"got no image .."<<endl;
		}
	}
	close(socket_result);
}
int main(int argc, char** argv)
{
	//    WSADATA wsaData;
	//    WSAStartup(0x01, &wsaData); //创建初始化句柄
	//
	//    SOCKET socket_vedio;
//	ros::param::get("~ip",IP);
	ros::init(argc, argv, "cam_recv");
	ros::NodeHandle nh;
	ros::Publisher gps_pub = nh.advertise<sensor_driver_msgs::GpswithHeading>("/sensor_fusion_output", 20);
	ros::Publisher status_pub = nh.advertise<control_msgs::GetECUReport>("ecudatareport", 10);
	ros::param::get("~port",PORT);
	cout<<"视频接收监听端口-- "<<PORT<<endl;
	cout<<"all begins now"<<endl;
	namedWindow("vedios",CV_WINDOW_NORMAL);

//	namedWindow("result22",CV_WINDOW_NORMAL);

	if ((socket_vedio = socket(AF_INET, SOCK_DGRAM, 0)) < 0)    //创建socket句柄，采用UDP协议
	{
		printf("create socket error: %s(errno: %d)\n", strerror(errno), errno);
		return -1;
	}

	memset(&m_servaddr, 0, sizeof(m_servaddr));  //初始化结构体
	m_servaddr.sin_family = AF_INET;           //设置通信方式
	m_servaddr.sin_port = htons(PORT);         //设置端口号
	bind(socket_vedio, (sockaddr*)&m_servaddr, sizeof(m_servaddr));//绑定套接字


	if ((socket_result = socket(AF_INET, SOCK_DGRAM, 0)) < 0)    //创建socket句柄，采用UDP协议
	{
		printf("create socket error: %s(errno: %d)\n", strerror(errno), errno);
		return -1;
	}

	memset(&addr_result, 0, sizeof(addr_result));  //初始化结构体
	addr_result.sin_family = AF_INET;           //设置通信方式
	addr_result.sin_port = htons(9999);         //设置端口号
	bind(socket_result, (sockaddr*)&addr_result, sizeof(addr_result));//绑定套接字

	std::thread thread_command{command};
	std::thread thread_receive{receive};
//	std::thread thread3{resultReceive};
	//	imshoww.create(960+480+3,1280+640+3,CV_8UC3);
	while(1){
		if(!image_vedio.empty()){
			mtx_0.lock();
			imshow("vedios",image_vedio);
			mtx_0.unlock();
		}
		sensor_driver_msgs::GpswithHeading gps_msg;
		mtx_gps.lock();
		gps_msg.gps.altitude = altitude;
		gps_msg.gps.longitude = longitude;
		gps_msg.gps.latitude = latitude;
		mtx_gps.unlock();
		gps_pub.publish(gps_msg);

		//发送车辆状态
		control_msgs::GetECUReport status_msg;
		status_msg.shift_cur.gear = gear;
		status_msg.speed.velocity.linear.x = speed;
		status_pub.publish(status_msg);
//		if(!image_result.empty()){
//			mtx_1.lock();
//			imshow("检测结果",image_result);
//			mtx_1.unlock();
//		}
		waitKey(3);
		usleep(100000);
	}
	thread_command.join();
	thread_receive.join();
//	thread3.join();

	//namedWindow("vedio-result",CV_WINDOW_NORMAL);



	return 0;
}
