
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
using namespace std;
using namespace cv;
#define PORT 6666
#define IP "10.62.144.239"
//std::vector<cv::Mat> srcImage;
std::mutex mtx;
sockaddr_in server_vedio;
int socket_vedio;
VideoCapture capture(0);//打开摄像头
Mat image_result;
Mat image_vedio;
uchar encodeImg[65535];
double lasttime=-1;
char flag_channel;

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
void thread11(){
	while(1){
		double timenow=ros::Time::now().toSec();

		// cout<<"1111111111"<<endl;
		capture>>image_result;
		if(lasttime!=-1&&abs(timenow-lasttime)<0.1){//控制发送间隔，
//			cout<<"?????????????"<<endl;
//			lasttime=timenow;
			continue;
		}
		lasttime=timenow;
		imshow("show",image_result);
		waitKey(3);
		if(!image_result.empty()){
			std::vector<uchar> data_encode;
			std::vector<int> quality;
			quality.push_back(CV_IMWRITE_JPEG_QUALITY);
			quality.push_back(50);//进行50%的压缩
			imencode(".jpg", image_result, data_encode,quality);//将图像编码

			int nSize = data_encode.size();
			mtx.lock();
			for (int i = 0; i < nSize; i++)
			{
				encodeImg[i] = data_encode[i];
			}
			encodeImg[nSize]=0;
			sendto(socket_vedio, encodeImg, nSize+1, 0, (const sockaddr*)& server_vedio, sizeof(server_vedio));
			memset(&encodeImg, 0, sizeof(encodeImg));  //初始化结构体
			mtx.unlock();
			//		imshow("rsult",image_result);
			//		waitKey(5);
		}
//		usleep(100000);

	}
}
void thread33(){
	//recv
	while(1)
	{
		char buf[1024];
		int n = recvfrom(socket_vedio, buf, sizeof(buf), 0,(sockaddr *)& server_vedio, &len);//接受缓存
		if (n > 0) {
			flag_channel = buf[0];
			cout << "read command from server: " << flag_channel << endl;
		}
	}
}
void thread22(const sensor_msgs::ImageConstPtr& orgmsg){
	image_vedio=(cv_bridge::toCvShare(orgmsg)->image).clone();;

	if(!image_vedio.empty()){
		std::vector<uchar> data_encode;
		std::vector<int> quality;
		quality.push_back(CV_IMWRITE_JPEG_QUALITY);
		quality.push_back(50);//进行50%的压缩
		imencode(".jpg", image_vedio, data_encode,quality);//将图像编码
		int nSize=data_encode.size();
		mtx.lock();
		for (int i = 0; i < nSize; i++)
		{
			encodeImg[i] = data_encode[i];
		}
		encodeImg[nSize]=1;
		sendto(socket_vedio, encodeImg, nSize+1, 0, (const sockaddr*)& server_vedio, sizeof(server_vedio));
		memset(&encodeImg, 0, sizeof(encodeImg));  //初始化结构体
		mtx.unlock();
		//		imshow("rsult",image_result);
		//		waitKey(5);
	}
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "sub_one_image");
	ros::NodeHandle nh;


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
	server_vedio.sin_port = htons(PORT);         //设置端口号
	server_vedio.sin_addr.s_addr = inet_addr(IP);
	server_vedio.sin_port = htons(6666);//设置需要发送的IP和端口号
	bind(socket_vedio, (sockaddr*)&server_vedio, sizeof(server_vedio));//绑定端口号


	ros::Subscriber _sub;
	_sub=nh.subscribe("/image", 1,thread22);
	std::thread thread1{thread11};
	std::thread thread3{thread33};
	//	std::thread thread2{thread22};
	//	thread1.join();
	ros::spin();
	close(socket_vedio);

}
