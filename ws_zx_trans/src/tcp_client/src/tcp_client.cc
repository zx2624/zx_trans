#include <ros/ros.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <stdio.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/shm.h>
#include <iostream>
#include <deque>
#include <mutex>
#include <thread>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "detection_result/detection_result_msg.h"

using namespace std;

int PORT=6668;
string IP = "192.168.8.109";
#define BUFFER_SIZE 655355
int sock_cli;
struct sockaddr_in servaddr;
std::mutex mtx_dq;
vector<string> names{"YINBIDIAN_1","YINBIDIAN_2","ZHENCHA1_1","ZHENCHA1_2","ZHENCHA2_1",
	"ZHENCHA2_2","XUNLUO1_1","XUNLUO1_2","XUNLUO2_1","XUNLUO2_2","XUNLUO3_1","XUNLUO3_2","XUNLUO4_1","XUNLUO4_2"};
//char* SERVER_IP = "192.168.8.109";
//char* SERVER_IP = "192.168.1.131";


deque<detection_result::detection_result_msgConstPtr> msg_q;
void float_to_uchar(float b, unsigned char* sendbuf){
	int a = b * 100000;
	sendbuf[0] = a & 0xff;
	sendbuf[1] = (a & 0xff00) >> 8;
	sendbuf[2] = (a & 0xff0000) >> 16;
	sendbuf[3] = (a & 0xff000000) >> 24;
}
void detetction_cb(const detection_result::detection_result_msgConstPtr& msg){

	mtx_dq.lock();
	msg_q.push_back(msg);
	mtx_dq.unlock();
	//	unsigned char sendbuf[BUFFER_SIZE];
	//	unsigned char rec;
	//	cv_bridge::CvImageConstPtr ptr;
	//	ptr = cv_bridge::toCvCopy(msg->imgdata, sensor_msgs::image_encodings::RGB8);
	//	cv::Mat image = ptr->image;
	//	cv::resize(image,image,cv::Size(image.rows/2,image.cols/2 ));
	//	cv::imshow("mine",image);
	//	cv::waitKey(3);
	//	if(!image.empty()){
	//		std::vector<uchar> data_encode;
	//		std::vector<int> quality;
	//		quality.push_back(CV_IMWRITE_JPEG_QUALITY);
	//		quality.push_back(10);//进行50%的压缩
	//		imencode(".jpg", image, data_encode,quality);//将图像编码
	//		int nSize=data_encode.size();
	//		cout<<"'''''  "<<nSize<<endl;
	//		for (int i = 0; i < nSize; i++)
	//		{
	//			sendbuf[i] = data_encode[i];
	//		}
	//		send(sock_cli, sendbuf, nSize,0);
	//		recv(sock_cli, &rec, 1,0);
	//		if( abs(msg->longitude)>10){
	//			memset(sendbuf, 0, sizeof(sendbuf));
	//			float longi = msg->longitude;
	//			float_to_uchar(longi, sendbuf);
	//			float lati = msg->latitude;
	//			float_to_uchar(lati, sendbuf+4);
	//			float alti = msg->altitude;
	//			float_to_uchar(alti, sendbuf+8);
	//			send(sock_cli, sendbuf, 12,0); ///发送
	//			cout<<fixed<<setprecision(5)<<"the number i send is "<<longi<<" "<<lati<<" "<<alti<<endl;
	//		}
	////			detection_pub.publish(msg_send);
	//		memset(sendbuf, 0, sizeof(sendbuf));
	//	}
}
void process(){
	unsigned char flag;
	unsigned char img_buf[1001];
	img_buf[0] = 3;
	while(ros::ok()){
		if(msg_q.size()>0){
			cv::Mat img;
			mtx_dq.lock();
			detection_result::detection_result_msgConstPtr msgptr = msg_q.front();
			msg_q.pop_front();
			int index = 0;
			for(;index<names.size();index++){
				if(msgptr->header.frame_id == names[index])
				{
					cout<<index<<"    "<<names[index]<<"   "<<msgptr->header.frame_id<<endl;
					break;
				}
			}
			cout<<msgptr->header.frame_id<<endl;
			mtx_dq.unlock();
			cv_bridge::CvImageConstPtr ptr;
			ptr = cv_bridge::toCvCopy(msgptr->imgdata, sensor_msgs::image_encodings::RGB8);
			img = ptr->image;
			//			imshow("Yuanshi",img);
			//			cv::waitKey(3);

			//发送size信息
			int width = img.cols;
			int height = img.rows;
			char size_buf[5];
			size_buf[0] = 1;
			size_buf[1] = width & 0xff;
			size_buf[2] = (width & 0xff00) >> 8;
			size_buf[3] = height& 0xff;
			size_buf[4] = (height & 0xff00) >>8;
			send(sock_cli,size_buf,5,0);
			recv(sock_cli,&flag,1,0);
			//发送经纬度
			if(abs(msgptr->longitude)>5){
				unsigned char longibuf[13];
				if(msgptr->header.frame_id == "ZHENCHA1_1"){
					longibuf[0] = 21;
				}
				if(msgptr->header.frame_id == "ZHENCHA1_2"){
					longibuf[0] = 22;
				}
				float longi = msgptr->longitude;
				float_to_uchar(longi, longibuf+1);
				float lati = msgptr->latitude;
				float_to_uchar(lati, longibuf+5);
				float alti = 3.467;
				//				msg_send.altitude = alti;
				//				msg_send.latitude = lati;
				//				msg_send.longitude = longi;
				float_to_uchar(alti, longibuf+9);
				send(sock_cli, longibuf, 13,0); ///发送
				recv(sock_cli,&flag,1,0);

			}
			//send img
			int pos = 0;
			int all_buf_size = 3*width*height;
			while( pos < all_buf_size){
				memset(img_buf,0,1001);
				img_buf[0] = 3;
				if(all_buf_size - pos >= 1000){
					memcpy(img_buf+1, img.data+pos,1000);
					send(sock_cli,img_buf,1001,0);
					pos+=1000;
				}else{
					int size_send = all_buf_size - pos;
					memcpy(img_buf+1, img.data+pos,size_send);
					char a = char(index & 0xff);
					img_buf[size_send+1] = a;
					send(sock_cli,img_buf,size_send + 2,0);
					pos += all_buf_size - pos;
				}
				recv(sock_cli ,&flag, 1, 0);
				usleep(10);

			}

		}else{
			sleep(1);
		}
	}
}
int main(int argc, char ** argv)
{
	ros::init(argc, argv, "tcp_client");
	ros::NodeHandle nh;
	ros::Publisher detection_pub = nh.advertise<detection_result::detection_result_msg>("detection_result",100);
	ros::param::get("ip",IP);
	ros::param::get("~port",PORT);

	//定义sockfd
	sock_cli = socket(AF_INET,SOCK_STREAM, 0);
	///定义sockaddr_in
	memset(&servaddr, 0, sizeof(servaddr));
	servaddr.sin_family = AF_INET;
	servaddr.sin_port = htons(PORT);  ///服务器端口
	servaddr.sin_addr.s_addr = inet_addr(IP.c_str());  ///服务器ip

	printf("连接%s:%d\n",IP,PORT);
	///连接服务器，成功返回0，错误返回-1
	while(connect(sock_cli, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0 && ros::ok())
	{
		cout<<"wating server..."<<endl;
		usleep(100000);
	}
	printf("服务器连接成功\n");
	ros::Subscriber detection_sub = nh.subscribe<detection_result::detection_result_msg>("detection_result",100,detetction_cb);
	std::thread thread1{process};

	while(0){
		static int i = 0;
		i++;
		cout<<"sending width height"<<endl;
		int width = 1024;
		int height = 1204;
		unsigned char pic_size[4];
		pic_size[0] = width & 0xff;
		pic_size[1] = (width & 0xff00) >> 8;
		pic_size[2] = height& 0xff;
		pic_size[3] = (height & 0xff00) >>8;
		char a;
		send(sock_cli, pic_size, 4,0); ///发送
		recv(sock_cli, &a, 1,0);
		if(i>1000)
			break;
		usleep(1000);

	}

	unsigned char sendbuf[BUFFER_SIZE];
	unsigned char recvbuf[BUFFER_SIZE];
	while (0)//ros::ok()
	{
		static int i =0;
		stringstream str;
		str<<"/home/zx/图片/IMG/"<<i<<".png";
		cv::Mat imgread;
		imgread = cv::imread(str.str());
		if(!imgread.empty())
		{
			cv::resize(imgread,imgread,cv::Size(960,640));
			sensor_msgs::ImagePtr immsg ;
			immsg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", imgread).toImageMsg();
			detection_result::detection_result_msg msg_send;
			msg_send.imgdata = *immsg;
			msg_send.header.frame_id = names[i];

			cv::imshow("readimg", imgread);
			cv::waitKey(3);

			std::vector<uchar> data_encode;
			std::vector<int> quality;
			quality.push_back(CV_IMWRITE_JPEG_QUALITY);
			quality.push_back(10);//进行50%的压缩
			imencode(".jpg", imgread, data_encode,quality);//将图像编码
			int nSize=data_encode.size();

			//			cout<<"size is "<<nSize<<endl;
			for (int i = 0; i < nSize; i++)
			{
				sendbuf[i] = data_encode[i];
				//				cout<<i<<"   "<<nSize<<endl;
			}
			if( i == 1 || i == 2){

				float longi = 35.23456;
				float_to_uchar(longi, sendbuf);
				float lati = 110.16785;
				float_to_uchar(lati, sendbuf+4);
				float alti = 3.467;
				msg_send.altitude = alti;
				msg_send.latitude = lati;
				msg_send.longitude = longi;
				float_to_uchar(alti, sendbuf+8);
				send(sock_cli, sendbuf, 12,0); ///发送
				cout<<fixed<<setprecision(5)<<"the number i send is "<<longi<<" "<<lati<<" "<<alti<<endl;
			}
			detection_pub.publish(msg_send);
			memset(sendbuf, 0, sizeof(sendbuf));
			memset(recvbuf, 0, sizeof(recvbuf));
		}
		i++;
		usleep(1000000);
	}

	ros::spin();
	close(sock_cli);
	return 0;
}

