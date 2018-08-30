#define  _CRT_SECURE_NO_WARNINGS
#define _WINSOCK_DEPRECATED_NO_WARNINGS
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
using namespace cv;
using namespace std;

#define PORT 6666
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
// char* name2="result";
// char* name1="result22";

void thread11(){
	while(1){
		// cout<<"thread11"<<endl;
		cout << "Input the video channel: ";
		unsigned char a;
		cin>>a;
		sendto(socket_vedio, &a, 2, 0, (const sockaddr*)& client, len);
		usleep(100000);
	}
}
void thread22(){

	while(1){
		cout<<"regular......."<<endl;

		char buf[65536];
		std::vector<uchar> decode;
		int n = recvfrom(socket_vedio, buf, sizeof(buf), 0,(sockaddr *)& client,&len);//接受缓存
		int pos = 0;
		while (pos < n)
		{
			decode.push_back(buf[pos++]);//存入vector
		}
		imshoww = imdecode(decode, CV_LOAD_IMAGE_COLOR);
		if(!imshoww.empty()){
			cout<<"got image ..  "<<endl;
		imshow("vedio-result",imshoww);
		waitKey(4);
		}else{
			cout<<"got no image .."<<endl;
		}
//		if(buf[n-1]==0){
//			image_result = imdecode(decode, CV_LOAD_IMAGE_COLOR);//图像解码}
////			cout<<image_result.rows<<"  result 2222   "<<image_result.cols<<endl;
//
//		}if(buf[n-1]==1){
//			image_vedio = imdecode(decode, CV_LOAD_IMAGE_COLOR);
////			cout<<image_vedio.rows<<"   vedio  11111   "<<image_vedio.cols<<endl;
//
//		}
//		imshoww.create(image_vedio.rows+image_result.rows+3,image_vedio.cols+image_result.cols+3,CV_8UC3);
//
//		if(!image_vedio.empty()){
//			image_vedio.copyTo(imshoww(Rect(0,0,image_vedio.cols,image_vedio.rows)));
//		}
//		if(!image_result.empty()){
//			image_result.copyTo(imshoww(Rect(image_vedio.cols+3,0,image_result.cols,image_result.rows)));
//		}

	}
	close(socket_vedio);
}

void resultReceive(){
	while(1){
		cout<<"result ---------------"<<endl;
		Mat img;
		char buf[65536];
		std::vector<uchar> decode;
		int n = recvfrom(socket_result, buf, sizeof(buf), 0,(sockaddr *)& client_result,&len_result);//接受缓存
		int pos = 0;
		while (pos < n)
		{
			decode.push_back(buf[pos++]);//存入vector
		}
		img = imdecode(decode, CV_LOAD_IMAGE_COLOR);
		if(!img.empty()){
			cout<<"got image ..  "<<endl;
		imshow("anatoer-test",img);
		waitKey(4);
		}else{
			cout<<"got no image .."<<endl;
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
	cout<<"all begins now"<<endl;
	//	namedWindow(name2,CV_WINDOW_NORMAL);
	//	namedWindow(name1,CV_WINDOW_NORMAL);

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
	addr_result.sin_port = htons(PORT);         //设置端口号
	bind(socket_result, (sockaddr*)&addr_result, sizeof(addr_result));//绑定套接字

	std::thread thread1{thread11};
	std::thread thread2{thread22};
	std::thread thread3{resultReceive};
	//	imshoww.create(960+480+3,1280+640+3,CV_8UC3);
	thread1.join();
	thread2.join();
	thread3.join();

	namedWindow("vedio-result",CV_WINDOW_NORMAL);



	return 0;
}
