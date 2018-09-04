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
#include <iomanip>
#include <opencv2/opencv.hpp>
#include <fstream>

const int  MYPORT=6668;
#define QUEUE   20
#define BUFFER_SIZE 655355
using namespace std;

int count_lati =0;
unsigned char imgbuf[7000000];
vector<string> names{"YIBIDIAN_1","YINBIDIAN_2","ZHENCHA1_1","ZHENCHA1_2","ZHENCHA2_1",
	"ZHENCHA2_2","XUNLUO1_1","XUNLUO1_2","XUNLUO2_1","XUNLUO2_2","XUNLUO3_1","XUNLUO3_2","XUNLUO4_1","XUNLUO4_2"};
void uchar_to_float(unsigned char* buffer, float& longi, float& lati,float& alti){
	//	unsigned char buffer[4];
	//	memcpy(buffer, buffer,4);
	longi = int ( buffer[0] | buffer[1] << 8 | buffer[2] <<16 | buffer[3] << 24) * 0.00001;
	//	memcpy(buffer,buffer+4,4);
	lati = int ( buffer[4] | buffer[5] << 8 | buffer[6] <<16 | buffer[7] << 24) * 0.00001;
	//	memcpy(buffer, buffer+8,4);
	alti = int ( buffer[8] | buffer[9] << 8 | buffer[10] <<16 | buffer[11] << 24) * 0.00001;
}
int main(int argc, char ** argv)
{
	ros::init(argc, argv, "tcp_server");
	ros::NodeHandle nh;
	string local_dir;
	ros::param::get("~local_dir", local_dir);
	ofstream outfile(local_dir+"/ZUOBIAO_KYXZ2018A.txt", ios::out);
	///定义sockfd
	int server_sockfd = socket(AF_INET,SOCK_STREAM, 0);

	///定义sockaddr_in
	struct sockaddr_in server_sockaddr;
	server_sockaddr.sin_family = AF_INET;
	server_sockaddr.sin_port = htons(MYPORT);
	server_sockaddr.sin_addr.s_addr = htonl(INADDR_ANY);

	///bind，成功返回0，出错返回-1
	if(bind(server_sockfd,(struct sockaddr *)&server_sockaddr,sizeof(server_sockaddr))==-1)
	{
		perror("bind");
		exit(1);
	}

	printf("监听%d端口\n",MYPORT);
	///listen，成功返回0，出错返回-1
	if(listen(server_sockfd,QUEUE) == -1)
	{
		perror("listen");
		exit(1);
	}

	///客户端套接字
	unsigned char buffer[BUFFER_SIZE];
	struct sockaddr_in client_addr;
	socklen_t length = sizeof(client_addr);

	printf("等待客户端连接\n");
	///成功返回非负描述字，出错返回-1
	int conn = accept(server_sockfd, (struct sockaddr*)&client_addr, &length);
	if(conn<0)
	{
		perror("connect");
		exit(1);
	}
	printf("客户端成功连接\n");
	cv::Mat img_recv;
	int width,height;
	int pos = 0;
	while(ros::ok())
	{

		vector<unsigned char> decode;
		memset(buffer,0,sizeof(buffer));
		int n = recv(conn, buffer, sizeof(buffer),0);
		if(buffer[0]==1){
			width = int(buffer[1] | buffer[2] << 8);
			height = int(buffer[3] | buffer[4] << 8);
			cout<<"the size of pic is"<<width<<"  "<<height<<endl;
			img_recv.create(height,width,CV_8UC3);//(cv::Size(width,height),CV_8UC3);
			cout<<"the size of pic is"<<width<<" ---------- "<<height<<endl;

		}
		if(buffer[0] == 21 ){
			count_lati++;
			float longi,lati,alti;
			uchar_to_float(buffer+1, longi,lati,alti);
			cout<<fixed<<setprecision(5)<<"receive  "<<longi<<" "<<lati<<" "<<alti<<endl;
			outfile<<"ZHENCHA1("<<longi<<" "<<lati<<" "<<alti<<")"<<endl;

		}
		if(buffer[0] == 22 ){
			count_lati++;
			float longi,lati,alti;
			uchar_to_float(buffer+1, longi,lati,alti);
			cout<<fixed<<setprecision(5)<<"receive  "<<longi<<" "<<lati<<" "<<alti<<endl;
			outfile<<"ZHENCHA2("<<longi<<" "<<lati<<" "<<alti<<")"<<endl;

		}
		if(count_lati == 2){
			outfile.close();
		}

		if(buffer[0] == 3){
			if(n == 1001){
				//				cout<<"got a full buf"<<endl;
				memcpy(imgbuf+pos,buffer+1,1000);
				pos+=1000;

			}else{
				cout<<"ending........."<<endl;
				memcpy(imgbuf+pos,buffer+1,n-2);
				unsigned char imgdata[3*width*height];
				memcpy(imgdata, imgbuf,3*width*height);
				img_recv.data = imgdata;
				pos = 0;
				memset(imgbuf,0,sizeof(imgbuf));
				int idx = int(buffer[n-1]);
				cout<<"idx  "<<idx<<endl;
				if(idx < 14)
				{
					stringstream str;
					str<<local_dir<<"/"<<names[idx]<<".jpg";
					cv::imwrite(str.str(),img_recv);
				}
				cv::imshow("got",img_recv);
				cv::waitKey(3);
			}

		}
		char y = 'i';
		send(conn, &y, 1, 0);
		//		if(n > 12){
		//			static int i = 0;
		//			i++;
		//			cout<<"------------ "<<n<<endl;
		//			while (pos < n)
		//			{
		//				decode.push_back(buffer[pos++]);//存入vector
		//			}
		//			//客户端发送exit或者异常结束时，退出
		//			if( n<=0)
		//				break;
		//			cv::Mat image_result = cv::imdecode(decode, CV_LOAD_IMAGE_COLOR);
		//			if(!image_result.empty()){
		//				stringstream str;
		//				str<<i<<".jpg";
		//				cv::imwrite(str.str(),image_result);
		//				cv::imshow("gottcp",image_result);
		//				cv::waitKey(3);
		//			}
		//		}
		//		if(n ==12) {//接收12个字节
		//			count_lati++;
		//			float longi,lati,alti;
		//			uchar_to_float(buffer, longi,lati,alti);
		//			cout<<fixed<<setprecision(5)<<"receive  "<<longi<<" "<<lati<<" "<<alti<<endl;
		//			outfile<<"ZHENCHA1("<<longi<<" "<<lati<<" "<<alti<<")"<<endl;
		//			if(count_lati == 2)
		//				outfile.close();
		//		}
		//		if(n == 4){
		//			static int i=0;
		//			i++;
		//			cout<<"before"<<int(buffer[0])<<"  "<<int(buffer[1])<<"  "<<int(buffer[2])<<"  "<<int(buffer[3])<<endl;
		//			int width = int(buffer[0] | buffer[1] << 8);
		//			int height = int(buffer[2] | buffer[3] << 8);
		//			cout<<"after"<<int(buffer[0])<<"  "<<int(buffer[1])<<"  "<<int(buffer[2])<<"  "<<int(buffer[3])<<endl;
		//			cout<<"the size of pic is"<<width<<"  "<<height<<"  "<<i<<endl;
		//		}

		//        cout<<int(buffer[0])<<" "<<int(buffer[1])<<" "<<int(buffer[2])<<" "<<int(buffer[3])<<" "<<endl;
		//        float b = int ( buffer[0] | buffer[1] << 8 | buffer[2] <<16 | buffer[3] << 24) * 0.00001;
		//        cout << "the server got "<<b<<"  from client "<<endl;

		//        usleep(100000);
	}
	//	ros::spin();
	close(conn);
	close(server_sockfd);
	return 0;
}

