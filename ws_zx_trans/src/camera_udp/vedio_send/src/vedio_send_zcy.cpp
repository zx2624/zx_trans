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
#include "lanelet_map_msgs/Way.h"
#include "plan2control_msgs/Trajectory.h"
using namespace std;
using namespace cv;
int PORT = 13010;
int PORT2 = 9999;
string IP = "192.168.199.205";
int heartbeat = 0;
//std::vector<cv::Mat> srcImage;
std::mutex mtx_0;
std::mutex mtx_1;
std::mutex mtx_2;
std::mutex mtx_gps;
std::mutex mtx_status;
std::mutex mtx_path;
std::mutex mtx_local_path;
double longitude = 100, altitude = 100, latitude = 100;
float speed = 0;
unsigned char gear;
typedef struct IMGMAT{
	unsigned char head1;//the angle of camera turning
	unsigned char head2;
	unsigned int len;
	unsigned char coord[65535];
}imagemat;

vector<double> globalPos_x;
vector<double> globalPos_y;
typedef struct CamCoord{
	unsigned char head1;//the angle of camera turning
	unsigned char head2;
	unsigned int len;
	double coord[65535];
}camcoord;

sockaddr_in client;
typedef struct Data
{
	//header head;
	unsigned char h1;
	unsigned char h2;
	unsigned char type;
	unsigned char len1;
	unsigned char len2;
	char data[65535-5];

}Data;

typedef struct ControlData
{
	unsigned char h1;
	unsigned char h2;
	unsigned char power;
	unsigned char start;
	unsigned char gear;
	//    float steer;
	//    float throttle;
	unsigned char s1;
	unsigned char s2;
	unsigned char s3;
	unsigned char s4;

	unsigned char t1;
	unsigned char t2;
	unsigned char t3;
	unsigned char t4;

	unsigned char park;
	unsigned char emergency;
	unsigned char Break;
	unsigned char mode;
} controlData;

typedef union
{
	float fdata;
	unsigned long ldata;
}FloatLongType;


double a2=6378137;
double e2= 0.0818192*0.0818192;
vector<double>  localPos_x;
vector<double>  localPos_y;
//套接字和对应的句柄，用udp等网络编程在linux下都需要经过套接字，当时是为了
//以不同的频率向不同的端口发送信息，所以用了两个套接字
sockaddr_in server_vedio;
int socket_vedio;
sockaddr_in socket_add_result;
int socket_result;

//为了获取笔记本摄像头图像
VideoCapture capture(0);//打开摄像头

//当时为了发送三路图像，定义了三个Mat
Mat image_0,image_1,image_2;

//图片压缩过后会有大概四万还是多少个字节来着，放到下面这个数组里
uchar encodeImg[6553555];

//这两个time应该是为了控制频率
double lasttime=-1;
double lasttime_1 = -1;

//用来接收服务器发过来的命令
unsigned char flag_channel='2';

socklen_t len;


//这个函数可以不用理会，就是当时为了调试用的
void computercam(){
	while(ros::ok()){
		double timenow = ros::Time::now().toSec();
		//控制图片进来的频率 ，电脑摄像头频率太高，这个条件可以降低发送的频率
		if(lasttime != -1 && timenow - lasttime < 0.1){
			//			lasttime = timenow;
			capture>>image_1;
			continue;
		}
		lasttime = timenow;
		mtx_1.lock();
		if(capture.isOpened())
			capture>>image_1;
		mtx_1.unlock();
	}
}
//下面这几个callback函数只是把图像存到对应的Mat里
void probecb(const object_detector_msgs::laser_electronic_resultConstPtr& msg){

	mtx_0.lock();
	image_0 = (cv_bridge::toCvCopy(msg->image_data)->image).clone();
	mtx_0.unlock();
}

void front_vedio_cb(const sensor_msgs::ImageConstPtr& orgmsg){
	mtx_1.lock();
	image_1=(cv_bridge::toCvShare(orgmsg)->image).clone();
	cv::resize(image_1,image_1,cv::Size(960,640));
	mtx_1.unlock();
}
void leftimageCb(const sensor_msgs::ImageConstPtr& orgmsg){
	mtx_0.lock();
	image_0 = (cv_bridge::toCvCopy(orgmsg)->image).clone();
	mtx_0.unlock();
}
void screen_cb(const sensor_msgs::ImageConstPtr& orgmsg)
{
	mtx_2.lock();
	image_2 = (cv_bridge::toCvShare(orgmsg)->image).clone();
	cv::resize(image_2,image_2,cv::Size(image_2.cols/2,image_2.rows/2));
	mtx_2.unlock();

}
//将一个float变成四个字节，比较傻的方法
void float_to_uchar(float b, unsigned char* sendbuf){
	int a = b * 100000;
	sendbuf[0] = a & 0xff;
	sendbuf[1] = (a & 0xff00) >> 8;
	sendbuf[2] = (a & 0xff0000) >> 16;
	sendbuf[3] = (a & 0xff000000) >> 24;
}

void command(){
	//recv
	while(ros::ok())
	{
		unsigned char buf[1];
		//接收从服务器（遥控端）发回来的命令
		int n = recvfrom(socket_vedio, buf, sizeof(buf), 1,(sockaddr *)& server_vedio, &len);//接受缓存
		if (n > 0) {
			if(buf[0]=='0'||buf[0]=='1'||buf[0]=='2')
			{
				flag_channel = buf[0];
				cout << "read command from server: " << flag_channel << endl;
			}
		}
		std::cout << "in while .." << std::endl;
	}
	std::cout << " command done " << std::endl;
}
void process(){
	while(ros::ok()){
//		std::cout << "in process --" << PORT << std::endl;
		double timenow=ros::Time::now().toSec();
		lasttime = timenow;
		Mat imgsend;
		//根据接收到的命令将imgsend指向对应的图像进行发送
		//		if(flag_channel == '0'){
		//			mtx_0.lock();
		//			imgsend = image_0.clone();
		//			mtx_0.unlock();
		//		}else if(flag_channel == '1'){
		//			mtx_1.lock();
		//			imgsend = image_1.clone();
		//			mtx_1.unlock();
		//		}else if(flag_channel == '2'){
		//			mtx_2.lock();
		//			imgsend = image_2.clone();
		//			mtx_2.unlock();
		//		}else{
		//			cout<<"wtf"<<endl;
		//		}

		//截图
		mtx_2.lock();
		imgsend = image_1.clone();
		mtx_2.unlock();
		if(!imgsend.empty()){
			imagemat screenImg;
			screenImg.head1 = 0xF4;
			screenImg.head2 = 0x03;
			screenImg.len = localPos_x.size();
			//			//图像压缩
			std::vector<uchar> data_encode;
			std::vector<int> quality;
			quality.push_back(CV_IMWRITE_JPEG_QUALITY);
			quality.push_back(60);//进行50%的压缩
			imencode(".jpg", imgsend, data_encode,quality);//将图像编码
			screenImg.len =data_encode.size();
			//cout << "image size " << screenImg.len << endl;
			//图像压缩完毕

			//将压缩好的图像（变成了一个个存在vector里的字节）写到buf里准备发送
			for (int i = 0; i < screenImg.len; i++)
			{
				screenImg.coord[i] = data_encode[i];
			}
			//显示
//			Mat image_result = imdecode(data_encode, CV_LOAD_IMAGE_COLOR);
//			int u1=1;
//			char imageFilePath1[200];
//			sprintf(imageFilePath1, "%s%d%s", "/home/zcy/", u1++, ".png");
//			imwrite(imageFilePath1, image_result);

			//			//发送
			char *sPack = (char *) &screenImg;
			sendto(socket_vedio, sPack,  screenImg.len+6, 0, (const sockaddr*)& server_vedio, sizeof(server_vedio));
			memset(&encodeImg, 0, sizeof(encodeImg));
		}






		usleep(100000);
	}


}

void receive(){
	while(ros::ok()){
		//接收任务文件
		unsigned char buf[65535];
		int n = recvfrom(socket_vedio, buf, sizeof(buf), 0,(sockaddr *)& client,&len);//接受缓存
		if ((buf[0] == 0xF3) && (buf[1] == 0x00)) {
			controlData control;
			memcpy(&control, buf, sizeof(ControlData));
			//            int power = buf[2], start = buf[3], gear = buf[4], park = buf[13], emergencyBreak = buf[14], Break = buf[15], mode = buf[16];
			//            int steer1 = buf[5], steer2 = buf[6], steer3 = buf[7], steer4 = buf[8], throttle1 = buf[9], throttle2 = buf[10], throttle3 = buf[11], throttle4 = buf[12];
			cout << "电源总开关 " << (int) control.power << endl;
			cout << "车辆启动 " << (int) control.start << endl;
			cout << "档位 " << (int) control.gear << endl;
			//            cout << "转向 " << control.steer << endl;
			//            cout << "油门 " << control.throttle << endl;
			//            cout << "转向 " << hex << " " << (int) control.s1 << " " << (int) control.s2 << " " << (int) control.s3 << " " << (int) control.s4 << endl;
			//            cout << "油门 " << hex << " " << (int) control.t1 << " " << (int) control.t2 << " " << (int) control.t3 << " " << (int) control.t4 << endl;
			FloatLongType fls;
			fls.ldata=0;
			fls.ldata = control.s4;
			fls.ldata=(fls.ldata<<8)|control.s3;
			fls.ldata=(fls.ldata<<8)|control.s2;
			fls.ldata=(fls.ldata<<8)|control.s1;
			float fs = fls.fdata;
			cout << "转向 " << 4 * fs << endl;

			FloatLongType flt;
			flt.ldata=0;
			flt.ldata = control.t4;
			flt.ldata=(flt.ldata<<8)|control.t3;
			flt.ldata=(flt.ldata<<8)|control.t2;
			flt.ldata=(flt.ldata<<8)|control.t1;
			float ft = flt.fdata;
			cout << "油门 " << ft << endl;

			cout << "驻车 " << (int) control.park << endl;
			cout << "紧急停车 " << (int) control.emergency << endl;
			cout << "制动 " << (int) control.Break << endl;
			cout << "工作方式 " << (int) control.mode << endl;
			cout << "controlling" << endl;

			//                        cout << "电源总开关 " << power << endl;
			//            cout << "车辆启动 " << start << endl;
			//            cout << "档位 " << gear << endl;
			//            cout << dec << "转向 " << str << endl;
			//
			//            cout << dec << "油门 " << tht << endl;
			//
			//            cout << dec << "驻车 " << park << endl;
			//            cout << "紧急停车 " << emergencyBreak << endl;
			//            cout << "制动 " << Break << endl;
			//            cout << "工作方式 " << mode << endl;
			//            cout << "controlling" << endl;

		}

		//                if ((buf[1]==0xF4)&&(buf[0]==0x04))
		//                   {
		//                    cout<<"got task  point  "<<endl;
		//                   }
		//        struct header header_udp;
		//                memcpy(&header_udp,buf,sizeof(buf));
		//                memcpy(&header_udp,buf,sizeof(header_udp)+1);
		else if ((buf[0] == 0xF3) && (buf[1] == 0x10)) {
			Data AA;
			memcpy(&AA, buf, sizeof(Data));

			unsigned short length = AA.len1;
			//    length=length|AA.len1;
			//    length=length<<8;
			//    length=length|AA.len2;
			//    cout<<"length"<<length<<endl;



			//                unsigned short  data_len=AA.len;
			//                cout<<sizeof(data_len)<<endl;
			//                cout<<data_len<<endl;

			//                unsigned short  data=AA.data;
			//                cout<<sizeof(data)<<endl;
			//                cout<<data<<endl;

			//FILE *fp = fopen("/home/zcy/taskfile/jidong.txt", "w");
			//                if(AA.type==0x01)
			//                  fp = fopen("/home/zcy/taskfile/jidong", "w");
			//                else if(AA.type==0x02)
			//                  fp = fopen("/home/zcy/taskfile/zhencha", "w");
			//                else if(AA.type==0x03)
			//                  fp = fopen("/home/zcy/taskfile/xunluo", "w");

			cout << "got task  point  ------zxzxzxzxzx-------- " << endl;
			std::cout << "the lengh is ......" << length << std::endl;

			//        string s;
			//        s=AA.data;
			std::fstream f;
			f.open("/home/zx/taskfile/jidong.txt", ios::out | ios::binary);
			f.write(AA.data, strlen(AA.data));
			f.close();

			//         int nwrite = fwrite(AA.data, sizeof(char), 65535, fp);
			//         fflush(fp);
			//         int duphandle;
			//         duphandle=dup(fileno(fp));
			//         close(duphandle);
			//         fclose(fp);

		}
		//接收任务命令
		//        测试接收控制命令
		//        unsigned char buf_control[11];
		//       int c = recvfrom(socket_vedio, buf_control, sizeof(buf_control), 0,(sockaddr *)& client,&len);

		usleep(10000);
	}

}

void gpscallback(const sensor_driver_msgs::GpswithHeadingConstPtr& gpsmsg){
	mtx_gps.lock();
	latitude = gpsmsg->gps.latitude;
	longitude = gpsmsg->gps.longitude;
	altitude = gpsmsg->gps.altitude;
	mtx_gps.unlock();
	//        cout<<"latitude: "<<latitude<<endl;
}
void statuscallback(const control_msgs::GetECUReportConstPtr& status_msg){
	//std::cout << "got ecudata -- " << std::endl;
	mtx_status.lock();

	speed = (float) status_msg->speed.velocity.linear.x;
	gear = (char)status_msg->shift_cur.gear;
	mtx_status.unlock();

}
void pathcallback(const lanelet_map_msgs::Way &global_path_msg){
	//        cout<<"lanelet_map_msgs.points.size(): "<<global_path_msg.points.size()<<endl;
	mtx_path.lock();
	globalPos_x.clear();
	globalPos_y.clear();
	for(int i=0;i<global_path_msg.points.size();i++) {
		globalPos_x.push_back( global_path_msg.points.at(i).point.x);
		globalPos_y.push_back( global_path_msg.points.at(i).point.y);
	}
	mtx_path.unlock();

}
void local_pathcallback(const plan2control_msgs::Trajectory &path_msg){
	//        cout<<"plan2control_msgs.points.size(): "<<path_msg.points.size()<<endl;
	mtx_local_path.lock();
	localPos_x.clear();
	localPos_y.clear();
	for(int i=0;i<path_msg.points.size();i++) {
		localPos_x.push_back(path_msg.points.at(i).position.x);
		localPos_y.push_back(path_msg.points.at(i).position.y);
	}
	mtx_local_path.unlock();

}
int main(int argc, char ** argv)
{
	ros::init(argc, argv, "vedio_send");
	ros::NodeHandle nh;

	//从launch文件里面获得需要发送到的目标Ip地址，端口等信息，这些有默认值（写在上面全局变量）
	ros::param::get("ip",IP);
	ros::param::get("~port",PORT);
	ros::param::get("~port2",PORT2);
	cout<<"========视频传输客户端IP=========="<<IP<<endl;
	cout<<"========视频传输客户端PORT=========="<<PORT<<endl;
	cout<<"========PORT2=========="<<PORT2<<endl;
	//	ImageDeal imgd(nh);
	int count = 0;

	//下面是发送信息前的一些准备工作，主要是把套接字跟目标的IP和端口绑定起来，
	//这样信息就可以通过套接字发送到指定IP地址的电脑上的制定端口，
	//接收端电脑（服务器）只需要监听自己的这个端口就可以收到信息了
	if ((socket_vedio = socket(AF_INET, SOCK_DGRAM, 0)) < 0)   //创建socket句柄，采用UDP协议
	{
		printf("create socket error: %s(errno: %d)\n", strerror(errno), errno);
		return -1;
	}
	memset(&server_vedio, 0, sizeof(server_vedio));  //初始化结构体
	server_vedio.sin_family = AF_INET;           //设置通信方式
	server_vedio.sin_addr.s_addr = inet_addr(IP.c_str());
	server_vedio.sin_port = htons(PORT);//设置需要发送的IP和端口号
	bind(socket_vedio, (sockaddr*)&server_vedio, sizeof(server_vedio));//绑定端口号

	//下面这个套接字程序里面没用，因为后来程序改了，不需要发送不同频率的东西了
	//所以你可以用来发送点云
	if ((socket_result = socket(AF_INET, SOCK_DGRAM, 0)) < 0)    //创建socket句柄，采用UDP协议
	{
		printf("create socket error: %s(errno: %d)\n", strerror(errno), errno);
		return -1;
	}
	memset(&socket_add_result, 0, sizeof(socket_add_result));  //初始化结构体
	socket_add_result.sin_family = AF_INET;           //设置通信方式
	socket_add_result.sin_addr.s_addr = inet_addr(IP.c_str());
	socket_add_result.sin_port = htons(PORT2);//设置需要发送的IP和端口号
	bind(socket_result, (sockaddr*)&socket_add_result, sizeof(socket_add_result));//绑定端口号


	//下面是订阅的三路图像
	//	ros::Subscriber sub_front=nh.subscribe("/image", 1,front_vedio_cb);//前视摄像头数据
	ros::Subscriber sub_screeb = nh.subscribe("screen_image_topic", 1, screen_cb);//屏幕截图
	//	ros::Subscriber sub_probe = nh.subscribe("realtime_video_topic", 1, probecb);//侦察视频
	//	ros::Subscriber sub_left = nh.subscribe("/left_image", 1, leftimageCb);//左侧相机--比赛时用他来代替侦察相机
	//todo:这里可以增加一个接收点云的回调函数，接收到之后直接在该函数里发送即可

	//这两个是后来添加的话题，唐波需要我发过去
	ros::Subscriber sub_gps = nh.subscribe("/sensor_fusion_output", 1, gpscallback);//只用了这里面的经纬高度
	ros::Subscriber sub_status = nh.subscribe("ecudatareport", 1, statuscallback);//只发送了这里面的档位和速度
	//全局规划任务点 zcy0113
	ros::Subscriber sub_path = nh.subscribe("/topology_global_path", 1, pathcallback);//

	//局部规划任务点 zcy
	ros::Subscriber sub_Localpath = nh.subscribe("/global_path/traj_plan", 1, local_pathcallback);//
	//接收命令线程 -- 这个线程是用来接收服务器（或者是遥控端）发过来的命令，
	//根据命令发送对应的三路图像中的一路
	//std::thread thread3{command};//zcy

	//整体的发送在这里面进行
	std::thread thread4{process};
	std::thread thread_receive{receive};
	std::thread thtead_computercam{computercam};
	//	thread4.join();
	ros::spin();
	close(socket_vedio);

}
