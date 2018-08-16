
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
//using namespace std;
using namespace cv;
#define PORT 6666
#define IP "192.168.199.149"
//std::vector<cv::Mat> srcImage;
std::mutex mtx;
class ImageDeal
{
    ros::NodeHandle& _nh;
    ros::Subscriber _sub;

public:
    cv::Mat _srcImage;
    ImageDeal(ros::NodeHandle& nh):_nh(nh)
    {
        _sub=_nh.subscribe("/image", 1, &ImageDeal::imageCb, this);

    }
    ~ImageDeal()
    {
        cv::destroyAllWindows();
    }

    void imageCb(const sensor_msgs::ImageConstPtr& orgmsg)
    {
    	mtx.lock();
        _srcImage = (cv_bridge::toCvShare(orgmsg)->image).clone();
        mtx.unlock();
        if(_srcImage.empty())
            std::cout << "Can't get image!" << std::endl;
        else
            std::cout << "Get one image!" << std::endl;
    }
};



int main(int argc, char ** argv)
{
    ros::init(argc, argv, "sub_one_image");
    ros::NodeHandle nh;
    ImageDeal imgd(nh);
    int count = 0;
    cv::Mat srcImage;
//    VideoCapture capture(0);//打开摄像头
    while (ros::ok())
    {
        ros::spinOnce();
        mtx.lock();
        srcImage = imgd._srcImage.clone();
        mtx.unlock();

//    	capture >> srcImage;//读入图片
        if(!srcImage.empty())
        {
            int m_sockClient;
            if ((m_sockClient = socket(AF_INET, SOCK_DGRAM, 0)) < 0)    //创建socket句柄，采用UDP协议
            {
                printf("create socket error: %s(errno: %d)\n", strerror(errno), errno);
                return -1;
            }
            sockaddr_in m_servaddr;
            memset(&m_servaddr, 0, sizeof(m_servaddr));  //初始化结构体
            m_servaddr.sin_family = AF_INET;           //设置通信方式
            m_servaddr.sin_port = htons(PORT);         //设置端口号
            m_servaddr.sin_addr.s_addr = inet_addr(IP);
            m_servaddr.sin_port = htons(6666);//设置需要发送的IP和端口号
            bind(m_sockClient, (sockaddr*)&m_servaddr, sizeof(m_servaddr));//绑定端口号

//            while (true)
//            {

                std::vector<uchar> data_encode;
                std::vector<int> quality;
                quality.push_back(CV_IMWRITE_JPEG_QUALITY);
                quality.push_back(50);//进行50%的压缩
                imencode(".jpg", srcImage, data_encode,quality);//将图像编码
                char encodeImg[65535];

                int nSize = data_encode.size();
                for (int i = 0; i < nSize; i++)
                {
                    encodeImg[i] = data_encode[i];
                }

                sendto(m_sockClient, encodeImg, nSize, 0, (const sockaddr*)& m_servaddr, sizeof(m_servaddr));
                memset(&encodeImg, 0, sizeof(encodeImg));  //初始化结构体
//            }


        	++count;
            cv::imshow("sub_image", srcImage);
            cv::waitKey(30);
        }
    }

    return 0;
}
