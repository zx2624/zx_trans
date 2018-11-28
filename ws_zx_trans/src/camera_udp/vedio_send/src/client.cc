#ifdef _WIN32
#define  _CRT_SECURE_NO_WARNINGS
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <WinSock2.h>
#pragma comment(lib,"WS2_32.lib")
#else
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
#endif
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
using namespace std;

enum
{
    PORT = 6666
};
int main(int argc, char** argv)
{
#ifdef _WIN32
    WSADATA wsaData;
    WSAStartup(0x01, &wsaData); //创建初始化句柄
#endif
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
    m_servaddr.sin_addr.s_addr = inet_addr("192.168.199.204");
    m_servaddr.sin_port = htons(6666);//设置需要发送的IP和端口号
    bind(m_sockClient, (sockaddr*)&m_servaddr, sizeof(m_servaddr));//绑定端口号
    VideoCapture capture(0);//打开摄像头
    Mat image;
    while (true)
    {
        capture >> image;//读入图片
        if (image.empty())    //如果照片为空则退出
        {
            printf("empty image\n\n");
            return -1;
        }
        std::vector<uchar> data_encode;
        std::vector<int> quality;
        quality.push_back(CV_IMWRITE_JPEG_QUALITY);
        quality.push_back(50);//进行50%的压缩
        imencode(".jpg", image, data_encode,quality);//将图像编码
        char encodeImg[65535];

        int nSize = data_encode.size();
        for (int i = 0; i < nSize; i++)
        {
            encodeImg[i] = data_encode[i];
        }

        sendto(m_sockClient, encodeImg, nSize, 0, (const sockaddr*)& m_servaddr, sizeof(m_servaddr));
        memset(&encodeImg, 0, sizeof(encodeImg));  //初始化结构体
    }
    return 0;
}
