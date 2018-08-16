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
using namespace cv;
using namespace std;

enum
{
    PORT = 6666
};
int main(int argc, char** argv)
{
//    WSADATA wsaData;
//    WSAStartup(0x01, &wsaData); //创建初始化句柄
//
//    SOCKET m_sockServer;
    int m_sockServer;
    if ((m_sockServer = socket(AF_INET, SOCK_DGRAM, 0)) < 0)    //创建socket句柄，采用UDP协议
    {
        printf("create socket error: %s(errno: %d)\n", strerror(errno), errno);
        return -1;
    }
    if ((m_sockServer = socket(AF_INET, SOCK_DGRAM, 0)) < 0)    //创建socket句柄，采用UDP协议
    {
        printf("create socket error: %s(errno: %d)\n", strerror(errno), errno);
        return -1;
    }
    sockaddr_in m_servaddr;
    memset(&m_servaddr, 0, sizeof(m_servaddr));  //初始化结构体
    m_servaddr.sin_family = AF_INET;           //设置通信方式
    m_servaddr.sin_port = htons(6666);         //设置端口号

    bind(m_sockServer, (sockaddr*)&m_servaddr, sizeof(m_servaddr));//绑定套接字
    Mat image;
    char buf[65536];
    while (true)
    {
        std::vector<uchar> decode;

        int n = recv(m_sockServer, buf, sizeof(buf), 0);//接受缓存
        int pos = 0;
        while (pos < n)
        {
            decode.push_back(buf[pos++]);//存入vector
        }
        buf[n] = 0;
        image = imdecode(decode, CV_LOAD_IMAGE_COLOR);//图像解码
        imshow("image", image);
        waitKey(30);
    }
    return 0;
}
