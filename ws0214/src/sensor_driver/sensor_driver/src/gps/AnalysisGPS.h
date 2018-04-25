
#include <sys/types.h>
#include <sys/socket.h>
#include <stdio.h>
#include <string.h>
#include <netinet/in.h>
#include <math.h>
#include <fstream>
#include <unistd.h>
#include  <iostream>
#include<stdlib.h>


#define BUFSIZE 4096
typedef char                 UINT;
typedef unsigned char                 UCHAR;
/******************GPS  Model output***************************************/
 struct struct_GPS
{
	double dLongitude;
	double dLatitude;
	double HDOP;//水平精度因子
	double VTG_V;//VTG格式的速度{gga}
	double VTG_heading;
	double  PHDT_heading;
	double PTRA_heading;
	//double HDOP;
	int DGPSState;//0:初始化，1：单点定位；2：码差分；3：无效PPS；4：固定解；5：浮点解；6：正在估算；7：人工输入固定值；8：模拟模式；9：WAAS差分。
	char	DGPSState_VTG;//A自主定位；D差分；E估算。
	char	DGPSState_HDT;
	double PASHR_heading;//海梁项目专用。qjy 20170928

	double	lTimeStamp;			// 时间戳(Unit:ms)
};



class CAnalysisGPS
{
   public:
	 struct_GPS   GPSData_struct;
    CAnalysisGPS(); 
    ~CAnalysisGPS(); 
    bool Init(int port);
    void Update();
  private:
    void GPSDataGPGGA(char GPSData[],int n);

//	void GPSDataGPRMC(char GPSData[],int n);
	void GPSDataGPVTG(char GPSData[],int n);
	void GPSDataGPHDT(char GPSData[],int n);
	void GPSDataGPTRA(char GPSData[],int n);
	void GPSDataPASHR(char GPSData[],int n);//qjy 20170928 海梁项目

    char m_cGPSDataFromReceiver[100];
    bool CreatNetSucess;


    sockaddr_in myaddr; /* our address */
    sockaddr_in remaddr; /* remote address */
    socklen_t addrlen; /* length of addresses */

    int recvlen; /* # bytes received */
    int fd; /* our socket */
    unsigned char buf[BUFSIZE]; /* receive buffer */

//    std::fstream data_backup;
};
