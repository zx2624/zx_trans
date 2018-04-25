#include "AnalysisGPS.h"



CAnalysisGPS::CAnalysisGPS()
{

}

CAnalysisGPS::~CAnalysisGPS()
{

}


bool CAnalysisGPS::Init(int port)
{

    addrlen = sizeof(myaddr); /* length of addresses */
    /* create a UDP socket */
    if((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
	perror("cannot create socket\n");
	return false;
    }
    /* bind the socket to any valid IP address and a specific port */
    memset((char*)&myaddr, 0, sizeof(myaddr));
    myaddr.sin_family = AF_INET;
    myaddr.sin_addr.s_addr =htonl(INADDR_ANY);
    myaddr.sin_port = htons(port);
    if(bind(fd, (sockaddr *)&myaddr, sizeof(myaddr)) < 0)
    {
	perror("bind failed GPS");
	return false;
    }
   return true;

  }



void CAnalysisGPS::Update()
{


    recvlen = recvfrom(fd, buf, BUFSIZE, 0, (struct sockaddr *)&remaddr, &addrlen);
    if(recvlen >10)
    {
				{

						static char static_cGPSHeadFlag			 = 0;						//接收到头标志
						static char static_cGPSTailFlag			 = 0;						//接收到尾标志
						static char static_cGPSOverFlag			 = 0;						//接收完成标志
						static char static_cGPSRXDataNum		 = 0;						//接收数据计数器
						static char static_cGPSRXAfterStarDataNum = 0;						//接收数据计数器
						static char static_cGPSRXDataChecksum	 = 0;						//数据校验
						static char static_cGPSRXDataCsc[5];								//接收到的校验位
						char		cTempGPSRXDataCsc1			 = 0;						//数据校验高4位
						char		cTempGPSRXDataCsc2			 = 0;						//数据校验低4位
						char		cTemp						 = 0;

						int bufflenth;
						int readlenth;

	for(int i = 0; i<recvlen; i++)
	{

		cTemp = buf[i];
		if('$' == cTemp)
		{
			static_cGPSHeadFlag = 1;
			static_cGPSTailFlag = 0;
			static_cGPSOverFlag = 0;
			static_cGPSRXDataChecksum = 0;
			static_cGPSRXDataNum = 0;
			static_cGPSRXAfterStarDataNum = 0;
			memset(m_cGPSDataFromReceiver,0,100);
			m_cGPSDataFromReceiver[static_cGPSRXDataNum++] = cTemp;

		}
		else
		{
			if(1 == static_cGPSHeadFlag)
			{
				m_cGPSDataFromReceiver[static_cGPSRXDataNum++] = cTemp;
				if('*' == cTemp)
				{
					static_cGPSTailFlag = 1;
				}
				if(1 == static_cGPSTailFlag)
				{
					static_cGPSRXDataCsc[static_cGPSRXAfterStarDataNum++] = cTemp;

					if(3 == static_cGPSRXAfterStarDataNum)
					{
						//分解计算校验值的高低4位
						cTempGPSRXDataCsc1 = ((static_cGPSRXDataChecksum >> 4) & 0x0F);
						cTempGPSRXDataCsc2 = static_cGPSRXDataChecksum & 0x0F;

						if(cTempGPSRXDataCsc1 < 10)
							cTempGPSRXDataCsc1  += '0';
						else cTempGPSRXDataCsc1 += 'A' - 10;

						if(cTempGPSRXDataCsc2 < 10)
							cTempGPSRXDataCsc2  += '0';
						else cTempGPSRXDataCsc2 += 'A' - 10;

						//校验正确
						//if((static_cGPSRXDataCsc[1] == cTempGPSRXDataCsc1)&&
						//	(static_cGPSRXDataCsc[2] == cTempGPSRXDataCsc2))
						{
							char GPSHead[10];

							memset(GPSHead,0,10);
							memcpy(GPSHead,m_cGPSDataFromReceiver,6);
							if(0 == strcmp(GPSHead,"$GPGGA")||0 == strcmp(GPSHead,"$GNGGA"))
							{
								GPSDataGPGGA(m_cGPSDataFromReceiver,static_cGPSRXDataNum-1);
							}
						/*	else if(0 == strcmp(GPSHead,"$GPRMC")||0 == strcmp(GPSHead,"$GNRMC"))
							{
								GPSDataGPRMC(m_cGPSDataFromReceiver,static_cGPSRXDataNum-1);
							}*/
							else if(0 == strcmp(GPSHead,"$GPVTG")||0 == strcmp(GPSHead,"$GNVTG"))
							{
								GPSDataGPVTG(m_cGPSDataFromReceiver,static_cGPSRXDataNum-1);
							}
							else if(0 == strcmp(GPSHead,"$GPHDT")||0 == strcmp(GPSHead,"$GNHDT"))
							{
								GPSDataGPHDT(m_cGPSDataFromReceiver,static_cGPSRXDataNum-1);
							}
							else if(0 == strcmp(GPSHead,"$GPTRA")||0 == strcmp(GPSHead,"$GNTRA"))
							{
								GPSDataGPTRA(m_cGPSDataFromReceiver,static_cGPSRXDataNum-1);
							}
							else if(0 == strcmp(GPSHead,"$PASHR"))
							{
								GPSDataPASHR(m_cGPSDataFromReceiver,static_cGPSRXDataNum-1);
							}

							else
							{
								static_cGPSOverFlag = 1;
							}





						}
					}
				}
				else
				{
					static_cGPSRXDataChecksum ^= cTemp;
				}


				if(static_cGPSRXDataNum >= 100)
				{
					static_cGPSOverFlag = 1;
				}
			}
		}

		if(1 == static_cGPSOverFlag)
		{
			static_cGPSHeadFlag			 = 0;
			static_cGPSOverFlag			 = 0;
			static_cGPSTailFlag			 = 0;
			static_cGPSRXDataChecksum	 = 0;
			static_cGPSRXDataNum		 = 0;
			static_cGPSRXAfterStarDataNum = 0;
			memset(m_cGPSDataFromReceiver,0,100);
		}
	}
	}
    }
}

void CAnalysisGPS::GPSDataGPGGA(char GPSData[],int n)
{

	double dTemp = 0.0;
	double dTemp1 = 0.0,dTemp2 = 0.0;
	UINT  unTemp = 0;
	long  lTemp = 0;
//	CString strTemp;

	int i = 0;
	int j = 0;
	int GPSCommaNumber = 0;
	char GPSDataTemp[15];

	memset(GPSDataTemp,0,15);

	while(i++ <= n)
	{

		if(GPSData[i] == ',')		//找到逗号
		{
			j = 0;
			GPSCommaNumber++;
			switch(GPSCommaNumber)
			{
			case 1: // $GPGGA
				memset(GPSDataTemp,0,15);
				break;
			case 2: // UTC
				lTemp = atof(GPSDataTemp);
				//strTemp.Format(_T("%d:%d:%d"),(lTemp/10000),(lTemp%10000/100),(lTemp%100));
				//m_ctrlEditUTC.SetWindowTextW(strTemp);
				//m_sDGPSData.dUTC=lTemp;
				memset(GPSDataTemp,0,15);
				break;
			case 3: // Latitude
				dTemp=atof(GPSDataTemp);
				unTemp = (UINT)(dTemp/100);
				dTemp1 = unTemp + (dTemp - unTemp * 100.0) / 60.0;
				GPSData_struct.dLatitude=dTemp1;
				memset(GPSDataTemp,0,15);
				break;
			case 4: // Latitude hemisphere
				//strTemp.Format(_T("%C:%.7f"),GPSDataTemp[0],dTemp1);
				//m_ctrlEditLat.SetWindowTextW(strTemp);
				memset(GPSDataTemp,0,15);
				break;
			case 5: // Longitude
				dTemp=atof(GPSDataTemp);
				unTemp = (UINT)(dTemp/100);
				dTemp1 = unTemp + (dTemp - unTemp * 100.0) / 60.0;
				GPSData_struct.dLongitude=dTemp1;


				memset(GPSDataTemp,0,15);
				break;
			case 6: // Longitude hemisphere
				//strTemp.Format(_T("%C:%.7f"),GPSDataTemp[0],dTemp1);
				//m_ctrlEditLon.SetWindowTextW(strTemp);
				memset(GPSDataTemp,0,15);
				break;
			case 7: // GPS quality
				unTemp = atoi(GPSDataTemp);
				//strTemp.Format(_T("%d"),unTemp);
				//m_ctrlEditQua.SetWindowTextW(strTemp);
				GPSData_struct.DGPSState=unTemp;
				memset(GPSDataTemp,0,15);
				break;
			case 8: // GPS number
				unTemp = atoi(GPSDataTemp);
				//strTemp.Format(_T("%d"),unTemp);
				//m_ctrlEditStarNum.SetWindowTextW(strTemp);
		//		m_sDGPSData.ucStarNum=unTemp;
				memset(GPSDataTemp,0,15);
				break;
			case 9: // HDOP
				dTemp = atof(GPSDataTemp);
				//strTemp.Format(_T("%.1f"),dTemp);
				//m_ctrlEditHdop.SetWindowTextW(strTemp);
				GPSData_struct.HDOP=dTemp;
				memset(GPSDataTemp,0,15);
				break;
			case 10: // Antenna height
				dTemp = atof(GPSDataTemp);
				//strTemp.Format(_T("%.1f"),dTemp);
				//m_ctrlEditAlt.SetWindowTextW(strTemp);
			//	m_sDGPSData.dAltitude=dTemp;
				memset(GPSDataTemp,0,15);
				break;
			case 11://高度单位m
				memset(GPSDataTemp,0,15);
				break;
			case 12://空
				memset(GPSDataTemp,0,15);
				break;
			case 13://空
				memset(GPSDataTemp,0,15);
				break;
			case 14:  //DGPS接收的期限，单位s
				dTemp=atoi(GPSDataTemp);
//				m_sDGPSData.age=dTemp;
				memset(GPSDataTemp,0,15);
				break;

			default: break;
			}
		}
		else
		{
			if(GPSCommaNumber <=15 )
				GPSDataTemp[j++] = GPSData[i];
		}
	}
	// save data in file (GPGGA.txt)
	//{
	//	FILE *fp;

	//	fp  = fopen( "GPGGA.txt","a+");//打开文件
	//	if( fp )
	//	{
	//		fprintf(fp,"%s\n",GPSData);
	//		fclose(fp);

	//	}
	//	else
	//	{
	//		AfxMessageBox(_T("The file 'GPGGA.txt' was not opened"));
	//	}
	//}

}



void CAnalysisGPS::GPSDataPASHR(char GPSData[],int n)
{
	double dTemp = 0.0;
	double dTemp1 = 0.0,dTemp2 = 0.0;
	int  unTemp = 0;
	long  lTemp = 0;

	int i = 0;
	int j = 0;
	int GPSCommaNumber = 0;
	char GPSDataTemp[15];

	memset(GPSDataTemp,0,15);

	while(i++ <= n)
	{

		if(GPSData[i] == ',')		//找到逗号
		{
			j = 0;
			GPSCommaNumber++;
			switch(GPSCommaNumber)
			{
			case 1:	// $PASHR
				memset(GPSDataTemp,0,15);
				break;
			case 2: // Course over ground
				/*dTemp = atof(GPSDataTemp);*/
				//strTemp.Format(_T("%.1f"),dTemp);
				//m_ctrlEditCourse.SetWindowTextW(strTemp);
				dTemp = atof(GPSDataTemp);

				//m_sDGPSData.realnorth_heading=dTemp;

			//	GPSData_struct.VTG_heading=dTemp;
				memset(GPSDataTemp,0,15);

				break;
			case 3: // Magnetic course over ground
				dTemp = atof(GPSDataTemp);
//				m_sDGPSData.magnorth_heading=dTemp;
				GPSData_struct.PASHR_heading=dTemp;

				memset(GPSDataTemp,0,15);
				break;
			case 4: // Speed over ground
				//dTemp = atof(GPSDataTemp);
				//strTemp.Format(_T("%.1f"),dTemp);
				//m_ctrlEditSpeed.SetWindowTextW(strTemp);
				memset(GPSDataTemp,0,15);
				break;
			case 5: // Speed over ground
				dTemp = atof(GPSDataTemp);
				//strTemp.Format(_T("%.1f"),dTemp);
				//m_ctrlEditSpeed.SetWindowTextW(strTemp);

				memset(GPSDataTemp,0,15);
				break;
			case 6: // Speed over ground in kilometers per hour
				dTemp = atof(GPSDataTemp);
			//	GPSData_struct.VTG_V=dTemp;//20170802
				memset(GPSDataTemp,0,15);
				break;
			case 7: //
			//	GPSData_struct.DGPSState_VTG=GPSDataTemp[0];
				memset(GPSDataTemp,0,15);
				break;

			default: break;
			}
		}
		else
		{
			if(GPSCommaNumber <=7 )
				GPSDataTemp[j++] = GPSData[i];
		}
	}

}


void CAnalysisGPS::GPSDataGPVTG(char GPSData[],int n)
{

	double dTemp = 0.0;
	double dTemp1 = 0.0,dTemp2 = 0.0;
	int  unTemp = 0;
	long  lTemp = 0;

	int i = 0;
	int j = 0;
	int GPSCommaNumber = 0;
	char GPSDataTemp[15];

	memset(GPSDataTemp,0,15);

	while(i++ <= n)
	{

		if(GPSData[i] == ',')		//找到逗号
		{
			j = 0;
			GPSCommaNumber++;
			switch(GPSCommaNumber)
			{
			case 1:	// $GPVTG
				memset(GPSDataTemp,0,15);
				break;
			case 2: // Course over ground
				/*dTemp = atof(GPSDataTemp);*/
				//strTemp.Format(_T("%.1f"),dTemp);
				//m_ctrlEditCourse.SetWindowTextW(strTemp);
				dTemp = atof(GPSDataTemp);

				//m_sDGPSData.realnorth_heading=dTemp;

				GPSData_struct.VTG_heading=dTemp;
				memset(GPSDataTemp,0,15);

				break;
			case 3: // Magnetic course over ground
				dTemp = atof(GPSDataTemp);
//				m_sDGPSData.magnorth_heading=dTemp;
				memset(GPSDataTemp,0,15);
				break;
			case 4: // Speed over ground
				//dTemp = atof(GPSDataTemp);
				//strTemp.Format(_T("%.1f"),dTemp);
				//m_ctrlEditSpeed.SetWindowTextW(strTemp);
				memset(GPSDataTemp,0,15);
				break;
			case 5: // Speed over ground
				dTemp = atof(GPSDataTemp);
				//strTemp.Format(_T("%.1f"),dTemp);
				//m_ctrlEditSpeed.SetWindowTextW(strTemp);

				memset(GPSDataTemp,0,15);
				break;
			case 6: // Speed over ground in kilometers per hour
				dTemp = atof(GPSDataTemp);
				GPSData_struct.VTG_V=dTemp;//20170802
				memset(GPSDataTemp,0,15);
				break;
			case 7: //
				GPSData_struct.DGPSState_VTG=GPSDataTemp[0];
				memset(GPSDataTemp,0,15);
				break;

			default: break;
			}
		}
		else
		{
			if(GPSCommaNumber <=7 )
				GPSDataTemp[j++] = GPSData[i];
		}
	}
	// save data in file (GPVTG.txt)
	//{
	//	FILE *fp;

	//	fp  = fopen( "GPVTG.txt","a+");//打开文件
	//	if( fp )
	//	{
	//		fprintf(fp,"%s\n",GPSData);
	//		fclose(fp);

	//	}
	//	else
	//	{
	//		AfxMessageBox(_T("The file 'GPVTG.txt' was not opened"));
	//	}
	//}
}


void CAnalysisGPS::GPSDataGPTRA(char GPSData[],int n)
{

	double dTemp = 0.0;
	double dTemp1 = 0.0,dTemp2 = 0.0;
	int  unTemp = 0;
	long  lTemp = 0;

	int i = 0;
	int j = 0;
	int GPSCommaNumber = 0;
	char GPSDataTemp[15];

	memset(GPSDataTemp,0,15);

	while(i++ <= n)
	{

		if(GPSData[i] == ',')		//找到逗号
		{
			j = 0;
			GPSCommaNumber++;
			switch(GPSCommaNumber)
			{
			case 1:	// $GPVTG
				memset(GPSDataTemp,0,15);
				break;
			case 2: // Course over ground
				/*dTemp = atof(GPSDataTemp);*/
				//strTemp.Format(_T("%.1f"),dTemp);
				//m_ctrlEditCourse.SetWindowTextW(strTemp);
				dTemp = atof(GPSDataTemp);


				//m_sDGPSData.realnorth_heading=dTemp;
				//m_sVehicleLeader.fHeadingGPS=m_sDGPSData.realnorth_heading;
				memset(GPSDataTemp,0,15);

				break;
			case 3: // Magnetic course over ground
				dTemp = atof(GPSDataTemp);
				GPSData_struct.PTRA_heading=dTemp;
//				m_sDGPSData.magnorth_heading=dTemp;
				memset(GPSDataTemp,0,15);
				break;
			case 4: // Speed over ground
				//dTemp = atof(GPSDataTemp);
				//strTemp.Format(_T("%.1f"),dTemp);
				//m_ctrlEditSpeed.SetWindowTextW(strTemp);
				memset(GPSDataTemp,0,15);
				break;
			case 5: // Speed over ground
				//dTemp = atof(GPSDataTemp);
				//strTemp.Format(_T("%.1f"),dTemp);
				//m_ctrlEditSpeed.SetWindowTextW(strTemp);
				memset(GPSDataTemp,0,15);
				break;
			case 6: // Speed over ground in kilometers per hour
				dTemp = atof(GPSDataTemp);

				memset(GPSDataTemp,0,15);
				break;
			case 7: //
				memset(GPSDataTemp,0,15);
				break;

			default: break;
			}
		}
		else
		{
			if(GPSCommaNumber <=7 )
				GPSDataTemp[j++] = GPSData[i];
		}
	}
	// save data in file (GPVTG.txt)
	//{
	//	FILE *fp;

	//	fp  = fopen( "GPVTG.txt","a+");//打开文件
	//	if( fp )
	//	{
	//		fprintf(fp,"%s\n",GPSData);
	//		fclose(fp);

	//	}
	//	else
	//	{
	//		AfxMessageBox(_T("The file 'GPVTG.txt' was not opened"));
	//	}
	//}
}





void CAnalysisGPS::GPSDataGPHDT(char GPSData[],int n)
{

	double dTemp = 0.0;
	double dTemp1 = 0.0,dTemp2 = 0.0;
	int  unTemp = 0;
	long  lTemp = 0;

	int i = 0;
	int j = 0;
	int GPSCommaNumber = 0;
	char GPSDataTemp[15];

	memset(GPSDataTemp,0,15);

	while(i++ <= n)
	{

		if(GPSData[i] == ',')		//找到逗号
		{
			j = 0;
			GPSCommaNumber++;
			switch(GPSCommaNumber)
			{
			case 1:	// $GPVTG
				memset(GPSDataTemp,0,15);
				break;
			case 2: // Course over ground
				/*dTemp = atof(GPSDataTemp);*/
				//strTemp.Format(_T("%.1f"),dTemp);
				//m_ctrlEditCourse.SetWindowTextW(strTemp);
				dTemp = atof(GPSDataTemp);
				GPSData_struct.PHDT_heading=dTemp;

				//m_sDGPSData.realnorth_heading=dTemp;
				//m_sVehicleLeader.fHeadingGPS=m_sDGPSData.realnorth_heading;
				memset(GPSDataTemp,0,15);

				break;
			case 3: // Magnetic course over ground
				dTemp = atof(GPSDataTemp);
//				m_sDGPSData.magnorth_heading=dTemp;
				memset(GPSDataTemp,0,15);
				break;
			case 4: // Speed over ground
				//dTemp = atof(GPSDataTemp);
				//strTemp.Format(_T("%.1f"),dTemp);
				//m_ctrlEditSpeed.SetWindowTextW(strTemp);
				memset(GPSDataTemp,0,15);
				break;
			case 5: // Speed over ground
				//dTemp = atof(GPSDataTemp);
				//strTemp.Format(_T("%.1f"),dTemp);
				//m_ctrlEditSpeed.SetWindowTextW(strTemp);
				memset(GPSDataTemp,0,15);
				break;
			case 6: // Speed over ground in kilometers per hour
				dTemp = atof(GPSDataTemp);

				memset(GPSDataTemp,0,15);
				break;
			case 7: //
				memset(GPSDataTemp,0,15);
				break;

			default: break;
			}
		}
		else
		{
			if(GPSCommaNumber <=7 )
				GPSDataTemp[j++] = GPSData[i];
		}
	}
	// save data in file (GPVTG.txt)
	//{
	//	FILE *fp;

	//	fp  = fopen( "GPVTG.txt","a+");//打开文件
	//	if( fp )
	//	{
	//		fprintf(fp,"%s\n",GPSData);
	//		fclose(fp);

	//	}
	//	else
	//	{
	//		AfxMessageBox(_T("The file 'GPVTG.txt' was not opened"));
	//	}
	//}
}





