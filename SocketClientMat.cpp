#include "SocketMatTransmissionClient.h"
using namespace std;
int main()
{
	SocketMatTransmissionClient socketMat;
	if (socketMat.socketConnect("192.168.199.204", 6666) < 0)
	{
		return 0;
	}
	
	cv::VideoCapture capture(0);
	cv::Mat image;
 
	while (1)
	{
		if (!capture.isOpened())
			return 0;

		capture >> image;

		if (image.empty())
			return 0;
		cout<<"sldkffs;k"<<endl;
		std::vector<uchar> data_encode;
		std::vector<int> quality;
		quality.push_back(CV_IMWRITE_JPEG_QUALITY);
		quality.push_back(50);//进行50%的压缩
		imencode(".jpg", image, data_encode,quality);//将图像编码
		cout<<data_encode.size()<<"   lll"<<endl;
		Mat imageshow = imdecode(data_encode, CV_LOAD_IMAGE_COLOR);
		imshow("test",imageshow);
		socketMat.transmit(data_encode);
		cv::waitKey(30);
	}

//	socketMat.socketDisconnect();
	return 0;
}

