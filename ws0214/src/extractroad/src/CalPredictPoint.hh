#pragma once
//#include "NOTATION.hh"//�������������ṹ��
//#include "BasicFunc.hh"
#include <vector>
#include "opencv/cv.h"				//张欣 解注释难道不用opencv吗
#include "opencv/cxcore.h"
#include "opencv/highgui.h"
//#include<math.h>//lzz
#include <thread>         // std::thread
#include <mutex>          // std::mutex
#include <ros/ros.h>
//#include "util/boostudp/boostudp.h"
#include "iv_slam_ros_msgs/TraversibleArea.h"
//#include "util/iv_inc.hh"
using namespace std;
#define BOUNDARY_EXTRACT_RANGE 10

#define  VITURA_LIDAR2VEHICLE 2//5   //1026
#define  BOUNDARY_START_POS   8
#define  BACK_VIRTUTA_LIDAR2VEHICLE -3  //1026
#define  ANGLE_SOLUTION       2
#define  START_ANGLE          0
#define  END_ANGLE           180
#define TOTAL_END_ANGLE      359
#define TOTAL_ANGLE_COUNT    180
#define ANGLE_COUNT           91
#define  BACK_START_ANGLE    182
#define  BACK_END_ANGLE      359
#define BIAS_THRESHOLD        0.6
//�����˲���

#define SAMPLENO 2000
#define SAMPLENONUM 1000
#define OGMRESOLUTION	0.2
#define OGMWIDTH_M		40	//OGM横轴长度m
#define OGMHEIGHT_M		80	//OGM纵轴长度m
#define	OGMWIDTH_CELL	cvRound(OGMWIDTH_M / OGMRESOLUTION + 1)
#define	OGMHEIGHT_CELL	cvRound(OGMHEIGHT_M / OGMRESOLUTION + 1)
#define VEHICLEPOSINOGM_Y_M 20//车后轴到OGM底边的距离
#define VEHICLEPOSINOGM_Y_CELL cvRound(VEHICLEPOSINOGM_Y_M/OGMRESOLUTION)
#define VEHICLEPOSINOGM_X_M 20//车后轴在OGM局部坐标系中的横坐标（m）
#define VEHICLEPOSINOGM_X_CELL cvRound(VEHICLEPOSINOGM_X_M/OGMRESOLUTION)
//////////////////////////////////
#define OGM_PASS			0
#define OGM_UNKNOWN			1
#define OGM_POSSIBLENOPASS	2
#define OGM_NOPASS			3
#define pi 3.14159265
#define PI 3.14159265
//#define USEKALMAN
struct Prob_cross 
{
	double angle;
	double weight;
	bool update;
};
struct angle_dis
{
	int angle;
	int dis;
};
struct boundary_pose
{
	double a0;
	double a1;
	double a2;
	double weight;
};
struct RoadBoundary_OGM_Data{
	int height;
	int width;
	unsigned char data[100000];
	double lTimeStamp;
};//lzz
struct Cross_Road_Param{
	float l_a0;
	float l_a1;
	float l_a2;
	float r_a0;
	float r_a1;
	float r_a2;
	float c_a0;
	float c_a1;
	float c_a2;

};
struct _OGMDataStruct			//zx
{
	unsigned char* m_OccupiedMap;
	ros::Time lTimeStamp;
};
struct pos					//zx
{
	double x;
	double y;
};
struct pos_int
{
	int x;
	int y;
};
struct state_struct				//zx
{
	pos position;
	double s;//弧长
	double theta;//弧度，车头与正东方向夹角，heading=pi/2-theta; heading_g=heading_l+heading_v
	bool forward;//前进或者后退

	double steering_angle;
	double radius;

	//来自路网的属性
	int index;
	char type;
	double roadwidth;
	double lanewidth;
	int lanecount;
	int lanecount_samedirection;
	int laneside;
	double maxvel;
	int RoadCharactetics;//道路特征.qjy add 20171113

	char side;
	pos mapmatch_pos;//qjy,地图匹配路网 20171104

	pos gps_pos;//gps路网20171104


};
struct Cross_Intf //lzz,20170630，齐建永参照更改。//zx
{
	int Orientation;
	float left_param0;
	float left_param1;
	float left_param2;
	float right_param0;
	float right_param1;
	float right_param2;
	float center_param0;
	float center_param1;
	float center_param2;
	bool Boundary_Valid;
	int Valid_Boundary_Num;
};
struct MSG{
	ros::Time stamp;
	int width;
	int height;
	int triD_submap_pose_image_index_x;
	int triD_submap_pose_image_index_y;
	unsigned char* cells;
};
//struct Cross_Intf //lzz
//{
//	int Orientation;
//	float left_param0;
//	float left_param1;
//	float left_param2;
//	float right_param0;
//	float right_param1;
//	float right_param2;
//	float center_param0;
//	float center_param1;
//	float center_param2;
//};
typedef unsigned char UCHAR;
class CalPredictPoint
{
public:
	CalPredictPoint();//ros::NodeHandle& nh
	~CalPredictPoint(void);
	void setup(MSG &msg);
	void ExtratCenterPoint(_OGMDataStruct* OGMData_ptr );//(const iv_slam_ros_msgs::TraversibleArea& msg
	void cal_predic_point(int virtual_lidar_pose_y,int angle,int range,pos* point);  
	void Extract_cross_road(int Virtual_x,int Virtual_y,int distance_thresold_cell);
	void Extract_cross_road_second(int Virtual_x,int Virtual_y,int distance_thresold_cell);
	void Extract_back_road(int distance_thresold_cell,int* back_road_angle);
	void Show_result(_OGMDataStruct* OGMData_ptr,int Virtual_x,int Virtual_y,vector<int> out_vec,int chosen_angle,state_struct vehicle_state_GPS);
	int Cal_dist_with_angle(int angle,int Virtual_x,int Virtual_y);
	void new_Cal_dist_with_angle();   
	int last_angle;
	int miss_count;
	int hit_count;
	IplConvKernel *element_erode, *element_dilate;//�������� 
	_OGMDataStruct OGMData_Cross;
	vector<int> out_vec;
	vector<int> back_out_vec;//lzz
	angle_dis* total_dis_vec;
	angle_dis* dis_vec;
	int imagewidth, imageheight;
	vector<Prob_cross> prob_cross_pos;//��Ҷ˹���ʸ��º��ǰ��·��
	int forward_road_angle; //����ǰ�����ڵ�·����
	int back_road_angle;
	void Clear_forward_road(_OGMDataStruct* OGMData_ptr,_OGMDataStruct* VelodyneRigidOGMGridData,int road_center_angle,float width,float height);
	//void Draw_road_boundary(_OGMDataStruct* OGMData_ptr,int road_head,int back_road_head,vector<pos>* pos_predict);
//	void Draw_road_boundary(_OGMDataStruct* OGMData_ptr,double front_road_head,int back_road_head,CvKalman* kalman);
	void Draw_road_boundary(_OGMDataStruct* OGMData_ptr,double front_road_head,int back_road_head,CvKalman* kalman,Cross_Road_Param *Cross);
	void Draw_road_boundary_point(_OGMDataStruct* OGMData_ptr,double cross_angle_,int back_road_head,float path_length,vector<pos>* left_boundary_point,vector<pos>* right_boundary_point);
	float Cal_road_boudary_RANSAC(int iteration_num,float threshold_scale,vector<pos>* boundary_poinnt,int sample_num,float bias_threshold,float* a0,float* a1,float* a2,int boud_style);
	float Cal_road_boudary_LSM(vector<pos>* boundary_poinnt,float* a0,float* a1,float* a2,int boud_style);
	void Show_boundary(_OGMDataStruct* OGMData_ptr,vector<pos>left_boundary_point,vector<pos>right_boundary_point,float path_length,int tempary_angle,Cross_Road_Param *Cross_Show);
	void Cal_weight_boundary_point(vector<pos>* boundary_point,float a0,float a1,float a2,float bias_threshold,bool boundary_valid);
	bool PF_update_boundary(vector<pos>* boundary_point,float* a0,float* a1,float* a2,int boud_style);
	float NormalDistribute();
	double unifytheta(double theta)		//zx
	{
		while(theta<0)
			theta=theta+2*PI;
		while(theta>=2*PI)
			theta=theta-2*PI;

		return theta;
	}
	unsigned char PosObsCost(pos_int pos, _OGMDataStruct* OGMData_ptr)  //zx
	{
		//return 0;
		if(pos.x >= ogmwidth_cell_ || pos.x< 0 || pos.y>=ogmheight_cell_ || pos.y<0)
		{
			return 0;
		}
		return (*(OGMData_ptr->m_OccupiedMap+pos.y*ogmwidth_cell_+pos.x));//·�ػ��߼���㣬��ogm����ڽ���
	}
	void Convert_World_to_OGM(double x, double y, int * x_ogm, int * y_ogm, double ref_x, double ref_y, double resolution)//zx
	{//m to cell
		*x_ogm = cvRound((ref_x + x)/resolution);
		*y_ogm = cvRound((ref_y + y)/resolution);
	}

	void Convert_OGM_to_World(int x, int y , double* x_world, double* y_world, double ref_x, double ref_y, double resolution)
	{//cell to m
		*x_world = x * resolution - ref_x;
		*y_world = y * resolution - ref_y;
	}
	bool left_boundary_vaild;
	bool right_boundary_valid;
	bool left_boundary_trackced;
	bool right_boundary_tracked;


	boundary_pose noisepos[SAMPLENONUM];
	boundary_pose left_bound_pos[SAMPLENONUM];
	boundary_pose right_bound_pos[SAMPLENONUM];
	bool left_filter_initilized;
	bool right_filter_initilized;
	//Kalman
	CvKalman* kalman0;
	CvKalman* kalman1;
	CvKalman* kalman2;
	CvKalman* kalman3;
	CvKalman* kalman4;
	CvKalman* kalman5;

	bool kalman_inited;
	IplImage* show_img;
	state_struct vehicle_state_GPS;
	Cross_Road_Param Cross0,Cross1,Cross2,Cross3,Cross4,Cross5;
	
	int Drawn_Cross_Num;//lzz
	struct Cross_Intf Detected_Cross[6];
	pos back_road_center_point;
	vector<Prob_cross> Cross_got;
	vector<double> Road_Orientation_Obtained;
private:
	int ogmwidth_cell_;
	int ogmheight_cell_;
	int vehicle_x_;
	int vehicle_y_;
//	ros::NodeHandle nh_;
//	ros::Subscriber sub_;

};
