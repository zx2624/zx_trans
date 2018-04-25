#include "CalPredictPoint.hh"
CalPredictPoint::CalPredictPoint() {//:nh_(nh)
//	sub_=nh_.subscribe<iv_slam_ros_msgs::TraversibleArea>
//			("traversible_area_topic",1,boost::bind((CalPredictPoint::ExtratCenterPoint),this,_1));

	last_angle = 90;
	miss_count = 0;
	hit_count = 0;
//	Cross0 = new Cross_Road_Param;
//	memset(Cross0,0,9*sizeof(float));
//
//	Cross1 = new Cross_Road_Param;
//	memset(Cross1,0,9*sizeof(float));
//
//	Cross2 = new Cross_Road_Param;
//	memset(Cross2,0,9*sizeof(float));
//	Cross3 = new Cross_Road_Param;
//	memset(Cross3,0,9*sizeof(float));
//	Cross4 = new Cross_Road_Param;
//	memset(Cross4,0,9*sizeof(float));
//	Cross5 = new Cross_Road_Param;
//	memset(Cross5,0,9*sizeof(float));

			Cross0.l_a0 = -2;
			Cross0.l_a1 = 0;
			Cross0.l_a2 = 0;
			Cross0.r_a0 = 2;
			Cross0.r_a1 = 0;
			Cross0.r_a2 = 0;
			Cross0.c_a0 = 0;
			Cross0.c_a1 = 0;
			Cross0.c_a2 = 0;
			Cross1.l_a0 = -2;
			Cross1.l_a1 = 0;
			Cross1.l_a2 = 0;
			Cross1.r_a0 = 2;
			Cross1.r_a1 = 0;
			Cross1.r_a2 = 0;
			Cross1.c_a0 = 0;
			Cross1.c_a1 = 0;
			Cross1.c_a2 = 0;

			Cross2.l_a0 = -2;
			Cross2.l_a1 = 0;
			Cross2.l_a2 = 0;
			Cross2.r_a0 = 2;
			Cross2.r_a1 = 0;
			Cross2.r_a2 = 0;
			Cross2.c_a0 = 0;
			Cross2.c_a1 = 0;
			Cross2.c_a2 = 0;

			Cross3.l_a0 = -2;
			Cross3.l_a1 = 0;
			Cross3.l_a2 = 0;
			Cross3.r_a0 = 2;
			Cross3.r_a1 = 0;
			Cross3.r_a2 = 0;
			Cross3.c_a0 = 0;
			Cross3.c_a1 = 0;
			Cross3.c_a2 = 0;

			Cross4.l_a0 = -2;
			Cross4.l_a1 = 0;
			Cross4.l_a2 = 0;
			Cross4.r_a0 = 2;
			Cross4.r_a1 = 0;
			Cross4.r_a2 = 0;
			Cross4.c_a0 = 0;
			Cross4.c_a1 = 0;
			Cross4.c_a2 = 0;

			Cross5.l_a0 = -2;
			Cross5.l_a1 = 0;
			Cross5.l_a2 = 0;
			Cross5.r_a0 = 2;
			Cross5.r_a1 = 0;
			Cross5.r_a2 = 0;
			Cross5.c_a0 = 0;
			Cross5.c_a1 = 0;
			Cross5.c_a2 = 0;

	back_road_angle = 90;

	forward_road_angle = 270;
	Drawn_Cross_Num = 0;
	element_erode = cvCreateStructuringElementEx(1 * 1 + 1, 1 * 1 + 1, 1, 1,
			CV_SHAPE_ELLIPSE, 0);
	element_dilate = cvCreateStructuringElementEx(2 * 2 + 1, 2 * 2 + 1, 2, 2,
			CV_SHAPE_ELLIPSE, 0);
//	OGMData_Cross.m_OccupiedMap = new UCHAR[OGMWIDTH_CELL * OGMHEIGHT_CELL];
//	memset(OGMData_Cross.m_OccupiedMap, 0, OGMWIDTH_CELL * OGMHEIGHT_CELL);

//	if (OGMWIDTH_CELL % 8)
//		imagewidth = (OGMWIDTH_CELL / 8 + 1) * 8;
//	else
//		imagewidth = (OGMWIDTH_CELL / 8) * 8;
//	imageheight = OGMHEIGHT_CELL;

	dis_vec = new angle_dis[TOTAL_ANGLE_COUNT];
	total_dis_vec = new angle_dis[TOTAL_ANGLE_COUNT];
	left_boundary_vaild = false;
	right_boundary_valid = false;
	left_boundary_trackced = false;
	right_boundary_tracked = false;
//	cout<<"lskdjf;slkdfjs;ldkjf"<<endl;
	memset(noisepos, 0, sizeof(boundary_pose) * SAMPLENONUM);
	memset(left_bound_pos, 0, sizeof(boundary_pose) * SAMPLENONUM);
	memset(right_bound_pos, 0, sizeof(boundary_pose) * SAMPLENONUM);
	left_filter_initilized = true;
	right_filter_initilized = true;
//	show_img = cvCreateImage(cvSize(cvRound(imagewidth), cvRound(imageheight)),
//			8, 3);
//	cvZero(show_img);

	kalman_inited = false;
	kalman0 = cvCreateKalman(9, 9, 0);
	kalman1 = cvCreateKalman(9, 9, 0);
	kalman2 = cvCreateKalman(9, 9, 0);
	kalman3 = cvCreateKalman(9, 9, 0);
	kalman4 = cvCreateKalman(9, 9, 0);
	kalman5 = cvCreateKalman(9, 9, 0);


}

CalPredictPoint::~CalPredictPoint(void) {
	cvReleaseStructuringElement(&element_erode);
	cvReleaseStructuringElement(&element_dilate);
	cvReleaseKalman(&kalman0);
	cvReleaseKalman(&kalman1);
	cvReleaseKalman(&kalman2);
	cvReleaseKalman(&kalman3);
	cvReleaseKalman(&kalman4);
	cvReleaseKalman(&kalman5);
	delete[] dis_vec;
	delete[] total_dis_vec;
//	delete Cross0;
//	delete Cross1;
//	delete Cross2;
//	delete Cross3;
//	delete Cross4;
//	delete Cross5;
}
void CalPredictPoint::setup(MSG &msg){
	ogmwidth_cell_=msg.width;
	ogmheight_cell_=msg.height;
	vehicle_x_=msg.triD_submap_pose_image_index_x;
	vehicle_y_=msg.triD_submap_pose_image_index_y;
	OGMData_Cross.m_OccupiedMap = new UCHAR[ogmwidth_cell_ * ogmheight_cell_];
	if (ogmwidth_cell_ % 8)
		imagewidth = (ogmwidth_cell_ / 8 + 1) * 8;
	else
		imagewidth = (ogmwidth_cell_ / 8) * 8;
	imageheight = ogmwidth_cell_;
	show_img = cvCreateImage(cvSize(cvRound(imagewidth), cvRound(imageheight)),
			8, 3);
	cvZero(show_img);

	unsigned char data[msg.width*msg.height];
	_OGMDataStruct* OGMData_ptr=new _OGMDataStruct;
	OGMData_ptr->lTimeStamp=msg.stamp;
//	int i=0;
//	for(;i<msg.cells.size();i++){
//		data[i]=msg.cells[i];
//}
	OGMData_ptr->m_OccupiedMap=msg.cells;
	char* windowname="testwindow";
	cvNamedWindow(windowname,0);
	IplImage *slopemat = cvCreateImage(cvSize(msg.width,msg.height),IPL_DEPTH_8U,3);
	for(int i=0;i<msg.height;i++){
		unsigned char* pdata = (unsigned char*)(slopemat->imageData + (msg.height-i-1)* slopemat->widthStep);
		for(int j=0;j<msg.width;j++){
		unsigned char val = OGMData_ptr->m_OccupiedMap[j + i*msg.width];
		if(val==0){
//			cout<<"000000000";
			pdata[3*j]=0;
			pdata[3*j+1]=0;
			pdata[3*j+2]=0;
		}
		else if(val==2){
			pdata[3*j]=255;
			pdata[3*j+1]=255;
			pdata[3*j+2]=255;
		}
		else{
			pdata[3*j]=0;
			pdata[3*j+1]=255;
			pdata[3*j+2]=0;
		}
		}
	}
	int height=msg.height-msg.triD_submap_pose_image_index_y,width=msg.triD_submap_pose_image_index_x;
	cvDilate(slopemat, slopemat, element_dilate, 1);
	cvCircle(slopemat,cvPoint(width,height),5,CvScalar(125,125,0),10);
	cvCircle(slopemat,cvPoint(width,height+10),5,CvScalar(125,125,125),10);
    cvShowImage(windowname,slopemat);
    cvWaitKey(10);
    cvReleaseImage(&slopemat);
	ExtratCenterPoint(OGMData_ptr);
}
//void CalPredictPoint::ExtratCenterPoint(_OGMDataStruct* OGMData_ptr,vector<pos>* pos_predict,Cross_state* cross_pos,pos* back_road_center_point)
void CalPredictPoint::ExtratCenterPoint(_OGMDataStruct* OGMData_ptr ){ //const iv_slam_ros_msgs::TraversibleArea& msg
//	_OGMDataStruct* OGMData_ptr;
//	OGMData_ptr->lTimeStamp=msg.header.stamp;
//	for(int i=0;i<msg.cells.size();i++){
//		OGMData_ptr->m_OccupiedMap[i]=msg.cells[i];
//	}
	IplImage *OGMData_Image = cvCreateImage(
			cvSize(cvRound(imagewidth), cvRound(imageheight)), 8, 1);

	cvZero(OGMData_Image);
	for (int i = 0; i < imageheight; i++)
		memcpy(OGMData_Image->imageData + OGMData_Image->widthStep * i,
				(char*) OGMData_ptr->m_OccupiedMap + ogmwidth_cell_ * i,
				ogmwidth_cell_);
	//cvErode(OGMData_Image,OGMData_Image,element_erode,1);
	cvDilate(OGMData_Image, OGMData_Image, element_dilate, 1);
//	cvShowImage("dialateimg",OGMData_Image);
	for (int i = 0; i < imageheight; i++)
		memcpy(OGMData_Cross.m_OccupiedMap + i * ogmwidth_cell_,			//这里是每行每行拷贝的
				(UCHAR*) OGMData_Image->imageData
						+ OGMData_Image->widthStep * i, ogmwidth_cell_);
	cvReleaseImage(&OGMData_Image);
	static pos_int result_pos_ogm;

	int Virtual_x = (ogmwidth_cell_ - 1) / 2; //雷达位置
	int Virtual_y = (int) (vehicle_y_ + VITURA_LIDAR2VEHICLE)
			/ OGMRESOLUTION;
//total_dis_vec[index]保存极坐标系中每个角度的最远障碍物距离，dis_vec[i]保存保存极坐标系中每个角度的最近障碍物距离
	new_Cal_dist_with_angle();
	Extract_cross_road(Virtual_x, Virtual_y, 100); //100为100个删格，即二十米外的才可被选为路口
	back_road_angle = 270;
//	Extract_back_road(100, &back_road_angle); //车后先不考虑
	back_road_center_point.x = 0;
	back_road_center_point.y = 0;
	cal_predic_point(BACK_VIRTUTA_LIDAR2VEHICLE, back_road_angle, 7,
			&back_road_center_point); //计算车后点在哪
	cout<<"outvec size is "<<out_vec.size()<<endl;
//贝叶斯概率更新
	static bool prob_valid = false;
//	prob_valid=false;
	Prob_cross single_road;
	prob_cross_pos.clear(); //lzz
	for (int i = 0; i < prob_cross_pos.size(); i++) {
		prob_cross_pos[i].update = false;
	}
	for (int i = 0; i < out_vec.size(); i++) {
		bool match_failed = true;
		for (int j = 0; j < prob_cross_pos.size(); j++) {
			double out_vec_angle = out_vec[i] * pi / 180; //lzz
			out_vec_angle = unifytheta(out_vec_angle);
			double delta_angle = fabs(
					(double) (out_vec_angle - prob_cross_pos[j].angle));
			if (delta_angle < 10 * pi / 180) {
				prob_cross_pos[j].angle = (prob_cross_pos[j].angle
						+ out_vec_angle) / 2;
				prob_cross_pos[j].update = true;
				double S = 0;
				double prob_m_z = 0.7;
				S = prob_m_z / (1 - prob_m_z) * prob_cross_pos[j].weight
						/ (1 - prob_cross_pos[j].weight);
				prob_cross_pos[j].weight = S / (1 + S);
				if (prob_cross_pos[j].weight > 1.0 - 0.02) {
					prob_cross_pos[j].weight = 1.0 - 0.02;
				}
				match_failed = false;
				break;
			}
		}
		if (match_failed) {
			double out_vec_angle = out_vec[i] * pi / 180; //lzz
			out_vec_angle = unifytheta(out_vec_angle);
			single_road.angle = out_vec_angle;
			single_road.weight = 0.7;
			single_road.update = true;
			prob_cross_pos.push_back(single_road);
		}
	}

	for (int i = 0; i < prob_cross_pos.size(); i++) {
		if (!prob_cross_pos[i].update) {
			double S = 0;
			double prob_m_z = 0.3;
			S = prob_m_z / (1 - prob_m_z) * prob_cross_pos[i].weight
					/ (1 - prob_cross_pos[i].weight);
			prob_cross_pos[i].weight = S / (1 + S);
			if (prob_cross_pos[i].weight < 0.02) {
				prob_cross_pos[i].weight = 0.02;
			}
		}
	}

	Cross_got.clear();
	for (int i = 0; i < prob_cross_pos.size(); i++) {
		if (prob_cross_pos[i].weight > 0.9) {
			prob_valid = true;
			Cross_got.push_back(prob_cross_pos[i]);
		} else if (prob_cross_pos[i].weight < 0.5) {
			prob_cross_pos.erase(prob_cross_pos.begin() + i);
			i--;
		}

	}

	bool re_extract = 0;
	if (out_vec.size() == 0 || Cross_got.size() == 0)
		re_extract = 1;
	if (re_extract) {
		Extract_cross_road_second(Virtual_x, Virtual_y, 100);
	}
//如果存在贝叶斯概率更新后的路口，则进行路口匹配
	if (prob_valid) {
		for (int i = 0; i < out_vec.size(); i++) {
			bool mached = false;
			for (int j = 0; j < Cross_got.size(); j++) {
				if (Cross_got[j].weight > 0.9) {
					int prob_angle = unifytheta(Cross_got[j].angle) * 180 / pi;
					if (prob_angle - out_vec[i] > -15
							&& prob_angle - out_vec[i] < 15) {
						mached = true;
						break;
					}
				}
			}
//			if (!mached)
//			{
//				out_vec.erase(out_vec.begin()+i);
//				i--;
//			}
		}

	}

	if (out_vec.size() > 0) {
	} else {

		int dis_max = -10;
		int Chosen_lane = 0;
		for (int i = 0; i < ANGLE_COUNT; i++) {
			if (dis_vec[i].dis > dis_max) {
				Chosen_lane = dis_vec[i].angle;
				dis_max = dis_vec[i].dis;
			}
		}

		Prob_cross single_road;
		single_road.update = true;
		single_road.weight = 0.7;
		single_road.angle = Chosen_lane;
		Cross_got.push_back(single_road);
	}
	Road_Orientation_Obtained.clear();
	for (int i = 0; i < Cross_got.size(); i++) {
		Road_Orientation_Obtained.push_back(Cross_got[i].angle);
	}
	cout<<"Road_Orientation_Obtained size is "<<Road_Orientation_Obtained.size()<<endl;
	Drawn_Cross_Num = 0;
	memset(Detected_Cross, 0, sizeof(struct Cross_Intf));
//	cout<< "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<< endl;

}

void CalPredictPoint::cal_predic_point(int virtual_lidar_pose_y, int angle,
		int range, pos* point) {
	if (angle != 90 && angle != 270) {
		double offset = (range / sqrt(1 + pow(tan(angle * CV_PI / 180), 2)));
		if (angle < 90 || angle > 270) {
			point->x = offset;
			double ring_offset = sqrt(
					(double) (range * range - (point->x) * (point->x)));
			if (angle < 90)
				point->y = ring_offset;
			else
				point->y = -ring_offset;
		}
		if (angle > 90 && angle < 270) {
			point->x = -offset;
			double ring_offset = sqrt(
					(double) (range * range - (point->x) * (point->x)));
			if (angle < 180)
				point->y = ring_offset;
			else
				point->y = -ring_offset;
		}
	}
	if (angle == 90) {
		point->x = 0;
		point->y = range;
	}
	if (angle == 270) {
		point->x = 0;
		point->y = -range;
	}
	point->y += virtual_lidar_pose_y;
}

void CalPredictPoint::Extract_cross_road(int Virtual_x, int Virtual_y,
		int distance_thresold_cell) {
	vector<int> vec;
	vec.clear();

	int Lane_count = 0;
	int vertica_ogm_count = 0;

//	for (int i=START_ANGLE;i<=END_ANGLE;i=i+ANGLE_SOLUTION)
	//统计车辆前方障碍物距离超过20米的方向
	for (int i = START_ANGLE; i <= END_ANGLE; i++) {
		int index = i / ANGLE_SOLUTION;
		int ogm_count = dis_vec[index].dis;
		if (ogm_count >= distance_thresold_cell)
			vec.push_back(i);	//将角度值依次压入向量中
	}

	//�ж��м���·��
	int num = vec.size();
	static int count_ = 1;
	out_vec.clear();	//保存每一方向束中最接近90度的方向

	if (num != 0) {
		int init_angle = vec.at(0);
		vector<int> temp_vec;	//保存方向束角度
		vector<int> num_vec;	//保存每一方向束中的方向数量
		temp_vec.clear();
		num_vec.clear();
		temp_vec.push_back(init_angle);

		if (num == 1) {
			out_vec.push_back(init_angle);
			num_vec.push_back(1);
		} else/*lzz*/{
			for (int i = 1; i < num; i++) {
				int angle = vec.at(i);
				if (angle - init_angle > 2) //如果角度不连续，进行处理
						{
					int temp_num = temp_vec.size();
					if (temp_num == 1) //如果没有相邻方向，直接放入到temp_vec中
							{
						temp_vec.clear();
					}

					else {
						int temp_angle = 270;
						for (int j = 0; j < temp_num; j++) {
							double a = fabs((double) (temp_angle - 90));
							double b = fabs((double) (temp_vec.at(j) - 90));
							if (a > 180)
								a = 360 - a;
							if (b > 180)
								b = 360 - b;
							if (a > b)
								temp_angle = temp_vec.at(j);
						}
						out_vec.push_back(temp_angle);

						num_vec.push_back(temp_num);
						temp_vec.clear();

					}
					init_angle = angle;
					temp_vec.push_back(init_angle);

				} else {
					temp_vec.push_back(angle); //如果角度连续，则压入temp_vec
					init_angle = angle;
				}
				if (i == num - 1) //处理最后一个方向，避免直接未处理直接跳出循环
						{
					int temp_num = temp_vec.size();

					if (temp_num > 1) {
						int temp_angle = 270;
						for (int j = 0; j < temp_num; j++) {
							double a = fabs((double) (temp_angle - 90));
							double b = fabs((double) (temp_vec.at(j) - 90));
							if (a > 180)
								a = 360 - a;
							if (b > 180)
								b = 360 - b;
							if (a > b)
								temp_angle = temp_vec.at(j);
						}
						out_vec.push_back(temp_angle);

						num_vec.push_back(temp_num);
					}
				}
			}
		}

//			if (out_vec.size()>1)
//		{
//			for (int i=0;i<out_vec.size()-1;i++)
//			{
//
//				if (out_vec[i+1]-out_vec[i]>45)
//				{
//					continue;
//				}
//				else
//				{
//					if (num_vec[i+1]>num_vec[i])
//					{
//						out_vec.erase(out_vec.begin()+i);
//						num_vec.erase(num_vec.begin()+i);
//						i--;
//					}
//					else if(num_vec[i+1]<num_vec[i])
//					{
//
//						out_vec.erase(out_vec.begin()+i+1);
//						num_vec.erase(num_vec.begin()+i+1);
//					}
//					else
//					{
//						out_vec.at(i)=(int)(out_vec.at(i)+out_vec.at(i+1))/2;
//						out_vec.erase(out_vec.begin()+i+1);
//						num_vec.erase(num_vec.begin()+i+1);
//					}
//				}
//			}
//		}
	}
}

void CalPredictPoint::Extract_cross_road_second(int Virtual_x, int Virtual_y,
		int distance_thresold_cell) {
	vector<int> vec;
	vec.clear();

	int Lane_count = 0;
	int vertica_ogm_count = 0;

//	for (int i=START_ANGLE;i<=END_ANGLE;i=i+ANGLE_SOLUTION)
	//统计车辆前方障碍物距离超过20米的方向
	for (int i = START_ANGLE; i <= END_ANGLE; i++) {
		int index = i / ANGLE_SOLUTION;
		int ogm_count = dis_vec[index].dis;
		if (ogm_count >= distance_thresold_cell)
			vec.push_back(i);	//将角度值依次压入向量中
	}

	//�ж��м���·��
	int num = vec.size();
	static int count_ = 1;
	out_vec.clear();	//保存每一方向束中最接近90度的方向

	if (num != 0) {
		int init_angle = vec.at(0);
		vector<int> temp_vec;	//保存方向束角度
		vector<int> num_vec;	//保存每一方向束中的方向数量
		temp_vec.clear();
		num_vec.clear();
		temp_vec.push_back(init_angle);

		if (num == 1) {
			out_vec.push_back(init_angle);
			num_vec.push_back(1);
		} else/*lzz*/{
			for (int i = 1; i < num; i++) {
				int angle = vec.at(i);
				if (angle - init_angle > 2) //如果角度不连续，进行处理
						{
					int temp_num = temp_vec.size();
					if (temp_num == 1) //如果没有相邻方向，直接放入到temp_vec中
							{
						temp_vec.clear();
					}

					else {
						int temp_angle = 270;
						for (int j = 0; j < temp_num; j++) {
							double a = fabs((double) (temp_angle - 90));
							double b = fabs((double) (temp_vec.at(j) - 90));
							if (a > 180)
								a = 360 - a;
							if (b > 180)
								b = 360 - b;
							if (a > b)
								temp_angle = temp_vec.at(j);
						}
						out_vec.push_back(temp_angle);

						num_vec.push_back(temp_num);
						temp_vec.clear();

					}
					init_angle = angle;
					temp_vec.push_back(init_angle);

				} else {
					temp_vec.push_back(angle); //如果角度连续，则压入temp_vec
					init_angle = angle;
				}
				if (i == num - 1) //处理最后一个方向，避免直接未处理直接跳出循环
						{
					int temp_num = temp_vec.size();

					if (temp_num > 1) {
						int temp_angle = 270;
						for (int j = 0; j < temp_num; j++) {
							double a = fabs((double) (temp_angle - 90));
							double b = fabs((double) (temp_vec.at(j) - 90));
							if (a > 180)
								a = 360 - a;
							if (b > 180)
								b = 360 - b;
							if (a > b)
								temp_angle = temp_vec.at(j);
						}
						out_vec.push_back(temp_angle);

						num_vec.push_back(temp_num);
					}
				}
			}
		}

		if (out_vec.size() > 1) {
			for (int i = 0; i < out_vec.size() - 1; i++) {

				if (out_vec[i + 1] - out_vec[i] > 45) {
					continue;
				} else {
					if (num_vec[i + 1] > num_vec[i]) {
						out_vec.erase(out_vec.begin() + i);
						num_vec.erase(num_vec.begin() + i);
						i--;
					} else if (num_vec[i + 1] < num_vec[i]) {

						out_vec.erase(out_vec.begin() + i + 1);
						num_vec.erase(num_vec.begin() + i + 1);
					} else {
						out_vec.at(i) =
								(int) (out_vec.at(i) + out_vec.at(i + 1)) / 2;
						out_vec.erase(out_vec.begin() + i + 1);
						num_vec.erase(num_vec.begin() + i + 1);
					}
				}
			}
		}

		for (int i = 0; i < out_vec.size(); i++) {
			Prob_cross single_road;
			single_road.update = true;
			single_road.weight = 0.7;
			single_road.angle = out_vec.at(i);
			Cross_got.push_back(single_road);
		}
	}
}

//��ȡ���󷽵�·����
void CalPredictPoint::Extract_back_road(int distance_thresold_cell,
		int* back_road_angle) {
	vector<int> vec;
	//vector<int> back_out_vec;lzz
	vec.clear();
	//dis_vec.clear();
	int Lane_count = 0;
	int vertica_ogm_count = 0;

	for (int i = BACK_START_ANGLE; i <= BACK_END_ANGLE; i = i + ANGLE_SOLUTION)
	{
		int index = i / ANGLE_SOLUTION;
		int ogm_count = dis_vec[index].dis;
		if (ogm_count >= distance_thresold_cell)
			vec.push_back(i);
	}

	//�ж��м���·��
	int num = vec.size();
	static int count_ = 1;
	back_out_vec.clear();

	if (num != 0) {
		int init_angle = vec.at(0);
		vector<int> temp_vec;
		vector<int> num_vec;
		temp_vec.clear();
		num_vec.clear();
		temp_vec.push_back(init_angle);

		if (num == 1) {
			back_out_vec.push_back(init_angle);
			num_vec.push_back(1);
		}

		for (int i = 1; i < num; i++) {
			int angle = vec.at(i);
			if (angle - init_angle > 2) //�������߲�������Ϊ�м����ϰ������ͨ��
					{
				int temp_num = temp_vec.size();
				if (temp_num == 1) {
					back_out_vec.push_back(init_angle); //���������С�ڵ���1�������ǣ������ʲô���⣿
					num_vec.push_back(temp_num);
					temp_vec.clear();
				}

				else {
					//ȡƽ��ֵ�ᵼ�·���ƫ����
					int mean_angle = 0;

					for (int j = 0; j < temp_num; j++) {
						mean_angle += temp_vec.at(j);
					}
					back_out_vec.push_back((int) (mean_angle / temp_num));
					num_vec.push_back(temp_num);
					temp_vec.clear();

				}
				init_angle = angle;
				temp_vec.push_back(init_angle);

			} else {
				temp_vec.push_back(angle);
				init_angle = angle;
			}
			if (i == num - 1) {
				int temp_num = temp_vec.size();
				if (1)
				//if(temp_num>1)
				{
					int mean_angle = 0;
					for (int j = 0; j < temp_num; j++) {
						mean_angle += temp_vec.at(j);
					}
					back_out_vec.push_back((int) (mean_angle / temp_num));
					num_vec.push_back(temp_num);
				}
			}
		}

		if (back_out_vec.size() > 0) {
			for (int i = 0; i < back_out_vec.size() - 1; i++) {

				if (back_out_vec[i + 1] - back_out_vec[i] > 45) {
					continue;
				} else {	//��·�ڽǶȲ�С��45��Ӧ���޳��ͳ���������С��·��
					if (num_vec[i + 1] > num_vec[i]) {
						back_out_vec.erase(back_out_vec.begin() + i);
						num_vec.erase(num_vec.begin() + i);
						i--;
					} else if (num_vec[i + 1] < num_vec[i]) {

						back_out_vec.erase(back_out_vec.begin() + i + 1);
						num_vec.erase(num_vec.begin() + i + 1);
					} else {
						back_out_vec.at(i) = (int) (back_out_vec.at(i)
								+ back_out_vec.at(i + 1)) / 2;
						back_out_vec.erase(back_out_vec.begin() + i + 1);
						num_vec.erase(num_vec.begin() + i + 1);
					}
				}
			}
			//ѡ���복�����ӽ�ĵ�·����
			if (back_out_vec.size() > 0) {
				*back_road_angle = 90;
				for (int i = 0; i < back_out_vec.size(); i++) {
					double a = fabs((double) (*back_road_angle - 270));
					double b = fabs((double) (back_out_vec.at(i) - 270));
					if (a > 180)
						a = 360 - a;
					if (b > 180)
						b = 360 - b;
					if (a > b)
						*back_road_angle = back_out_vec.at(i);
				}
			} else
				*back_road_angle = 270;
		}
	}
}

//����ÿһ������Ŀ�ͨ��դ�����
int CalPredictPoint::Cal_dist_with_angle(int angle, int Virtual_x,
		int Virtual_y) {
	bool valid = true;
	int ogm_count = 0;

	if (angle < 90 | angle > 270) {
		for (int x = vehicle_y_ / 2; x < OGMWIDTH_CELL - 2; x++) {
			if (1) {
				int y = (int) (tan(angle * CV_PI / 180) * (x - Virtual_x)
						+ Virtual_y);
				if (y >= 0 && y < OGMHEIGHT_CELL - 2) {
					pos_int tmp_pos_int;
					int val = 0;
					double deltax = x - Virtual_x;
					double deltay = y - Virtual_y;

					for (int i = -1; i <= 1; i++) {

						tmp_pos_int.x = x + i;
						tmp_pos_int.y = y;
						val = MAX((int )PosObsCost(tmp_pos_int, &OGMData_Cross),
								val);
					}
					ogm_count = (int) sqrt(deltax * deltax + deltay * deltay);
					if (val > OGM_UNKNOWN)
						break;
				} else
					continue;
			}
		}
	}
	if (angle > 90 && angle < 270) {
		for (int x = OGMWIDTH_CELL / 2; x > 0; x--) {
			if (1) {
				int y = (int) (tan(angle * CV_PI / 180) * (x - Virtual_x)
						+ Virtual_y);
				if (y >= 0 && y < OGMHEIGHT_CELL) {
					pos_int tmp_pos_int;
					int val = 0;
					for (int i = -1; i <= 1; i++) {

						tmp_pos_int.x = x + i;
						tmp_pos_int.y = y;
						val = MAX((int )PosObsCost(tmp_pos_int, &OGMData_Cross),
								val);
					}
					double deltax = x - Virtual_x;
					double deltay = y - Virtual_y;
					ogm_count = (int) sqrt(deltax * deltax + deltay * deltay);
					if (val > OGM_UNKNOWN)
						break;

				}
			} else
				break;
		}
	}
	if (angle == 90) {
		for (int y = Virtual_y; y < OGMHEIGHT_CELL; y++) {
			ogm_count = y - Virtual_y;
			pos_int tmp_pos_int;
			tmp_pos_int.x = Virtual_x;
			tmp_pos_int.y = y;
			int val = 0;
			for (int i = -3; i <= 3; i++) {
				tmp_pos_int.x += i;
				int a = PosObsCost(tmp_pos_int, &OGMData_Cross);
				val = MAX((int )PosObsCost(tmp_pos_int, &OGMData_Cross), val);
			}

			if (val > OGM_UNKNOWN)
				break;
		}
	}
	if (angle == 270) {
		for (int y = Virtual_y; y < 0; y--) {
			pos_int tmp_pos_int;
			tmp_pos_int.x = Virtual_x;
			tmp_pos_int.y = y;
			int val = 0;
			for (int i = -3; i <= 3; i++) {
				tmp_pos_int.x += i;
				val = MAX((int )PosObsCost(tmp_pos_int, &OGMData_Cross), val);
			}
			ogm_count = y - Virtual_y;
			if (val > OGM_UNKNOWN)
				break;
		}
	}

	return MIN(ogm_count, 110);
}

//new ����ÿһ������Ŀ�ͨ��դ�����,��ŵ�dis_vec������
void CalPredictPoint::new_Cal_dist_with_angle() {
	int angle_count = TOTAL_ANGLE_COUNT;
	int * valid_dis_angle = new int[angle_count];
	for (int i = 0; i < angle_count; i++) {
		valid_dis_angle[i] = 200;
	}

	for (int i = 0; i < angle_count; i++) {
		total_dis_vec[i].angle = i * 2;
		total_dis_vec[i].dis = -100;
	}
	for (int i = 0; i < ogmheight_cell_; i++) {
		for (int j = 0; j < ogmwidth_cell_; j++) {
			float x = (j - vehicle_x_) * OGMRESOLUTION;
			float y = i * OGMRESOLUTION - vehicle_y_;
			if (y < VITURA_LIDAR2VEHICLE && y > BACK_VIRTUTA_LIDAR2VEHICLE) {
				continue;
			}
			float angle = pi / 2;
			//如果位于雷达传感器前,计算每一删格所对应的角度值，（相对于x轴，逆时针方向为正）
			if (y >= VITURA_LIDAR2VEHICLE) {
				if (fabs(x) > 0.1) {
					angle = atan((y - VITURA_LIDAR2VEHICLE) / x);
					if (angle < 0)
						angle = angle + pi;
					if (angle == 0 && x < 0)
						angle = angle + pi;
				} else {
					angle = pi / 2;
				}

			}

			//��·�ڼ�⼫���ԭ����ǰ�ƶ�BACK_VIRTUTA_LIDAR2VEHICLE
			else if (y < BACK_VIRTUTA_LIDAR2VEHICLE) {
				if (fabs(x) > 0.01) {
					angle = atan((y - BACK_VIRTUTA_LIDAR2VEHICLE) / x);
					if (angle < 0)
						angle = angle + 2 * pi;
					else
						angle = angle + pi;
				} else
					angle = 3 * pi / 2;
			} else
				continue;

			angle = angle * 180 / pi;
			int center_laserindex = cvRound(angle);
			if (center_laserindex < START_ANGLE)
				center_laserindex = START_ANGLE;
			if (center_laserindex > TOTAL_END_ANGLE)
				center_laserindex = TOTAL_END_ANGLE;
			float dis = sqrt(x * x + y * y);
			if (dis > 0.1) {
				float anglerange = OGMRESOLUTION / dis * 180 / pi;

				int upper_laserindex = center_laserindex
						+ cvRound(anglerange / 2);
				if (upper_laserindex >= TOTAL_END_ANGLE)
					upper_laserindex = TOTAL_END_ANGLE;

				int lower_laserindex = center_laserindex
						- cvRound(anglerange / 2);
				if (lower_laserindex < START_ANGLE)
					lower_laserindex = START_ANGLE;

				pos_int tmp_pos_int;
				tmp_pos_int.x = j;
				tmp_pos_int.y = i;
				unsigned char val = PosObsCost(tmp_pos_int, &OGMData_Cross);//取出删格属性
				if (val > OGM_UNKNOWN) {											//这里的栅格属性应该是有变化zx
					for (int laserindex = lower_laserindex;
							laserindex <= upper_laserindex; laserindex++) {
						if (laserindex == 90)
							int a = laserindex;

						int polar_rho = cvRound(dis / OGMRESOLUTION);
						int index = laserindex / ANGLE_SOLUTION;
						if (polar_rho < valid_dis_angle[index]) {
							valid_dis_angle[index] = polar_rho;
						}
						//1027
						if (polar_rho > total_dis_vec[index].dis) {
							total_dis_vec[index].dis = polar_rho;
						}
					}
				}
			}

		}
	}
	//�����������ͨ�о��뱣����dis_vec��
	int count=0;
	for (int i = 0; i < angle_count; i++) {
		angle_dis temp;
		temp.angle = (int) (i * ANGLE_SOLUTION);
		temp.dis = valid_dis_angle[i];
		dis_vec[i] = temp;
		if(temp.dis>1000)
			{count++;}
	}
	cout<<"ddayu 20m you "<<count<<endl;
	delete[] valid_dis_angle;
}

void CalPredictPoint::Show_result(_OGMDataStruct* OGMData_ptr, int Virtual_x,
		int Virtual_y, vector<int> out_vec, int chosen_angle,
		state_struct vehicle_state_GPS) {
	//LXN	
	IplImage *m_DilatedOGMData_inflated_Image = cvCreateImage(
			cvSize(cvRound(imagewidth), cvRound(imageheight)), 8, 3);
	cvZero(m_DilatedOGMData_inflated_Image);

	/*	for(int m1=0;m1<OGMHEIGHT_CELL;m1++)
	 {
	 for(int n1=0;n1<OGMWIDTH_CELL;n1++)
	 {
	 UCHAR tmpchar=*(OGMData_ptr->m_OccupiedMap+m1*OGMWIDTH_CELL+n1);
	 if(tmpchar > OGM_PASS)
	 {
	 UCHAR val = 255;
	 if(tmpchar == OGM_UNKNOWN)
	 val = 125;
	 UCHAR *ptr1=(UCHAR*)(m_DilatedOGMData_inflated_Image->imageData + ((imageheight - m1 ) * 1 - 1) * m_DilatedOGMData_inflated_Image->widthStep);
	 ptr1[3*(n1)]=val;
	 ptr1[3*(n1)+1]=val;
	 ptr1[3*(n1)+2]=val;
	 }
	 }
	 }*/

	CvPoint point;
	CvPoint vehicle_point, virtual_pos;
	CvPoint back_center_point;
	pos Pos_forward, show_pos;
	//int predict_dis=MAX(Cal_dist_with_angle(chosen_angle,Virtual_x,Virtual_y)*OGMRESOLUTION,3);
	int predict_dis = MAX(
			dis_vec[chosen_angle/ANGLE_SOLUTION].dis*OGMRESOLUTION, 3);
	predict_dis = 15;
	cal_predic_point(VITURA_LIDAR2VEHICLE, chosen_angle, predict_dis,
			&Pos_forward);
	point.x = cvRound(Pos_forward.x / OGMRESOLUTION) + (OGMWIDTH_CELL - 1) / 2;
	point.y = imageheight
			- cvRound(
					((Pos_forward.y) / OGMRESOLUTION + VEHICLEPOSINOGM_Y_CELL));
	vehicle_point.x = (OGMWIDTH_CELL - 1) / 2;
	//vehicle_point.y=imageheight-VEHICLEPOSINOGM_Y_CELL-(VITURA_LIDAR2VEHICLE/OGMRESOLUTION);
	vehicle_point.y = imageheight - VEHICLEPOSINOGM_Y_CELL;
//		pos back_road_center_point;lzz
	back_road_center_point.x = 0;
	back_road_center_point.y = 0;
	cal_predic_point(BACK_VIRTUTA_LIDAR2VEHICLE, back_road_angle, 15,
			&back_road_center_point);
	back_center_point.x = cvRound(back_road_center_point.x / OGMRESOLUTION)
			+ (OGMWIDTH_CELL - 1) / 2;
	back_center_point.y = imageheight
			- cvRound(
					((back_road_center_point.y) / OGMRESOLUTION
							+ VEHICLEPOSINOGM_Y_CELL));

	//	cvCircle(m_DilatedOGMData_inflated_Image,point,3,cvScalar(255,255,0),-1);
	cvCircle(m_DilatedOGMData_inflated_Image, vehicle_point, 5,
			cvScalar(255, 0, 0), -1);
	//	cvCircle(m_DilatedOGMData_inflated_Image,back_center_point,5,cvScalar(0,255,0),-1);
	//1026 ����ģ��
	if (0) {
		for (int i = 0; i < TOTAL_ANGLE_COUNT; i++) {
			cal_predic_point(VITURA_LIDAR2VEHICLE, i * ANGLE_SOLUTION,
					dis_vec[i].dis * OGMRESOLUTION, &Pos_forward);
			point.x = cvRound(Pos_forward.x / OGMRESOLUTION)
					+ (OGMWIDTH_CELL - 1) / 2;
			point.y = imageheight
					- cvRound(
							((Pos_forward.y) / OGMRESOLUTION
									+ VEHICLEPOSINOGM_Y_CELL));
			cvLine(m_DilatedOGMData_inflated_Image, vehicle_point, point,
					cvScalar(0, 0, 255));
		}
	}
	if (1) {
		for (int i = 0; i < out_vec.size(); i++) {
			cal_predic_point(VITURA_LIDAR2VEHICLE, out_vec[i], 15,
					&Pos_forward);
			point.x = cvRound(Pos_forward.x / OGMRESOLUTION)
					+ (OGMWIDTH_CELL - 1) / 2;
			point.y = imageheight
					- cvRound(
							((Pos_forward.y) / OGMRESOLUTION
									+ VEHICLEPOSINOGM_Y_CELL));
			cvLine(m_DilatedOGMData_inflated_Image, vehicle_point, point,
					cvScalar(255, 0, 255), 2);
		}
	}
	//	cvLine(m_DilatedOGMData_inflated_Image,vehicle_point,back_center_point,cvScalar(255,0,255),2);
	//	cvLine(m_DilatedOGMData_inflated_Image,vehicle_point,point,cvScalar(0,0,255),2);
	//	cvLine(m_DilatedOGMData_inflated_Image,vehicle_point,back_center_point,cvScalar(0,255,0),2);
	if (0) {
		for (int i = 0; i < prob_cross_pos.size(); i++) {
			if (prob_cross_pos[i].weight > 0.9) {
				double local_angle = (pi / 2 + prob_cross_pos[i].angle
						- vehicle_state_GPS.theta);
				local_angle = unifytheta(local_angle);
				int local_angle2 = local_angle * 180 / pi;
				cal_predic_point(VITURA_LIDAR2VEHICLE, local_angle2, 15,
						&Pos_forward);
				point.x = cvRound(Pos_forward.x / OGMRESOLUTION)
						+ (OGMWIDTH_CELL - 1) / 2;
				point.y = imageheight
						- cvRound(
								((Pos_forward.y) / OGMRESOLUTION
										+ VEHICLEPOSINOGM_Y_CELL));
				cvLine(m_DilatedOGMData_inflated_Image, vehicle_point, point,
						cvScalar(0, 0, 255), 2);
			}

		}
	}
	cvShowImage("predict_img", m_DilatedOGMData_inflated_Image);
	int key = cvWaitKey(10);
	if (key == 32) {
		cvSaveImage("1.bmp", m_DilatedOGMData_inflated_Image);
		//1027
		if (0) {
			FILE*fp;
			fp = fopen("norm.txt", "w");
			for (int i = 0; i < TOTAL_ANGLE_COUNT; i++) {
				float total_dis = MIN(30, 30.0);
				float valid_dis = MIN(dis_vec[i].dis*OGMRESOLUTION, 30.0);
				float norm = MIN(valid_dis / total_dis, 1.0);
				fprintf(fp, "%d %f\n", i * ANGLE_SOLUTION, norm);
			}
			fclose(fp);
		}
	}
	cvReleaseImage(&m_DilatedOGMData_inflated_Image);

}

void CalPredictPoint::Show_boundary(_OGMDataStruct* OGMData_ptr,
		vector<pos> left_boundary_point, vector<pos> right_boundary_point,
		float path_length, int tempary_angle,Cross_Road_Param *Cross_Show) {
	float l_a0 = Cross_Show->l_a0;
	float l_a1 = Cross_Show->l_a1;
	float l_a2 = Cross_Show->l_a2;
	float r_a0 = Cross_Show->r_a0;
	float r_a1 = Cross_Show->r_a1;
	float r_a2 = Cross_Show->r_a2;
	float c_a0 = Cross_Show->c_a0;
	float c_a1 = Cross_Show->c_a1;
	float c_a2 = Cross_Show->c_a2;
//	float l_a0 = Cross0.l_a0 ;
//				float l_a1 = Cross0.l_a1;
//				float l_a2 = Cross0.l_a2 ;
//				float r_a0 = Cross0.r_a0;
//				float r_a1 = Cross0.r_a1 ;
//				float r_a2 = Cross0.r_a2 ;
//				float c_a0 = Cross0.c_a0 ;
//				float c_a1 = Cross0.c_a1;
//				float c_a2 = Cross0.c_a2 ;

	IplImage* road_boounday = cvCreateImage(
			cvSize(cvRound(imagewidth), cvRound(imageheight)), 8, 3);
	cvZero(road_boounday);
	CvPoint vehicle_point, point;
	//画车辆位置
	vehicle_point.x = (OGMWIDTH_CELL - 1) / 2;
	vehicle_point.y = imageheight - VEHICLEPOSINOGM_Y_CELL;
//	cvCircle(road_boounday, vehicle_point, 5, cvScalar(0, 255, 0));
	pos Pos_forward;
	int predict_dis = MAX(
			dis_vec[tempary_angle/ANGLE_SOLUTION].dis*OGMRESOLUTION, 3);
	//画车前15米位置点
	predict_dis = 15;
	cal_predic_point(VITURA_LIDAR2VEHICLE, tempary_angle, predict_dis,
			&Pos_forward);
	point.x = cvRound(Pos_forward.x / OGMRESOLUTION) + (OGMWIDTH_CELL - 1) / 2;
	point.y = imageheight
			- cvRound(
					((Pos_forward.y) / OGMRESOLUTION + VEHICLEPOSINOGM_Y_CELL));
	cvLine(road_boounday, vehicle_point, point, cvScalar(0, 0, 255), 2);
	//地图中白色为障碍物，灰色为未知区域
	for (int m1 = 0; m1 < OGMHEIGHT_CELL; m1++) {
		for (int n1 = 0; n1 < OGMWIDTH_CELL; n1++) {
			UCHAR tmpchar = *(OGMData_ptr->m_OccupiedMap + m1 * OGMWIDTH_CELL
					+ n1);
			if (tmpchar > OGM_PASS) {
				UCHAR val = 255;
				if (tmpchar == OGM_UNKNOWN)
					val = 125;
				UCHAR *ptr1 = (UCHAR*) (road_boounday->imageData
						+ ((imageheight - m1) * 1 - 1)
								* road_boounday->widthStep);
				ptr1[3 * (n1)] = val;
				ptr1[3 * (n1) + 1] = val;
				ptr1[3 * (n1) + 2] = val;
			}
		}
	}
	//画左边界种子点
	for (int i = 0; i < left_boundary_point.size(); i++) {
		CvPoint temp_point;
		Convert_World_to_OGM((double) left_boundary_point[i].x,
				(double) left_boundary_point[i].y, &temp_point.x, &temp_point.y,
				VEHICLEPOSINOGM_X_M, vehicle_y_, OGMRESOLUTION);
		temp_point.y = OGMHEIGHT_CELL - 1 - temp_point.y;
		UCHAR *ptr1 = (UCHAR*) (road_boounday->imageData
				+ temp_point.y * road_boounday->widthStep);
		ptr1[3 * (temp_point.x)] = 0;
		ptr1[3 * (temp_point.x) + 1] = 0;
		ptr1[3 * (temp_point.x) + 2] = 255;
		cvCircle(road_boounday, temp_point, 1, cvScalar(0, 0, 255));

	}
	//画右边界种子点

	for (int i = 0; i < right_boundary_point.size(); i++) {
		CvPoint temp_point;
		Convert_World_to_OGM((double) right_boundary_point[i].x,
				(double) right_boundary_point[i].y, &temp_point.x,
				&temp_point.y, VEHICLEPOSINOGM_X_M, vehicle_y_,
				OGMRESOLUTION);
		temp_point.y = OGMHEIGHT_CELL - 1 - temp_point.y;
		cvCircle(road_boounday, temp_point, 1, cvScalar(255, 0, 0));
		UCHAR *ptr1 = (UCHAR*) (road_boounday->imageData
				+ temp_point.y * road_boounday->widthStep);
		ptr1[3 * (temp_point.x)] = 255;
		ptr1[3 * (temp_point.x) + 1] = 0;
		ptr1[3 * (temp_point.x) + 2] = 0;
	}
	//画左边界
	vector<CvPoint> bound_point;
	//��߽�
//	for (int i = VEHICLEPOSINOGM_Y_CELL - 5 / OGMRESOLUTION;
//			i < VEHICLEPOSINOGM_Y_CELL + path_length / OGMRESOLUTION; i++) {
	for (int i = 10 / 0.2;
			i < VEHICLEPOSINOGM_Y_CELL + path_length / OGMRESOLUTION; i++) {
		float y = (i - 10 / 0.2) * OGMRESOLUTION;
		float x = l_a0 + l_a1 * y + l_a2 * y * y;
		CvPoint temp_point;
		temp_point.x = cvRound(x / OGMRESOLUTION) + (OGMWIDTH_CELL - 1) / 2;
		temp_point.y = imageheight - i;
		float dis = sqrt(x * x + y * y);
		if (dis < 30) {
			bound_point.push_back(temp_point);
		}
	}
	if (bound_point.size() > 1) {
		for (int i = 0; i < bound_point.size() - 1; i++) {
			if (left_boundary_vaild)
				cvLine(road_boounday, bound_point.at(i), bound_point.at(i + 1),
						cvScalar(0, 255, 0), 2);
			else
				cvLine(road_boounday, bound_point.at(i), bound_point.at(i + 1),
						cvScalar(100, 100, 0), 2);
		}
	}
	//画右边界
	bound_point.clear();
	for (int i = 10 / 0.2;
			i < VEHICLEPOSINOGM_Y_CELL + path_length / OGMRESOLUTION; i++) {
		float y = (i - 10 / 0.2) * OGMRESOLUTION;
		float x = r_a0 + r_a1 * y + r_a2 * y * y;
		CvPoint temp_point;
		temp_point.x = cvRound(x / OGMRESOLUTION) + (OGMWIDTH_CELL - 1) / 2;
		temp_point.y = imageheight - i;
		float dis = sqrt(x * x + y * y);
		if (dis < 30)
			bound_point.push_back(temp_point);

	}
	if (bound_point.size() > 1) {
		for (int i = 0; i < bound_point.size() - 1; i++) {
			if (right_boundary_valid)
				cvLine(road_boounday, bound_point.at(i), bound_point.at(i + 1),
						cvScalar(0, 255, 0), 2);
			else
				cvLine(road_boounday, bound_point.at(i), bound_point.at(i + 1),
						cvScalar(100, 100, 0), 2);
		}
	}
	//
	bound_point.clear();
	for (int i = VEHICLEPOSINOGM_Y_CELL - 5 / OGMRESOLUTION;
			i < VEHICLEPOSINOGM_Y_CELL + path_length / OGMRESOLUTION; i++) {
		float y = (i - VEHICLEPOSINOGM_Y_CELL) * OGMRESOLUTION;
		float x = c_a0 + c_a1 * y + c_a2 * y * y;
		CvPoint temp_point;
		temp_point.x = cvRound(x / OGMRESOLUTION) + (OGMWIDTH_CELL - 1) / 2;
		temp_point.y = imageheight - i;
		float dis = sqrt(x * x + y * y);
		if (dis < 30) {
			bound_point.push_back(temp_point);
		}

	}
	if (bound_point.size() > 1) {
		for (int i = 0; i < bound_point.size() - 1; i++) {
			cvLine(road_boounday, bound_point.at(i), bound_point.at(i + 1),
					cvScalar(255, 255, 0), 2);
		}
	}
	cvCircle(road_boounday, vehicle_point, 5, cvScalar(0, 255, 0));	//lzz
	if (0) {
		//��ʾ�ֲ���
		char text[20] = "road boundary!";

		CvPoint point = cvPoint(100, 100);

		CvFont font;

		cvInitFont(&font, CV_FONT_HERSHEY_DUPLEX, 0.5, 0.5, 1, 2, 8);

		cvPutText(road_boounday, text, point, &font, CV_RGB(255, 0, 0));
	}
	cvShowImage("boundary", road_boounday);
	int key = cvWaitKey(10);
	if (key == 32)
		cvSaveImage("2.bmp", road_boounday);
	cvReleaseImage(&road_boounday);
}
//�ص�·���ķ������ǰ����·
void CalPredictPoint::Clear_forward_road(_OGMDataStruct* OGMData_ptr,
		_OGMDataStruct* VelodyneRigidOGMGridData, int road_center_angle,
		float width, float height) {
	int width_cell = width / OGMRESOLUTION;
	int height_cell = height / OGMRESOLUTION;
	for (int i = VEHICLEPOSINOGM_Y_CELL;
			i < VEHICLEPOSINOGM_Y_CELL + height_cell; i++) {
		for (int j = 0; j < OGMWIDTH_CELL; j++) {
			float x = (j - (OGMWIDTH_CELL - 1) / 2) * OGMRESOLUTION;
			float y = i * OGMRESOLUTION - vehicle_y_;
			for (int w = -width_cell; w <= width_cell; w++) {
				float angle = pi / 2;
				float delta_x = x - w * OGMRESOLUTION;
				if (fabs(delta_x) > 0.01) {
					angle = atan((y) / delta_x);
					if (angle < 0)
						angle = angle + pi;
					if (angle == 0 && delta_x < 0)
						angle = angle + pi;
				} else {
					angle = pi / 2;
				}
				int angle_int = angle * 180 / pi;
				int center_laserindex = cvRound(angle_int);
				if (center_laserindex < START_ANGLE)
					center_laserindex = START_ANGLE;
				if (center_laserindex > TOTAL_END_ANGLE)
					center_laserindex = TOTAL_END_ANGLE;
				float dis = sqrt(delta_x * delta_x + y * y);
				if (dis > 0.1) {
					float anglerange = OGMRESOLUTION / dis * 180 / pi;

					int upper_laserindex = center_laserindex
							+ cvRound(anglerange / 2);
					if (upper_laserindex >= TOTAL_END_ANGLE)
						upper_laserindex = TOTAL_END_ANGLE;

					int lower_laserindex = center_laserindex
							- cvRound(anglerange / 2);
					if (lower_laserindex < START_ANGLE)
						lower_laserindex = START_ANGLE;
					if (road_center_angle >= lower_laserindex
							&& road_center_angle <= upper_laserindex) {
						OGMData_ptr->m_OccupiedMap[i * OGMWIDTH_CELL + j] =
						OGM_PASS;
						VelodyneRigidOGMGridData->m_OccupiedMap[i
								* OGMWIDTH_CELL + j] = OGM_PASS;
						break;
					}
				}

			}
		}
	}
}

void CalPredictPoint::Draw_road_boundary(_OGMDataStruct* OGMData_ptr,
		double front_road_head, int back_road_head, CvKalman* kalman,Cross_Road_Param *Cross) {
//	cout<< "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<< endl;
//	float l_a0 = Cross0.l_a0 ;
//			float l_a1 = Cross0.l_a1;
//			float l_a2 = Cross0.l_a2 ;
//			float r_a0 = Cross0.r_a0;
//			float r_a1 = Cross0.r_a1 ;
//			float r_a2 = Cross0.r_a2 ;
//			float c_a0 = Cross0.c_a0 ;
//			float c_a1 = Cross0.c_a1;
//			float c_a2 = Cross0.c_a2 ;
	//		cout<< "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<< endl;
	float l_a0 = Cross->l_a0;
	float l_a1 = Cross->l_a1;
	float l_a2 = Cross->l_a2;
	float r_a0 = Cross->r_a0;
	float r_a1 = Cross->r_a1;
	float r_a2 = Cross->r_a2;
	float c_a0 = Cross->c_a0;
	float c_a1 = Cross->c_a1;
	float c_a2 = Cross->c_a2;
//	cout<< "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<< endl;

	vector<pos> left_boundary_point, right_boundary_point;
	double tempary_angle = front_road_head;
	int a = tempary_angle / ANGLE_SOLUTION;
	float front_road_length = 20.0;	///×dis_vec[a].dis*OGMRESOLUTION*8×///这里有问题啊！！！！problem！！lzz

	Draw_road_boundary_point(OGMData_ptr, tempary_angle, back_road_head,
			front_road_length, &left_boundary_point, &right_boundary_point);


	//RANSAC
	//static int left_missed_time=0,right_missed_time=0;
	float l_matched_scale = 0.0;
	float r_matched_scale = 0.0;
	left_boundary_vaild = false;
	right_boundary_valid = false;
	int valid_boundary_num = 0;
	if (left_boundary_point.size() > 40)		//在大概10000个删格中如果存在四十个左边界种子点
			{
		//��ֱ�����
		l_matched_scale = Cal_road_boudary_RANSAC(1500, 0.9,
				&left_boundary_point, 2, 0.8, &l_a0, &l_a1, &l_a2, 1);//2为直线拟合，1表示左边界；获得拟合率
		if (l_matched_scale > 0.5)
			left_boundary_vaild = true;
		//left_missed_time=0;
		//���������
		if (!left_boundary_vaild) {
			l_matched_scale = Cal_road_boudary_RANSAC(400, 0.9,
					&left_boundary_point, 3, 0.8, &l_a0, &l_a1, &l_a2, 1);//3为二次曲线拟合，1表示左边界

			if (l_matched_scale > 0.4)
				left_boundary_vaild = true;
		}
	} else if (left_boundary_point.size() > 10) {
		l_matched_scale = Cal_road_boudary_RANSAC(200, 0.9,
				&left_boundary_point, 2, 1, &l_a0, &l_a1, &l_a2, 1);
		if (l_matched_scale >= 0.4)
			left_boundary_vaild = true;
		//left_missed_time=0;
	}
	if (right_boundary_point.size() > 40) {
		r_matched_scale = Cal_road_boudary_RANSAC(1500, 0.9,
				&right_boundary_point, 2, 0.8, &r_a0, &r_a1, &r_a2, 2);
		//r_matched_scale=Cal_road_boudary_RANSAC(1500,0.9,&right_boundary_point,3,0.4,&r_a0,&r_a1,&r_a2,2);
		if (r_matched_scale > 0.5)
			right_boundary_valid = true;
		//right_missed_time=0;
		if (!right_boundary_valid) {
			r_matched_scale = Cal_road_boudary_RANSAC(400, 0.9,
					&right_boundary_point, 3, 0.8, &r_a0, &r_a1, &r_a2, 2);
			if (r_matched_scale >= 0.4)
				right_boundary_valid = true;
		}

	} else if (right_boundary_point.size() > 10) {
		r_matched_scale = Cal_road_boudary_RANSAC(200, 0.9,
				&right_boundary_point, 2, 1, &r_a0, &r_a1, &r_a2, 2);
		if (r_matched_scale > 0.4)
			right_boundary_valid = true;
		//right_missed_time=0;
	}
	if (left_boundary_vaild && !right_boundary_valid) {
		c_a0 = 0;			//常数项
		c_a1 = l_a1;			//一次项系数
		c_a2 = l_a2;			//二次项系数
		valid_boundary_num = 1;
	}
	//����Ч
	else if (!left_boundary_vaild && right_boundary_valid) {
		c_a0 = 0;
		c_a1 = r_a1;
		c_a2 = r_a2;
		valid_boundary_num = 1;
	}
	//����Ч
	else if (left_boundary_vaild && right_boundary_valid) {
		c_a0 = 0;
		c_a1 = (l_a1 + r_a1) / 2;
		c_a2 = (l_a2 + r_a2) / 2;
		valid_boundary_num = 2;
	} else {
		//�Ƚ��������߽����϶Ƚ���ѡ��
		if (l_matched_scale > r_matched_scale) {
			c_a0 = 0;
			c_a1 = l_a1;
			c_a2 = l_a2;
			valid_boundary_num = 0;
		} else {
			c_a0 = 0;
			c_a1 = r_a1;
			c_a2 = r_a2;
			valid_boundary_num = 0;
		}

	}

	//KALMAN滤波


	//��ʼ��
	if (!kalman_inited) {

		l_a0 = -2;
		l_a1 = 0;
		l_a2 = 0;
		r_a0 = 2;
		r_a1 = 0;
		r_a2 = 0;
		c_a0 = 0;
		c_a1 = 0;
		c_a2 = 0;
		//kalman = cvCreateKalman(9, 9, 0);			//状态向量维数，测量向量维数，控制向量维数
		const float F[] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
				0, -0.5, 0.5, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 1, };
		memcpy(kalman->transition_matrix->data.fl, F, sizeof(F));	//过程矩阵
		const float H[] =
				{ 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
						0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
						1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
						0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
						0, 0, 1, };
		memcpy(kalman->measurement_matrix->data.fl, H, sizeof(H));	//测量矩阵
		const float Q[] = { 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0.1, };
		memcpy(kalman->process_noise_cov->data.fl, Q, sizeof(Q));	//过程噪声

		const float R[] =
				{ 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
						0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
						1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
						0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
						0, 0, 1, };
		memcpy(kalman->measurement_noise_cov->data.fl, R, sizeof(R));	//测量噪声

		const float P[] = { 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0,
				0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0,
				10, };
		memcpy(kalman->error_cov_post->data.fl, P, sizeof(P));	//初始化最小协方差矩阵
		kalman_inited = true;
		CvMat* measurement = cvCreateMat(9, 1, CV_32FC1);
		kalman->state_post->data.fl[0] = l_a0;
		kalman->state_post->data.fl[1] = r_a0;
		kalman->state_post->data.fl[2] = r_a0 - l_a0;
		kalman->state_post->data.fl[3] = l_a1;
		kalman->state_post->data.fl[4] = l_a2;
		kalman->state_post->data.fl[5] = r_a1;
		kalman->state_post->data.fl[6] = r_a2;
		kalman->state_post->data.fl[7] = c_a1;
		kalman->state_post->data.fl[8] = c_a2;
		cvKalmanPredict(kalman, 0);			//进行预测，控制向量为0
		measurement->data.fl[0] = l_a0;
		measurement->data.fl[1] = r_a0;
		measurement->data.fl[2] = r_a0 - l_a0;
		measurement->data.fl[3] = l_a1;
		measurement->data.fl[4] = l_a2;
		measurement->data.fl[5] = r_a1;
		measurement->data.fl[6] = r_a2;
		measurement->data.fl[7] = c_a1;
		measurement->data.fl[8] = c_a2;
		cvKalmanCorrect(kalman, measurement);	//得到测量值，measurement为最后的测量向量
		l_a0 = kalman->state_post->data.fl[0];
		l_a1 = kalman->state_post->data.fl[3];
		l_a2 = kalman->state_post->data.fl[4];
		r_a0 = kalman->state_post->data.fl[1];
		r_a1 = kalman->state_post->data.fl[5];
		r_a2 = kalman->state_post->data.fl[6];
		c_a1 = kalman->state_post->data.fl[7];
		c_a2 = kalman->state_post->data.fl[8];
		cvReleaseMat(&measurement);
	}
	//kalman����
	else {
		CvMat* measurement = cvCreateMat(9, 1, CV_32FC1);
		cvKalmanPredict(kalman, 0);
		static int failed_count = 0;
//		measurement->data.fl[7] = c_a1;
//		measurement->data.fl[8] = c_a2;
		//���߽����Ч
		if (left_boundary_vaild && right_boundary_valid) {
			measurement->data.fl[0] = l_a0;
			measurement->data.fl[1] = r_a0;
			measurement->data.fl[2] = r_a0 - l_a0;
			measurement->data.fl[3] = l_a1;
			measurement->data.fl[4] = l_a2;
			measurement->data.fl[5] = r_a1;
			measurement->data.fl[6] = r_a2;
			measurement->data.fl[7] = c_a1;
			measurement->data.fl[8] = c_a2;
			failed_count = 0;
		}
		//��߽����Ч
		else if (left_boundary_vaild && !right_boundary_valid) {
			measurement->data.fl[0] = l_a0;
			measurement->data.fl[1] = kalman->state_pre->data.fl[1];	//使用预测值
			measurement->data.fl[2] = measurement->data.fl[1]
					- measurement->data.fl[0];
			measurement->data.fl[3] = l_a1;
			measurement->data.fl[4] = l_a2;
			measurement->data.fl[5] = l_a1;	//kalman->state_pre->data.fl[5];
			measurement->data.fl[6] = l_a2;	//kalman->state_pre->data.fl[6];
			measurement->data.fl[7] = c_a1;
			measurement->data.fl[8] = c_a2;
			failed_count = 0;
		}
		//�ұ߽����Ч
		else if (!left_boundary_vaild && right_boundary_valid) {
			measurement->data.fl[0] = kalman->state_pre->data.fl[0];
			measurement->data.fl[1] = r_a0;
			measurement->data.fl[2] = measurement->data.fl[1]
					- measurement->data.fl[0];
			measurement->data.fl[3] = r_a1;	//kalman->state_pre->data.fl[3];
			measurement->data.fl[4] = r_a2;	//kalman->state_pre->data.fl[4];
			measurement->data.fl[5] = r_a1;
			measurement->data.fl[6] = r_a2;
			measurement->data.fl[7] = c_a1;
			measurement->data.fl[8] = c_a2;
			failed_count = 0;
		} else {
			measurement->data.fl[0] = kalman->state_pre->data.fl[0];
			measurement->data.fl[1] = kalman->state_pre->data.fl[1];
			measurement->data.fl[2] = measurement->data.fl[1]
					- measurement->data.fl[0];
			measurement->data.fl[3] = kalman->state_pre->data.fl[3];
			measurement->data.fl[4] = kalman->state_pre->data.fl[4];
			measurement->data.fl[5] = kalman->state_pre->data.fl[5];
			measurement->data.fl[6] = kalman->state_pre->data.fl[6];
			measurement->data.fl[7] = kalman->state_pre->data.fl[7];
			measurement->data.fl[8] = kalman->state_pre->data.fl[8];
			failed_count++;
		}
		cvKalmanCorrect(kalman, measurement);
		l_a0 = kalman->state_post->data.fl[0];
		l_a1 = kalman->state_post->data.fl[3];
		l_a2 = kalman->state_post->data.fl[4];
		r_a0 = kalman->state_post->data.fl[1];
		r_a1 = kalman->state_post->data.fl[5];
		r_a2 = kalman->state_post->data.fl[6];
		c_a1 = kalman->state_post->data.fl[7];
		c_a2 = kalman->state_post->data.fl[8];
//		Cross0.l_a0 = l_a0;
//		Cross0.l_a1 = l_a1;
//		 Cross0.l_a2  =l_a2;
//		Cross0.r_a0=  r_a0 ;
//		Cross0.r_a1 =  r_a1;
//		Cross0.r_a2=  r_a2 ;
//		 Cross0.c_a0 =c_a0 ;
//		Cross0.c_a1 = c_a1;
//		Cross0.c_a2 =  c_a2;
		Cross->l_a0 = l_a0;
				Cross->l_a1 = l_a1;
				 Cross->l_a2  =l_a2;
				Cross->r_a0=  r_a0 ;
				Cross->r_a1 =  r_a1;
				Cross->r_a2=  r_a2 ;
				 Cross->c_a0 =c_a0 ;
				Cross->c_a1 = c_a1;
				Cross->c_a2 =  c_a2;
		cvReleaseMat(&measurement);
		if (failed_count == 5) {
			kalman_inited = false;
		}

	}

	Show_boundary(OGMData_ptr, left_boundary_point, right_boundary_point,
			front_road_length, tempary_angle,Cross);

	Detected_Cross[Drawn_Cross_Num].Orientation = tempary_angle;
	Detected_Cross[Drawn_Cross_Num].left_param0 = l_a0;
	Detected_Cross[Drawn_Cross_Num].left_param1 = l_a1;
	Detected_Cross[Drawn_Cross_Num].left_param2 = l_a2;
	Detected_Cross[Drawn_Cross_Num].right_param0 = r_a0;
	Detected_Cross[Drawn_Cross_Num].right_param1 = r_a1;
	Detected_Cross[Drawn_Cross_Num].right_param2 = r_a2;
	Detected_Cross[Drawn_Cross_Num].center_param0 = c_a0;
	Detected_Cross[Drawn_Cross_Num].center_param1 = c_a1;
	Detected_Cross[Drawn_Cross_Num].center_param2 = c_a2;
	Detected_Cross[Drawn_Cross_Num].Boundary_Valid = left_boundary_vaild || right_boundary_valid;
	Detected_Cross[Drawn_Cross_Num].Valid_Boundary_Num = valid_boundary_num;
	Drawn_Cross_Num++;
}
void CalPredictPoint::Draw_road_boundary_point(_OGMDataStruct* OGMData_ptr,
		double cross_angle_, int back_road_head, float path_length,
		vector<pos>* left_boundary_point, vector<pos>* right_boundary_point) {
	left_boundary_point->clear();
	right_boundary_point->clear();
	float range;
//		if(back_road_head-cross_angle_>210)lzz
	if (abs(back_road_head - cross_angle_ - 180) > 30)
		range = -10;			//直道，或者前后道路方向差别大于30度时，车后提取道路长度为10米
	else
		range = -20;
	while (range < MIN(path_length - 3, 30))			//车前最大提取道路长度，为30米
	{
		state_struct newstate;
		int theta = 90;
		float ini_pos = 0;
		if (range < BACK_VIRTUTA_LIDAR2VEHICLE) {
			theta = back_road_head - 180;
			ini_pos = BACK_VIRTUTA_LIDAR2VEHICLE;
		} else if (range > VITURA_LIDAR2VEHICLE) {
			theta = cross_angle_;
			ini_pos = VITURA_LIDAR2VEHICLE;
		}
		newstate.position.x = range * cos(theta * pi / 180);
		newstate.position.y = range * sin(theta * pi / 180) + ini_pos;
		range = range + 0.2;

		int range_cell = BOUNDARY_EXTRACT_RANGE / OGMRESOLUTION;

		for (int i = 0; i < range_cell; i++) {
			float delta = -i * OGMRESOLUTION;
			float x = newstate.position.x + delta * sin(theta * pi / 180);
			float y = newstate.position.y - delta * cos(theta * pi / 180);
			pos_int tmp_pos_int;
			Convert_World_to_OGM(x, y, &tmp_pos_int.x, &tmp_pos_int.y,
			VEHICLEPOSINOGM_X_M, vehicle_y_, OGMRESOLUTION);//其实是把当前车体坐标系位置映射到删格地图
			unsigned char val = PosObsCost(tmp_pos_int, OGMData_ptr);
			if (val > OGM_UNKNOWN) {
				double g_x, g_y;
				Convert_OGM_to_World(tmp_pos_int.x, tmp_pos_int.y, &g_x, &g_y,
						(OGMWIDTH_CELL - 1) / 2 * OGMRESOLUTION,
						vehicle_y_, OGMRESOLUTION);
				pos boundary_point;
				boundary_point.x = g_x;
				boundary_point.y = g_y;
				for (int m = MIN(left_boundary_point->size() - 5, 0);
						m < left_boundary_point->size(); m++)		//不压入相同的种子点
						{
					pos his_boundary_point = left_boundary_point->at(m);
					if (boundary_point.x == his_boundary_point.x) {
						if (boundary_point.y == his_boundary_point.y) {
							break;
						}
					}
				}
				left_boundary_point->push_back(boundary_point);
				break;
			}
		}
		//�Ҳ�
		for (int i = 0; i < range_cell; i++) {
			float delta = i * OGMRESOLUTION;
			float x = newstate.position.x + delta * sin(theta * pi / 180);
			float y = newstate.position.y - delta * cos(theta * pi / 180);
			////ä��ͳ�Ʋ�׼
			//if (fabs(y)<5)
			//{
			//	continue;
			//}
			pos_int tmp_pos_int;
			Convert_World_to_OGM(x, y, &tmp_pos_int.x, &tmp_pos_int.y,
			VEHICLEPOSINOGM_X_M, vehicle_y_, OGMRESOLUTION);
			unsigned char val = PosObsCost(tmp_pos_int, OGMData_ptr);
			if (val > OGM_UNKNOWN) {
				double g_x, g_y;
				Convert_OGM_to_World(tmp_pos_int.x, tmp_pos_int.y, &g_x, &g_y,
						(OGMWIDTH_CELL - 1) / 2 * OGMRESOLUTION,
						vehicle_y_, OGMRESOLUTION);
				pos boundary_point;
				boundary_point.x = g_x;
				boundary_point.y = g_y;
				//��ֹ�ظ����
				for (int m = MIN(right_boundary_point->size() - 5, 0);
						m < right_boundary_point->size(); m++) {
					pos his_boundary_point = right_boundary_point->at(m);
					if (boundary_point.x == his_boundary_point.x) {
						if (boundary_point.y == his_boundary_point.y) {
							break;
						}
					}
				}
				right_boundary_point->push_back(boundary_point);
				break;
			}
		}
	}
}
float CalPredictPoint::Cal_road_boudary_RANSAC(int iteration_num,
		float threshold_scale, vector<pos>* boundary_poinnt, int sample_num,
		float bias_threshold, float* a0, float* a1, float* a2, int boud_style) {
	int inter = 0;
	int point_num = boundary_poinnt->size();
	int *choesn_point = new int[point_num];
	double* px = new double[sample_num * sample_num];
	double* py = new double[sample_num];
	int max_matched_point_num = 0;
	vector<pos> point_ransac;
	while (inter < iteration_num) {
		inter++;
		memset(choesn_point, 0, sizeof(int) * point_num);
		point_ransac.clear();
		int x = 0;					//���ѡȡ��x�������
		//�����ȡsample_num����
		int num = MIN(sample_num * 5, boundary_poinnt->size());	//因为boundary_poinnt->size()大于40，所以肯定是前者即10个！
		//随机选取num个种子点
		for (int i = 0; i < num; i++) {
			do {
				x = rand() % point_num;
			} while (choesn_point[x]);
			choesn_point[x] = 1;
			point_ransac.push_back(boundary_poinnt->at(x));
		}
		float temp_a0, temp_a1, temp_a2;
		Cal_road_boudary_LSM(&point_ransac, &temp_a0, &temp_a1, &temp_a2,
				sample_num);		//sample_num=2为直线，=3为二次曲线。
		if (boud_style == 1) {
			if (temp_a0 > -1 || temp_a0 < -7)
				continue;
		}
		if (boud_style == 2) {
			if (temp_a0 < 1 || temp_a0 > 7)
				continue;
		}
		int matched_point_num = 0;
		for (int i = 0; i < point_num; i++) {
			float x = boundary_poinnt->at(i).x;
			float y = boundary_poinnt->at(i).y;
			float predict_x = temp_a0 + temp_a1 * y + temp_a2 * y * y;
			float del_x = fabs(predict_x - x);
			if (del_x < bias_threshold)		//如果偏差在左右bias_threshold范围内
					{
				matched_point_num++;
			}

		}
		if (matched_point_num > point_num * threshold_scale) {
			*a0 = temp_a0;
			*a1 = temp_a1;
			*a2 = temp_a2;
			break;
		} else if (matched_point_num > max_matched_point_num) {
			max_matched_point_num = matched_point_num;
			*a0 = temp_a0;
			*a1 = temp_a1;
			*a2 = temp_a2;
		}

	}
	delete[] choesn_point;
	delete[] px;
	delete[] py;
	float matched_sacle = (float) max_matched_point_num / point_num;	//返回拟合率
	return matched_sacle;
}

float CalPredictPoint::Cal_road_boudary_LSM(vector<pos>* boundary_poinnt,
		float* a0, float* a1, float* a2, int boud_style) {
	if (boud_style == 3) {
		double* px = new double[3 * 3];
		double* py = new double[3];
		for (int i = 0; i < boundary_poinnt->size(); i++) {
			float x = boundary_poinnt->at(i).x;
			float y = boundary_poinnt->at(i).y;
			px[0] += 1;
			px[1] += y;
			px[2] += y * y;
			px[5] += y * y * y;
			px[8] += y * y * y * y;
			py[0] += x;
			py[1] += x * y;
			py[2] += x * y * y;
		}
		px[3] = px[1];
		px[4] = px[2];
		px[6] = px[2];
		px[7] = px[5];
		CvMat xMat = cvMat(3, 3, CV_64FC1, px);
		CvMat yMat = cvMat(3, 1, CV_64FC1, py);
		CvMat* result = cvCreateMat(3, 1, CV_64FC1);
		cvInvert(&xMat, &xMat, 0);
		cvGEMM(&xMat, &yMat, 1, NULL, 0, result, 0);
		*a0 = cvmGet(result, 0, 0);
		*a1 = cvmGet(result, 1, 0);
		*a2 = cvmGet(result, 2, 0);
		delete[] px;
		delete[] py;
		cvReleaseMat(&result);
	}
	if (boud_style == 2) {
		double* px = new double[2 * 2];
		double* py = new double[2];
		for (int i = 0; i < boundary_poinnt->size(); i++) {
			float x = boundary_poinnt->at(i).x;
			float y = boundary_poinnt->at(i).y;
			px[0] += 1;
			px[1] += y;
			px[2] += y;
			px[3] += y * y;
			py[0] += x;
			py[1] += x * y;
		}
		CvMat xMat = cvMat(2, 2, CV_64FC1, px);
		CvMat yMat = cvMat(2, 1, CV_64FC1, py);
		CvMat* result = cvCreateMat(2, 1, CV_64FC1);
		cvInvert(&xMat, &xMat, 0);
		cvGEMM(&xMat, &yMat, 1, NULL, 0, result, 0);
		*a0 = cvmGet(result, 0, 0);
		*a1 = cvmGet(result, 1, 0);
		*a2 = 0;
		delete[] px;
		delete[] py;
		cvReleaseMat(&result);
	}
	return 0;
}
