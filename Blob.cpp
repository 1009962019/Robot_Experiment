#include "stdafx.h"
#include "Blob.h"
#include "subFunctions.h"

using namespace cv;
using namespace std;
using namespace Eigen;

void Blob::InfoChange(Mat& mHomog)
{
	this->m_center = Get_P_World(mHomog, this->m_center);
	this->m_pts = Get_P_World(mHomog, this->m_pts);
	float temp_angle = (this->m_center.y - this->m_pts.y) / (this->m_center.x - this->m_pts.x);
	this->m_angle = atan(temp_angle) * 180 / 3.14;
}

/*  TODO:先定义出曲线，然后进行三角滤波和等距采样*/
double* Blob::Curve_Cal(const int cnt, double height_offset)
{
	/* 1.定义出由离散点构成的曲线 */
	float a = pow(pow((this->m_center.x - this->m_target.x), 2) + pow((this->m_center.y - this->m_target.y), 2), 0.5)*0.5;
	float b = 200;
	float n = 2.5;
	ArrayXd theta = ArrayXd::LinSpaced(cnt, 0, PI);

	ArrayXd X_coordinates(cnt);//所有采样点的X坐标
	X_coordinates << a * sign(cos(theta)) * pow(abs(cos(theta)), (2 / n)) - a;//起点平移到原点
	ArrayXd Z_coordinates(cnt);//所有采样点的Z坐标
	Z_coordinates << b * sign(sin(theta)) * pow(abs(sin(theta)), (2 / n)) + Crab_Height + height_offset;

	/* 1.5 对离散的曲线进行等距重采样 */
	int cnt_new = 2000;
	ArrayXd X_temp(cnt_new);  ArrayXd Z_temp(cnt_new);
	double distance = 0.65; //0.5~1.5

	//double sum = 0, idx = 0;
	//for (int i = 0; i < cnt / 2; i = i + 7)
	//{
	//	float x0 = X_coordinates.data()[i];
	//	float y0 = Z_coordinates.data()[i];
	//	float dx = X_coordinates.data()[i + 1] - x0;
	//	float dy = Z_coordinates.data()[i + 1] - y0;
	//	double a = sqrt(dx * dx + dy * dy);
	//	sum += a;
	//	idx++;
	//}
	//distance = sum/idx;

	resample_points(X_coordinates.data(), Z_coordinates.data(), cnt, X_temp.data(), Z_temp.data(), &cnt_new, distance);
	if (cnt_new >= 2000)
	{
		cout << "cnt_new过小" << endl;
		return 0;
	}
	ArrayXd Y_coordinates = ArrayXd::Zero(cnt_new);//所有采样点的Y坐标
	ArrayXd X_New(cnt_new); X_New << X_temp(seq(0, cnt_new-1), 0);
	ArrayXd Z_New(cnt_new); Z_New << Z_temp(seq(0, cnt_new-1), 0);
	ArrayXXd pts(4, cnt_new);//将XYZ三轴的坐标合成为齐次向量(4x1)，列数代表坐标的个数

	pts << X_New.transpose(), Y_coordinates.transpose(),
				 Z_New.transpose(), ArrayXd::Ones(cnt_new).transpose();

	/* 2.将曲线起点从原点平移到色块处 */
	Vector3d transl(this->m_center.x,this->m_center.y, 0);
	Vector3d Angle(0, 0, 0);
	Isometry3d H1 = Isometry3d::Identity(); //创建4x4的齐次变换矩阵
	Get_TransMtx(H1, transl, Angle);
	//cout << H1.matrix() << endl;

	/* 3.将曲线旋转到指向目标点方向 */
	Isometry3d H2 = Isometry3d::Identity();
	//仅适用于第一象限
	float k = (this->m_center.y - this->m_target.y) / (this->m_center.x - this->m_target.x);
	float t;
	if (k >= 0)
	{
		if (this->m_target.x > this->m_center.x)
			t = atanf(k) - PI;
		else
			t = atanf(k);
	}
	else
	{
		if (this->m_target.x < this->m_center.x)
			t = atanf(k);
		else
			t = atanf(k) + PI;
	}
	transl << 0, 0, 0;
	Angle << 0, 0, t;
	Get_TransMtx(H2, transl, Angle);
	//cout << "k = " << k << endl;
	//cout << "t = " << t << endl;
	//cout << H1.matrix() << endl;
	ArrayXXd pts_1(4, cnt_new);
	Isometry3d H3 = H1 * H2;
	pts_1.matrix() = H3.matrix() * pts.matrix();

	/* 2.描述机械臂的所有末端位姿 */
	vector<VectorXd> T1;//6xcnt_new 维
	Vector3d Angle_temp(-180, 0, this->m_angle);	//这里Z轴角度要改,因为要抓方块
	Vector3d transl_temp(0, 0, 0);

	/* 笛卡尔坐标(mm) + 欧拉角(deg)形式 (6x1xcnt_new) */
	for (int i = 0; i < cnt_new; ++i)
	{
		VectorXd temp1(6);
		transl_temp = (Vector3d)pts_1.block<3, 1>(0, i);
		temp1 << transl_temp, Angle_temp;
		T1.push_back(temp1);
	}

	/* 将所有数据存入一个数组内 */
	double* end_arr = new double[cnt_new * 6];
	double* ptr = end_arr;
	ptr = end_arr;
	for (vector<VectorXd>::iterator it = T1.begin(); it != T1.end(); ++it)
	{
		for (int i = 0; i < 6; ++i)
		{
			ptr[i] = (*it)[i];
		}
		ptr += 6;
	}

	return end_arr;
}

ostream& operator<< (ostream& cout, Blob& b)
{
	string color;
	switch (b.m_color)
	{
		case _orange:	color = "橙色"; break;
		case _green:	color = "绿色"; break;
		case _blue:		color = "蓝色"; break;
		case _yellow:	color = "黄色"; break;
		case _purple:	color = "紫色"; break;
		default: color = "默认";  break;
	}
	cout << "颜色: " << color << "  中心坐标: " << b.m_center << "  角度: " << b.m_angle << endl;
	cout << "------------------------------\n";
	return cout;
}


double Blob::get_target(JAKAZuRobot& jakaRob)
{
	switch (this->m_color)
	{
	case _orange:	this->m_target = Point2f(321, 209); return BLOB_HEIGHT * (jakaRob.blob_num[0]++);
	case _green:	this->m_target = Point2f(321, 209); return BLOB_HEIGHT * (jakaRob.blob_num[1]++);
	case _blue:		this->m_target = Point2f(321, 209); return BLOB_HEIGHT * (jakaRob.blob_num[2]++);
	case _yellow:	this->m_target = Point2f(321, 209); return BLOB_HEIGHT * (jakaRob.blob_num[3]++);
	case _purple:	this->m_target = Point2f(321, 209); return BLOB_HEIGHT * (jakaRob.blob_num[4]++);
	default: this->m_target = Point2f(321, 209);  break;
	}
}