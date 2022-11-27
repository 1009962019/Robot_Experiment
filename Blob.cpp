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

/*  TODO:�ȶ�������ߣ�Ȼ����������˲��͵Ⱦ����*/
double* Blob::Curve_Cal(const int cnt, double height_offset)
{
	/* 1.���������ɢ�㹹�ɵ����� */
	float a = pow(pow((this->m_center.x - this->m_target.x), 2) + pow((this->m_center.y - this->m_target.y), 2), 0.5)*0.5;
	float b = 200;
	float n = 2.5;
	ArrayXd theta = ArrayXd::LinSpaced(cnt, 0, PI);

	ArrayXd X_coordinates(cnt);//���в������X����
	X_coordinates << a * sign(cos(theta)) * pow(abs(cos(theta)), (2 / n)) - a;//���ƽ�Ƶ�ԭ��
	ArrayXd Z_coordinates(cnt);//���в������Z����
	Z_coordinates << b * sign(sin(theta)) * pow(abs(sin(theta)), (2 / n)) + Crab_Height + height_offset;

	/* 1.5 ����ɢ�����߽��еȾ��ز��� */
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
		cout << "cnt_new��С" << endl;
		return 0;
	}
	ArrayXd Y_coordinates = ArrayXd::Zero(cnt_new);//���в������Y����
	ArrayXd X_New(cnt_new); X_New << X_temp(seq(0, cnt_new-1), 0);
	ArrayXd Z_New(cnt_new); Z_New << Z_temp(seq(0, cnt_new-1), 0);
	ArrayXXd pts(4, cnt_new);//��XYZ���������ϳ�Ϊ�������(4x1)��������������ĸ���

	pts << X_New.transpose(), Y_coordinates.transpose(),
				 Z_New.transpose(), ArrayXd::Ones(cnt_new).transpose();

	/* 2.����������ԭ��ƽ�Ƶ�ɫ�鴦 */
	Vector3d transl(this->m_center.x,this->m_center.y, 0);
	Vector3d Angle(0, 0, 0);
	Isometry3d H1 = Isometry3d::Identity(); //����4x4����α任����
	Get_TransMtx(H1, transl, Angle);
	//cout << H1.matrix() << endl;

	/* 3.��������ת��ָ��Ŀ��㷽�� */
	Isometry3d H2 = Isometry3d::Identity();
	//�������ڵ�һ����
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

	/* 2.������е�۵�����ĩ��λ�� */
	vector<VectorXd> T1;//6xcnt_new ά
	Vector3d Angle_temp(-180, 0, this->m_angle);	//����Z��Ƕ�Ҫ��,��ΪҪץ����
	Vector3d transl_temp(0, 0, 0);

	/* �ѿ�������(mm) + ŷ����(deg)��ʽ (6x1xcnt_new) */
	for (int i = 0; i < cnt_new; ++i)
	{
		VectorXd temp1(6);
		transl_temp = (Vector3d)pts_1.block<3, 1>(0, i);
		temp1 << transl_temp, Angle_temp;
		T1.push_back(temp1);
	}

	/* ���������ݴ���һ�������� */
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
		case _orange:	color = "��ɫ"; break;
		case _green:	color = "��ɫ"; break;
		case _blue:		color = "��ɫ"; break;
		case _yellow:	color = "��ɫ"; break;
		case _purple:	color = "��ɫ"; break;
		default: color = "Ĭ��";  break;
	}
	cout << "��ɫ: " << color << "  ��������: " << b.m_center << "  �Ƕ�: " << b.m_angle << endl;
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