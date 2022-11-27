#ifndef _BLOB_H_
#define _BLOB_H_

#include <iostream>
#include "opencv.hpp"
#include "Eigen/Dense"
#include "JAKARobot_API.h"

//using namespace cv;//���ﲻ���ø������ռ�,���JAKAAPI.h�����õ�ϵͳ�����ͻ
using namespace std;
using namespace Eigen;

#define PI 3.1416

//���о߱�С��140,�����о߱�С��80
#define BLOB_HEIGHT 100 //101(�ⶥ�궨)��103(�ⶥtest)��93(��צtest)��120��צ
#define Crab_Height 93 //��צץȡ���������ϵ�ɫ��ʱZ��߶�
#define Crab_Open 0
#define Crab_Close 1
#define Crab_IO 4
#define Base_Delay 500 //ms

typedef enum
{
	_orange = 0,
	_blue,
	_yellow,
	_purple,
	_green
}Color;

class JAKAZuRobot;//���Ƕ��ʹ����Ҫǰ������

class Blob
{
public:
	Blob(cv::Point2f m_center, cv::Point2f m_pts = cv::Point2f(0, 0),
			 float m_angle = 0, Color m_color = _orange, cv::Point2f m_target = cv::Point2f(150, 150))
	{
		this->m_center = m_center;
		this->m_pts = m_pts;
		this->m_angle = m_angle;
		this->m_color = m_color;
		//this->m_target = m_target;
	}
	friend ostream& operator<< (ostream& cout, Blob& b);
	/**
	 * @brief ����ʵ����������ͼ������ϵתΪ��������ϵ
	 * @param mHomog 3x3��Ӧ����
	*/
	void InfoChange(cv::Mat& mHomog);
	
	/**
	 * @brief �������߼�������������������������������ϻ�е�۵�ĩ��λ��
	 * @param cnt �ܵ���ɢ�����
	 * @param end_arr ��ĩ��λ�˵�6*cnt������
	 * @param height_offset �����Z��߶�ƫ��
	*/
	double* Curve_Cal(const int cnt, double height_offset);

	/**
	 * @brief ������ɫ�����Ŀ���
	 * @param  
	*/
	double get_target(JAKAZuRobot& jakaRob);

public:
	cv::Point2f m_center;	//���ĵ�
	cv::Point2f m_pts;		//��������һ��
	cv::Point2f m_target;	//Ŀ��λ��
	float m_angle;		//����Ƕ�
	Color m_color;
};

#endif