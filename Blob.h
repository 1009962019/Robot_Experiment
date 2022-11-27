#ifndef _BLOB_H_
#define _BLOB_H_

#include <iostream>
#include "opencv.hpp"
#include "Eigen/Dense"
#include "JAKARobot_API.h"

//using namespace cv;//这里不能用该命名空间,会和JAKAAPI.h中引用的系统库起冲突
using namespace std;
using namespace Eigen;

#define PI 3.1416

//带夹具别小于140,不带夹具别小于80
#define BLOB_HEIGHT 100 //101(尖顶标定)、103(尖顶test)、93(气爪test)，120气爪
#define Crab_Height 93 //气爪抓取放在桌面上的色块时Z轴高度
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

class JAKAZuRobot;//类的嵌套使用需要前向声明

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
	 * @brief 将该实例的数据由图像坐标系转为基座坐标系
	 * @param mHomog 3x3单应矩阵
	*/
	void InfoChange(cv::Mat& mHomog);
	
	/**
	 * @brief 根据曲线几何特征及采样点总数计算各采样点上机械臂的末端位姿
	 * @param cnt 总的离散点个数
	 * @param end_arr 存末端位姿的6*cnt的数组
	 * @param height_offset 各点的Z轴高度偏置
	*/
	double* Curve_Cal(const int cnt, double height_offset);

	/**
	 * @brief 根据颜色计算出目标点
	 * @param  
	*/
	double get_target(JAKAZuRobot& jakaRob);

public:
	cv::Point2f m_center;	//中心点
	cv::Point2f m_pts;		//主轴上另一点
	cv::Point2f m_target;	//目标位置
	float m_angle;		//主轴角度
	Color m_color;
};

#endif