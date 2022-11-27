#ifndef _SubAPI_H_
#define _SubAPI_H_

#include <iostream>
#include "opencv.hpp"
#include "Eigen/Dense"
#include "JAKARobot_API.h"
#include "Blob.h"

using namespace cv;
using namespace std;
using namespace Eigen;

#define MAX(a,b) (((a) > (b)) ? (a) : (b))

double Get_MaxValue(Mat& input_Array, Rect ROI);

/**
 * @brief 画出倾斜的矩形框并标出中心坐标
 * @param input_Array 
 * @param target 
 * @param color 
 * @return 
*/
Point draw_rotrect(Mat input_Array, RotatedRect target, Scalar color);

/**
 * @brief 将单通道图像转成三通道
 * @param binImg 输入单通道的图像
 * @return 
*/
Mat convertTo3Channels(const Mat& binImg);

/**
 * @brief 计算单应矩阵，并保存到文件
 * @param ptsInImage 图像中的特征点坐标
 * @param ptsInWord 世界坐标系（也可以是机器人坐标系）中的特征点坐标，2D坐标，不考虑z轴分量
 * @param filenames 计算单应矩阵后保存到文件中，建议用xml格式
 * @return >=0 成功 其他失败
*/
int saveHomog(vector<Point2f> ptsInImage, vector<Point2f> ptsInWord, char *filenames);

/**
 * @brief 从文件中读出单应矩阵
 * @param filenames 保存单应矩阵的文件，建议用xml格式
 * @return
*/
Mat loadHomog(char* filenames);

/**
 * @brief 在输出图上标注一些信息
 * @param img 
 * @param rect_center 色块中心坐标
 * @param rect_color 色块颜色
 * @param num 该色块的序号
*/
void Text_Annotate(InputOutputArray img, Point rect_center, Color rect_color, int num);

void Line_Annotate(InputOutputArray img, Point rect_center, double k);

/**
 * @brief 从输出图中找出色块
 * @param blob_img 输入图像
 * @param blobs 色块对象,通过其成员属性可知其位置及颜色信息
 * @return 色块个数
*/
int findBlobs(Mat &blob_img, vector<Blob> &blobs);

/**
 * @brief 计算输入图像ROI内的色度平均值并确定颜色
 * @param Inputmtx 
 * @param  
 * @return 
*/
Color Get_Hue(Mat& Inputmtx, RotatedRect roi);

/**
 * @brief 由单应矩阵对某点进行坐标变换
 * @param H 单应矩阵
 * @param P_pixel 像素坐标
 * @return 变换后的坐标
*/
Point2f Get_P_World(Mat H, Point2f P_pixel);

/**
 * @brief 由旋转变换矩阵的Z-Y-X欧拉角(radian) 和 平移矢量创建一个齐次变换矩阵
 * @param Inputmtx(out)
 * @param transl
 * @param EurAngle,单位:rad
*/
void Get_TransMtx(Isometry3d& Inputmtx, Vector3d& transl, Vector3d& EurAngle);

double L2_Norm(double* arr1, double* arr2);
void resample_points(double* pts_in1, double* pts_in2, int num1, double* pts_out1, double* pts_out2, int* num2, double dist);
#endif