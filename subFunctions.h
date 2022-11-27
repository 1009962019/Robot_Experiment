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
 * @brief ������б�ľ��ο򲢱����������
 * @param input_Array 
 * @param target 
 * @param color 
 * @return 
*/
Point draw_rotrect(Mat input_Array, RotatedRect target, Scalar color);

/**
 * @brief ����ͨ��ͼ��ת����ͨ��
 * @param binImg ���뵥ͨ����ͼ��
 * @return 
*/
Mat convertTo3Channels(const Mat& binImg);

/**
 * @brief ���㵥Ӧ���󣬲����浽�ļ�
 * @param ptsInImage ͼ���е�����������
 * @param ptsInWord ��������ϵ��Ҳ�����ǻ���������ϵ���е����������꣬2D���꣬������z�����
 * @param filenames ���㵥Ӧ����󱣴浽�ļ��У�������xml��ʽ
 * @return >=0 �ɹ� ����ʧ��
*/
int saveHomog(vector<Point2f> ptsInImage, vector<Point2f> ptsInWord, char *filenames);

/**
 * @brief ���ļ��ж�����Ӧ����
 * @param filenames ���浥Ӧ������ļ���������xml��ʽ
 * @return
*/
Mat loadHomog(char* filenames);

/**
 * @brief �����ͼ�ϱ�עһЩ��Ϣ
 * @param img 
 * @param rect_center ɫ����������
 * @param rect_color ɫ����ɫ
 * @param num ��ɫ������
*/
void Text_Annotate(InputOutputArray img, Point rect_center, Color rect_color, int num);

void Line_Annotate(InputOutputArray img, Point rect_center, double k);

/**
 * @brief �����ͼ���ҳ�ɫ��
 * @param blob_img ����ͼ��
 * @param blobs ɫ�����,ͨ�����Ա���Կ�֪��λ�ü���ɫ��Ϣ
 * @return ɫ�����
*/
int findBlobs(Mat &blob_img, vector<Blob> &blobs);

/**
 * @brief ��������ͼ��ROI�ڵ�ɫ��ƽ��ֵ��ȷ����ɫ
 * @param Inputmtx 
 * @param  
 * @return 
*/
Color Get_Hue(Mat& Inputmtx, RotatedRect roi);

/**
 * @brief �ɵ�Ӧ�����ĳ���������任
 * @param H ��Ӧ����
 * @param P_pixel ��������
 * @return �任�������
*/
Point2f Get_P_World(Mat H, Point2f P_pixel);

/**
 * @brief ����ת�任�����Z-Y-Xŷ����(radian) �� ƽ��ʸ������һ����α任����
 * @param Inputmtx(out)
 * @param transl
 * @param EurAngle,��λ:rad
*/
void Get_TransMtx(Isometry3d& Inputmtx, Vector3d& transl, Vector3d& EurAngle);

double L2_Norm(double* arr1, double* arr2);
void resample_points(double* pts_in1, double* pts_in2, int num1, double* pts_out1, double* pts_out2, int* num2, double dist);
#endif