#include "stdafx.h"
#include "Blob.h"
#include "JAKARobot_API.h"
#include "subFunctions.h"
#include <iostream>
#include <fstream>  
#include <string> 
#include "Eigen/Dense"
#include "task.h"
using namespace std;
using namespace Eigen;

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#elif defined(_MSC_VER)
#pragma warning(disable : 4996)
#endif


/* 注意：JAKA API的单位分别是 毫米 和 度   别搞错单位了 
	NO_ERR = 0
*/

char* mhomog_path = "homo_matrix.xml";

#if 0
int main()
{
	Mat src = imread("C:\\Users\\lrq\\Desktop\\OTSU.png");
	cvtColor(src, src, COLOR_BGR2HSV);
	vector<Mat> channels;
	split(src, channels);
	Mat t = channels[2];
	double thresh = threshold(t, t, 85, 255, THRESH_BINARY);
	cout << thresh << endl;
}
#endif

#if 1
/**
 * @brief 完整的工作demo
 * @return
*/
int main()
{
	JAKAZuRobot jakaRob;
	//Robot_Init(jakaRob);
	//PhotoInput();
	//Eye2Hand_Calibration();

	Mat H = loadHomog(mhomog_path);
	Mat Img_Input = imread("./blob/kk.jpg");
	vector<Blob> my_blobs;
	findBlobs(Img_Input, my_blobs);

	for (vector<Blob>::iterator it = my_blobs.begin(); it != my_blobs.end(); ++it)
	{
		(*it).InfoChange(H);
		cout << (*it); //输出色块信息
		double height_offset = (*it).get_target(jakaRob);

		//cout << endl << "1.抓取色块" << endl;
		//jakaRob.Crab_Blob((*it), height_offset, 5.0);

		//cout << endl << "2.运输色块" << endl;
		jakaRob.Transport_Blob((*it), height_offset, 430);

		cout << endl << "3.放置色块" << endl;
		jakaRob.Lay_Blob((*it), height_offset, 5.0);
	}

	cout << "全部结束" << endl;
	waitKey(0);

	jakaRob.login_out();//结束

}
#endif



#if 0
/**
 * @brief 用于测试视觉识别部分的代码
 * @return 
*/
int main()
{
	//PhotoInput();
	Eye2Hand_Calibration();

	Mat H = loadHomog(mhomog_path);
	Mat Img_Input = imread("./blob/5.jpg");
	vector<Blob> my_blobs;
	findBlobs(Img_Input, my_blobs);
	for (vector<Blob>::iterator it = my_blobs.begin(); it != my_blobs.end(); ++it)
	{
		(*it).InfoChange(H);
		cout << (*it);
	}
	cout << "请按空格键关闭窗口！" << endl;
	waitKey(0);

}
#endif

#if 0
/**
 * @brief 用于相机标定时将末端移到某个位置
 * @return 
*/
int main()
{
	JAKAZuRobot jakaRob;
	Robot_Init(jakaRob);

	double end[6] = { 204,331,BLOB_HEIGHT,-180,0,0 };
	//double end[6] = { 287,238,BLOB_HEIGHT,-180,0,0 };
	//double end[6] = { 212,188,BLOB_HEIGHT,-180,0,0 };
	//double end[6] = { 182,276,BLOB_HEIGHT,-180,0,0 };
	int ret = jakaRob.moveP(end, 5.0);
	cout << "movep: " << ret << endl;
	jakaRob.waitEndMove();

	jakaRob.login_out();//结束
	cout << "移动结束" << endl;
}
#endif

#if 0
int main()
{
	JAKAZuRobot jakaRob;
	double jt_now[6] = { 0,0,0,0,0,0 };
	double end_next[6] = { 150,200,160,-60,70,80 };
	double solve[6];
	jakaRob.Cal_iKine(jt_now, end_next, solve);
	system("pause");
}
#endif

#if 0
int main()
{
	JAKAZuRobot jakaRob;
	double T[4][4] = {
	{ 0.512358962775918, -0.858119909692108, -0.0334441901261411,	0.261885052682280 },
	{ -0.858592201243573, -0.512658968830468,	0.000462213584561732,	0.294352405492099 },
	{ -0.0175420987028802,	0.0284781015464451, -0.999440479721233,	0.291656649169983 },
	{0.0, 0.0, 0.0, 1.0} };
	double qs[48];
	int num = jakaRob.iKinematics(T, qs);
	system("pause");
}
#endif

#if 0
int main()
{	
	//读取关节角
	//double pts[3000];
	//char pl[128];
	//double *pt = pts;
	//fstream infile("joints_inc.txt", ios::in);
	//while (infile.getline(pl, 128))
	//{
	//	sscanf(pl, "%lf,%lf,%lf,%lf,%lf,%lf", pt, pt + 1, pt + 2, pt + 3, pt + 4, pt + 5);
	//	pt += 6;
	//}
	//infile.close();

	JAKAZuRobot jakaRob;
	double jointpos1[6];
	jakaRob.getJoints(jointpos1);

	//逆解的用法
	double T[4][4] = { { 0.512358962775918, -0.858119909692108, -0.0334441901261411,	0.261885052682280 },
	{ -0.858592201243573, -0.512658968830468,	0.000462213584561732,	0.294352405492099 },
	{ -0.0175420987028802,	0.0284781015464451, -0.999440479721233,	0.291656649169983 },
	{0.0, 0.0, 0.0, 1.0} };
	double qs[48];
	int num = jakaRob.iKinematics(T, qs);

	double jointsHome[6] = { -109.30, 81.61, -11.53, 116.11, 177.77, 107.22};
	double incJoint[6] = { 0.0, 0.0,40.0,0.0,0.0,0.0 };
	double end1[12] = {
		100,200,600,90,80,160,
							};
	int ret = jakaRob.moveJ(jointsHome, 8.0, 0);
	ret = jakaRob.moveL(incJoint, 8.0, 1);
	double end[6] = { 140,250,320,-180,0,0 };
	int ret = jakaRob.moveP(end,8.0);	//MOVEP用法
	ret = jakaRob.waitEndMove();
	
	/*CVMOVE用法*/
	jakaRob.enableCVMotion(1);
	ret = jakaRob.cvMove(end1, 2, COORD_JOINT, 1);
	ret = jakaRob.enableCVMotion(0);
	ret = jakaRob.waitEndMove();

	//LARGE_INTEGER start_t, end_t, frequency;
	//QueryPerformanceFrequency(&frequency);	//获取计时器的时钟频率
	//QueryPerformanceCounter(&start_t);		//获得计数器初始值
	//ret = jakaRob.enableCVMotion(1);
	//ret = jakaRob.cvMove(pts, 500, COORD_JOINT, 1);
	//ret = jakaRob.enableCVMotion(0);
	//QueryPerformanceCounter(&end_t);		//获得计数器初始值
	//double tt = (end_t.QuadPart - start_t.QuadPart) / frequency.QuadPart;

	jakaRob.login_out();

    return 0;
}

#endif
