#include "stdafx.h"
#include "subFunctions.h"

using namespace cv;
using namespace std;
using namespace Eigen;

double Get_MaxValue(Mat& input_Array, Rect ROI)
{
	double temp,max = 0;
	int x = ROI.x;		int y = ROI.y;
	int w = ROI.width/4;	int h = ROI.height/4;
	for (int i = -w; i < w; ++i)
	{
		for (int j = -h; j < h; ++j)
		{
			temp = input_Array.at<uint8_t>(Point2d(x + i, y + j)); //亮度最大值
			if (temp > max)
				max = temp;
		}
	}
	return max;
}

double L2_Norm(double* arr1, double* arr2)
{
	double ans = 0;
	for (int i = 0; i < 6; ++i)
	{
		ans += pow(arr2[i] - arr1[1], 2);
	}
	return ans;
}

Point draw_rotrect(Mat input_Array, RotatedRect target, Scalar color)
{
	Point2f* rec_point = new Point2f[4];
	target.points(rec_point);//4个顶点的坐标
	for (int i = 0; i < 3; i++)
	{
		line(input_Array, rec_point[i], rec_point[i + 1], color, 2);
	}
	line(input_Array, rec_point[3], rec_point[0], color, 2);

	circle(input_Array, target.center, 2, Scalar(255, 0, 0), 2, 8, 0);
	return target.center;
}

Mat convertTo3Channels(const Mat& binImg)
{
	Mat three_channel = Mat::zeros(binImg.rows, binImg.cols, CV_8UC3);
	vector<Mat> channels;
	for (int i = 0; i < 3; i++)
	{
		channels.push_back(binImg);
	}
	merge(channels, three_channel);
	return three_channel;
}

int saveHomog(vector<Point2f> ptsInImage, vector<Point2f> ptsInWord, char* filenames)
{
	Mat H = findHomography(ptsInImage, ptsInWord);
	FileStorage fs(filenames, FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "mHomog" << H;
		fs.release();
		return 1;
	}
	else
		return -1;

}

Mat loadHomog(char* filenames)
{
	Mat mHomog;
	FileStorage fs(filenames, FileStorage::READ);
	if (fs.isOpened())
	{
		fs["mHomog"] >> mHomog;
		cout << "mHomog readed = \n" << mHomog << endl << endl;
		fs.release();
		return mHomog;
	}
	else
	{
		cout << "读取单应矩阵失败" << endl;
		return mHomog;
	}
}

Color Get_Hue(Mat& Inputmtx, RotatedRect roi)
{
	float temp = 0;
	int x = roi.center.x;		int y = roi.center.y;
	//cout << "center_Hue:" << (int)Inputmtx.at<Vec3b>(Point(x, y))[0] << endl;
	temp = Inputmtx.at<Vec3b>(Point(x, y))[0];
	if ((temp>160) || (temp <20))
		return _orange;
	else if (20 <= temp && temp < 35)
		return _yellow;
	else if (35 <= temp && temp < 77)
		return _green;
	else if (78 <= temp && temp < 124)
		return _blue;
	else 
		return _purple;
}

void Get_TransMtx(Isometry3d& Inputmtx, Vector3d& transl, Vector3d& EurAngle)
{
	Matrix3d R;
	R = AngleAxisd(EurAngle(2), Vector3d::UnitZ())
		* AngleAxisd(EurAngle(1), Vector3d::UnitY())
		* AngleAxisd(EurAngle(0), Vector3d::UnitX());

	Inputmtx = Isometry3d::Identity();
	/* 注意：用这种方式对Input_mtx操作是在当前的基础上进行的,会叠加,所以每次让输入先变成单位阵 */
	Inputmtx.rotate(R); // 按照ratation_vector进行旋转
	Inputmtx.pretranslate(transl);
}

void Text_Annotate(InputOutputArray img, Point rect_center, Color rect_color, int num)
{
	int x = rect_center.x;
	int y = rect_center.y;
	string color;
	switch (rect_color)
	{
		case _orange:	color = "Orange"; break;
		case _green:	color = "Green"; break;
		case _blue:		color = "Blue"; break;
		case _yellow:	color = "Yellow"; break;
		case _purple:	color = "Purple"; break;
		default: color = "UNKOWN";  break;
	}
	putText(img, color, Point(x + 25, y), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 2);
	putText(img, to_string(num), Point(x + 40, y+20), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 2);
}

void Line_Annotate(InputOutputArray img, Point rect_center, double k)
{
	int offset;
	if (abs(k) < 0.3)
	{
		offset = 5;
		if (abs(k) < 0.05)
			offset = 1.5;
	}
	else
		offset = 23;
	Point pt_1 = Point2d(rect_center.x - (offset / k), rect_center.y - offset);
	Point pt_2 = Point2d(rect_center.x + (offset / k), rect_center.y + offset);
	arrowedLine(img, pt_1, pt_2, Scalar(255, 255, 255), 2);//绘制Y轴箭头
	//putText(img, "Y", pt_1, FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 2);

	Point pt_3 = Point2d(rect_center.x - offset, rect_center.y + offset / k);
	Point pt_4 = Point2d(rect_center.x + offset, rect_center.y - offset / k);
	arrowedLine(img, pt_3, pt_4, Scalar(255, 255, 255), 2);//绘制X轴箭头
	//putText(img, "X", pt_3, FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 2);
}

int findBlobs(Mat& blob_img, vector<Blob>& blobs)
{
	clock_t start = clock();

	Mat src, src_HSV, ret;
	resize(blob_img, src, Size(640, 480));

	/* 1.去掉桌面和小孔 */
	cvtColor(src, src_HSV, COLOR_BGR2HSV);//转到HSV空间方便颜色识别
	Scalar L_Thresh(0, 120, 30);
	Scalar H_Thresh(179, 255, 255);
	inRange(src_HSV, L_Thresh, H_Thresh, ret);//根据颜色阈值分割图像
 
	/* 2.形态学运算 */
	Mat kernel = getStructuringElement(MORPH_RECT, Size(7,7), Point(-1, -1));
	morphologyEx(ret, ret, MORPH_OPEN, kernel);
	

	/* 3.轮廓查找 */
	vector<vector<Point>> counters;
	vector<Vec4i> Hierarchy;
	findContours(ret, counters, Hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));
	cout << "轮廓数为:" << counters.size() << endl;

	/* 4.根据面积确定桌上有几个色块 */
	vector<vector<Point>> Blob_Counters;//色块的轮廓
	vector<Mat> SingleBlob_Images;//该容器中每个图只有一个被填充的色块
	vector<Rect> Rcets;

	for (int i = 0; i < counters.size(); i++)
	{
		double area = contourArea(counters[static_cast<int>(i)]);
		cout << "第" << i << "个轮廓的面积是" << area << endl;
		if (area > 500 && area < 4500)
		{
			SingleBlob_Images.push_back(Mat::zeros(src.rows, src.cols, CV_8UC3));
			drawContours(SingleBlob_Images[Blob_Counters.size()], counters, i, Scalar(255, 255, 255), -1, 8, Hierarchy, 0);//二值图上填充
			Blob_Counters.push_back(counters[i]);
			Rcets.push_back(boundingRect(*(Blob_Counters.end()-1) ));

			//drawContours(src, counters, i, Scalar(0, 255, 255), 1, 8, Hierarchy, 0);//在原图上画出轮廓
		}
	}
	cout << "色块数为:" << Blob_Counters.size() << endl;

	/* 4.对每一个色块用亮度进一步分割 */
	vector<Mat> HSV_Images;
	counters.clear();
	Hierarchy.clear();
	Mat sum_Image = Mat::zeros(src.rows, src.cols, CV_8UC3);//分割最终效果图
	Mat binary = Mat::zeros(src.rows, src.cols, CV_8UC1);

	for (int i = 0; i < Blob_Counters.size(); i++)
	{
		Mat mask = Mat::zeros(src.rows, src.cols, CV_8UC1);
		HSV_Images.push_back(src_HSV.clone() & SingleBlob_Images[i]);	//在HSV图上扣出第i个色块

		vector<Mat> channels;
		split(HSV_Images[i], channels);
		/* 用亮度进行分割 */
		Mat Value = channels[2];
		Mat img_loc;//灰度图
		Value(Rcets[i]).copyTo(img_loc);
		double value_thresh = threshold(img_loc, img_loc, 0, 255, THRESH_OTSU);
		//cout << "value_thresh1 = " << value_thresh << endl;
		Mat Value_Binary;

		double value_max = Get_MaxValue(Value, Rcets[i]);
		//cout << "value_max = " << value_max << endl;
		if (value_max - value_thresh > 30) //选择最合适的阈值
			value_thresh = MAX((value_max - 40), (value_thresh + (value_max - value_thresh)/2) );

		//cout << "value_thresh2 = " << value_thresh << endl;
		threshold(Value, Value_Binary, value_thresh, 255, THRESH_BINARY);
		morphologyEx(Value_Binary, Value_Binary, MORPH_OPEN, getStructuringElement(MORPH_RECT, Size(9, 9)));
		
		binary = binary | Value_Binary;//总的二值图
	}

	sum_Image = convertTo3Channels(binary) & src;//在原图上扣出所有色块

	/* 5.遍历每个色块的轮廓,确定其形心及方向 */
	int num = 0;
	Point Center_Pixel;
	double k; //斜率
	Point2f point_line;//形心上另一点
	counters.clear();
	Hierarchy.clear();

	findContours(binary, counters, Hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));
	cout << "num of counters:" << counters.size() << endl;
	for (int i = 0; i < counters.size(); i++)
	{
		RotatedRect rect_now = minAreaRect(counters[i]);//找每个轮廓的最小外接矩形
		if (rect_now.boundingRect2f().area() > 40)//满足该条件则说明 该轮廓是色块的轮廓
		{
			++num;
			/* 计算区域颜色平均值以确定颜色 */
			Color color = Get_Hue(src_HSV, rect_now);

			Center_Pixel = draw_rotrect(src, rect_now, Scalar(255, 255, 255));//在输入图片上画出斜边矩形框
			k = tanf(rect_now.angle * 3.14 / 180);//计算主轴斜率
			point_line = Point2d(rect_now.center.x + (30 / k), rect_now.center.y + 30);//计算直线上的一点
			
			/* 将色块数据写入容器中 */
			Blob temp(rect_now.center, point_line, 0, color);
			blobs.push_back(temp);

			/* 在输出图上标注信息 */
			Text_Annotate(src, rect_now.center, color, num);
			Line_Annotate(src, (Point)rect_now.center, k);//绘制箭头
		}
	}
	cout << "num of rect:" << num << endl;
	imshow("最终结果", src);

	clock_t end = clock();
	cout << "消耗时间 = " << (end - start) << "ms" << endl;

	cout << "请按空格键继续下一步！" << endl;
	waitKey(0);
	return num;//返回色块个数
}

Point2f Get_P_World(Mat H, Point2f P_pixel)
{
	double h11 = H.at<double>(0, 0); double h12 = H.at<double>(0, 1); double h13 = H.at<double>(0, 2);
	double h21 = H.at<double>(1, 0); double h22 = H.at<double>(1, 1); double h23 = H.at<double>(1, 2);
	double h31 = H.at<double>(2, 0); double h32 = H.at<double>(2, 1); double h33 = 1;
	
	double lamda = h31 * P_pixel.x + h32 * P_pixel.y + 1;

	Point2f P_World;
	P_World.x = (h11 * P_pixel.x + h12 * P_pixel.y + h13)/ lamda;
	P_World.y = (h21 * P_pixel.x + h22 * P_pixel.y + h23)/ lamda;

	return P_World;//世界坐标系下的坐标
}

void resample_points(double* pts_in1, double* pts_in2, int num_in, double* pts_out1, double* pts_out2, int* num_out, double dist)
{
	if (num_in < 0) 
	{
		*num_out = 0;
		return;
	}
	pts_out1[0] = pts_in1[0];
	pts_out2[0] = pts_in2[0];
	int len = 1;
	for (int i = 0; i < num_in - 1 && len < *num_out; i++) 
	{
		float x0 = pts_in1[i];
		float y0 = pts_in2[i];
		float x1 = pts_in1[i + 1];
		float y1 = pts_in2[i + 1];

		do {
			float x = pts_out1[len - 1];
			float y = pts_out2[len - 1];

			float dx0 = x0 - x;
			float dy0 = y0 - y;
			float dx1 = x1 - x;
			float dy1 = y1 - y;

			float dist0 = sqrt(dx0 * dx0 + dy0 * dy0);
			float dist1 = sqrt(dx1 * dx1 + dy1 * dy1);

			float r0 = (dist1 - dist) / (dist1 - dist0);
			float r1 = 1 - r0;

			if (r0 < 0 || r1 < 0) break;
			x0 = x0 * r0 + x1 * r1;
			y0 = y0 * r0 + y1 * r1;
			pts_out1[len] = x0;
			pts_out2[len] = y0;
			len++;
		} while (len < *num_out);

	}
	*num_out = len;
}
