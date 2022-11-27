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
			temp = input_Array.at<uint8_t>(Point2d(x + i, y + j)); //�������ֵ
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
	target.points(rec_point);//4�����������
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
		cout << "��ȡ��Ӧ����ʧ��" << endl;
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
	/* ע�⣺�����ַ�ʽ��Input_mtx�������ڵ�ǰ�Ļ����Ͻ��е�,�����,����ÿ���������ȱ�ɵ�λ�� */
	Inputmtx.rotate(R); // ����ratation_vector������ת
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
	arrowedLine(img, pt_1, pt_2, Scalar(255, 255, 255), 2);//����Y���ͷ
	//putText(img, "Y", pt_1, FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 2);

	Point pt_3 = Point2d(rect_center.x - offset, rect_center.y + offset / k);
	Point pt_4 = Point2d(rect_center.x + offset, rect_center.y - offset / k);
	arrowedLine(img, pt_3, pt_4, Scalar(255, 255, 255), 2);//����X���ͷ
	//putText(img, "X", pt_3, FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 2);
}

int findBlobs(Mat& blob_img, vector<Blob>& blobs)
{
	clock_t start = clock();

	Mat src, src_HSV, ret;
	resize(blob_img, src, Size(640, 480));

	/* 1.ȥ�������С�� */
	cvtColor(src, src_HSV, COLOR_BGR2HSV);//ת��HSV�ռ䷽����ɫʶ��
	Scalar L_Thresh(0, 120, 30);
	Scalar H_Thresh(179, 255, 255);
	inRange(src_HSV, L_Thresh, H_Thresh, ret);//������ɫ��ֵ�ָ�ͼ��
 
	/* 2.��̬ѧ���� */
	Mat kernel = getStructuringElement(MORPH_RECT, Size(7,7), Point(-1, -1));
	morphologyEx(ret, ret, MORPH_OPEN, kernel);
	

	/* 3.�������� */
	vector<vector<Point>> counters;
	vector<Vec4i> Hierarchy;
	findContours(ret, counters, Hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));
	cout << "������Ϊ:" << counters.size() << endl;

	/* 4.�������ȷ�������м���ɫ�� */
	vector<vector<Point>> Blob_Counters;//ɫ�������
	vector<Mat> SingleBlob_Images;//��������ÿ��ͼֻ��һ��������ɫ��
	vector<Rect> Rcets;

	for (int i = 0; i < counters.size(); i++)
	{
		double area = contourArea(counters[static_cast<int>(i)]);
		cout << "��" << i << "�������������" << area << endl;
		if (area > 500 && area < 4500)
		{
			SingleBlob_Images.push_back(Mat::zeros(src.rows, src.cols, CV_8UC3));
			drawContours(SingleBlob_Images[Blob_Counters.size()], counters, i, Scalar(255, 255, 255), -1, 8, Hierarchy, 0);//��ֵͼ�����
			Blob_Counters.push_back(counters[i]);
			Rcets.push_back(boundingRect(*(Blob_Counters.end()-1) ));

			//drawContours(src, counters, i, Scalar(0, 255, 255), 1, 8, Hierarchy, 0);//��ԭͼ�ϻ�������
		}
	}
	cout << "ɫ����Ϊ:" << Blob_Counters.size() << endl;

	/* 4.��ÿһ��ɫ�������Ƚ�һ���ָ� */
	vector<Mat> HSV_Images;
	counters.clear();
	Hierarchy.clear();
	Mat sum_Image = Mat::zeros(src.rows, src.cols, CV_8UC3);//�ָ�����Ч��ͼ
	Mat binary = Mat::zeros(src.rows, src.cols, CV_8UC1);

	for (int i = 0; i < Blob_Counters.size(); i++)
	{
		Mat mask = Mat::zeros(src.rows, src.cols, CV_8UC1);
		HSV_Images.push_back(src_HSV.clone() & SingleBlob_Images[i]);	//��HSVͼ�Ͽ۳���i��ɫ��

		vector<Mat> channels;
		split(HSV_Images[i], channels);
		/* �����Ƚ��зָ� */
		Mat Value = channels[2];
		Mat img_loc;//�Ҷ�ͼ
		Value(Rcets[i]).copyTo(img_loc);
		double value_thresh = threshold(img_loc, img_loc, 0, 255, THRESH_OTSU);
		//cout << "value_thresh1 = " << value_thresh << endl;
		Mat Value_Binary;

		double value_max = Get_MaxValue(Value, Rcets[i]);
		//cout << "value_max = " << value_max << endl;
		if (value_max - value_thresh > 30) //ѡ������ʵ���ֵ
			value_thresh = MAX((value_max - 40), (value_thresh + (value_max - value_thresh)/2) );

		//cout << "value_thresh2 = " << value_thresh << endl;
		threshold(Value, Value_Binary, value_thresh, 255, THRESH_BINARY);
		morphologyEx(Value_Binary, Value_Binary, MORPH_OPEN, getStructuringElement(MORPH_RECT, Size(9, 9)));
		
		binary = binary | Value_Binary;//�ܵĶ�ֵͼ
	}

	sum_Image = convertTo3Channels(binary) & src;//��ԭͼ�Ͽ۳�����ɫ��

	/* 5.����ÿ��ɫ�������,ȷ�������ļ����� */
	int num = 0;
	Point Center_Pixel;
	double k; //б��
	Point2f point_line;//��������һ��
	counters.clear();
	Hierarchy.clear();

	findContours(binary, counters, Hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));
	cout << "num of counters:" << counters.size() << endl;
	for (int i = 0; i < counters.size(); i++)
	{
		RotatedRect rect_now = minAreaRect(counters[i]);//��ÿ����������С��Ӿ���
		if (rect_now.boundingRect2f().area() > 40)//�����������˵�� ��������ɫ�������
		{
			++num;
			/* ����������ɫƽ��ֵ��ȷ����ɫ */
			Color color = Get_Hue(src_HSV, rect_now);

			Center_Pixel = draw_rotrect(src, rect_now, Scalar(255, 255, 255));//������ͼƬ�ϻ���б�߾��ο�
			k = tanf(rect_now.angle * 3.14 / 180);//��������б��
			point_line = Point2d(rect_now.center.x + (30 / k), rect_now.center.y + 30);//����ֱ���ϵ�һ��
			
			/* ��ɫ������д�������� */
			Blob temp(rect_now.center, point_line, 0, color);
			blobs.push_back(temp);

			/* �����ͼ�ϱ�ע��Ϣ */
			Text_Annotate(src, rect_now.center, color, num);
			Line_Annotate(src, (Point)rect_now.center, k);//���Ƽ�ͷ
		}
	}
	cout << "num of rect:" << num << endl;
	imshow("���ս��", src);

	clock_t end = clock();
	cout << "����ʱ�� = " << (end - start) << "ms" << endl;

	cout << "�밴�ո��������һ����" << endl;
	waitKey(0);
	return num;//����ɫ�����
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

	return P_World;//��������ϵ�µ�����
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
