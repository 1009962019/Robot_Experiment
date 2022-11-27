#include "task.h"

/**
 * @brief 机械臂手眼标定,需要手动输入4组点,结果存入homo_matrix.xml中
 * @param
*/
void Eye2Hand_Calibration(void)
{
	vector<Point2f> points_pixel{ Point2f(255,268),Point2f(272,68),
						  Point2f(428,90),Point2f(365,224) };//pix
	vector<Point2f> points_world{ Point2f(204,331),Point2f(287,238),
								  Point2f(212,188),Point2f(182,276) };//mm
	saveHomog(points_pixel, points_world, "homo_matrix.xml");
	cout << "手眼标定完成！" << endl;
}


/**
 * @brief 测试CVMove函数
 * @param  
*/
//void move_test(void)
//{
//	double jointpos1[6], jointpos2[6];
//	JAKAZuRobot jakaRob;
//	Robot_Init(jakaRob);
//	Robot_Print_Joints(jakaRob);
//
//	//搬物块前先移动到物块处
//	double end[6] = { 140,250,BLOB_HEIGHT,-180,0,0 };
//	int ret = jakaRob.moveP(end,8.0);
//	cout << "movep:" << ret << endl;
//	jakaRob.waitEndMove();
//
//	Blob b1(Point2f(140, 250));
//	b1.m_color = _blue;
//
//	b1.JakaMove(400, jakaRob);
//	jakaRob.waitEndMove();
//
//	jakaRob.login_out();//结束
//}

/**
 * @brief 由摄像头输入一帧图像
 * @param 按空格取消本次采集、按C键采集当前帧并保存
*/
bool PhotoInput(void)
{
	//从电脑摄像头读入
	VideoCapture capture(1);
	if (capture.isOpened())
	{
		Mat frame;
		const string pic_path = "./blob/kk.jpg";
		while (1)
		{
			capture >> frame;
			resize(frame, frame, Size(640, 480));//调整照片尺寸
			imshow("PhotoInput", frame);
			char key = waitKey(10);//10ms扫描一次按键
			if (key == ' ')
				break;
			else if (key == 'c')
			{
				imwrite(pic_path, frame);//如果resize了,这里尺寸是reszie后的
				cout << "采集完成了" << endl;
				break;
			}
		}
		capture.release();
		return 1;
	}
	else
	{
		cout << "相机打开失败\n";
		return 0;
	}

}

/**
 * @brief 完整的抓取Demo
 * @param  
*/
//void WorkDemo(void)
//{
//	/* 机械臂初始化 */
//	JAKAZuRobot jakaRob;
//	//Robot_Init(jakaRob);
//
//	/* 图像采集和处理 */
//	//PhotoInput();
//	Mat Img_Input = imread("./blob/PhotoInput.jpg");
//	Mat mHomog = loadHomog("homo_matrix.xml");
//	vector<Blob> my_blobs;
//	findBlobs(Img_Input, my_blobs);
//
//	/* 抓取 */
//	for (auto it = my_blobs.begin(); it < my_blobs.end(); ++it)
//	{
//		cout << (*it);
//		/* 先移动到色块处 */
//		(*it).InfoChange(mHomog);
//		cout << (*it);
//		double end[6] = {(*it).m_center.x,(*it).m_center.y,BLOB_HEIGHT,-180,0,(*it).m_angle};
//		int ret = jakaRob.moveP(end, 5.0);
//		cout << "movep: " << ret << endl;
//		jakaRob.waitEndMove();
//
//		/* 用CVMOVE */
//		(*it).JakaMove(400, jakaRob);
//		jakaRob.waitEndMove();
//	}
//
//	jakaRob.login_out();//结束
//}