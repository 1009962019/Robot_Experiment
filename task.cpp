#include "task.h"

/**
 * @brief ��е�����۱궨,��Ҫ�ֶ�����4���,�������homo_matrix.xml��
 * @param
*/
void Eye2Hand_Calibration(void)
{
	vector<Point2f> points_pixel{ Point2f(255,268),Point2f(272,68),
						  Point2f(428,90),Point2f(365,224) };//pix
	vector<Point2f> points_world{ Point2f(204,331),Point2f(287,238),
								  Point2f(212,188),Point2f(182,276) };//mm
	saveHomog(points_pixel, points_world, "homo_matrix.xml");
	cout << "���۱궨��ɣ�" << endl;
}


/**
 * @brief ����CVMove����
 * @param  
*/
//void move_test(void)
//{
//	double jointpos1[6], jointpos2[6];
//	JAKAZuRobot jakaRob;
//	Robot_Init(jakaRob);
//	Robot_Print_Joints(jakaRob);
//
//	//�����ǰ���ƶ�����鴦
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
//	jakaRob.login_out();//����
//}

/**
 * @brief ������ͷ����һ֡ͼ��
 * @param ���ո�ȡ�����βɼ�����C���ɼ���ǰ֡������
*/
bool PhotoInput(void)
{
	//�ӵ�������ͷ����
	VideoCapture capture(1);
	if (capture.isOpened())
	{
		Mat frame;
		const string pic_path = "./blob/kk.jpg";
		while (1)
		{
			capture >> frame;
			resize(frame, frame, Size(640, 480));//������Ƭ�ߴ�
			imshow("PhotoInput", frame);
			char key = waitKey(10);//10msɨ��һ�ΰ���
			if (key == ' ')
				break;
			else if (key == 'c')
			{
				imwrite(pic_path, frame);//���resize��,����ߴ���reszie���
				cout << "�ɼ������" << endl;
				break;
			}
		}
		capture.release();
		return 1;
	}
	else
	{
		cout << "�����ʧ��\n";
		return 0;
	}

}

/**
 * @brief ������ץȡDemo
 * @param  
*/
//void WorkDemo(void)
//{
//	/* ��е�۳�ʼ�� */
//	JAKAZuRobot jakaRob;
//	//Robot_Init(jakaRob);
//
//	/* ͼ��ɼ��ʹ��� */
//	//PhotoInput();
//	Mat Img_Input = imread("./blob/PhotoInput.jpg");
//	Mat mHomog = loadHomog("homo_matrix.xml");
//	vector<Blob> my_blobs;
//	findBlobs(Img_Input, my_blobs);
//
//	/* ץȡ */
//	for (auto it = my_blobs.begin(); it < my_blobs.end(); ++it)
//	{
//		cout << (*it);
//		/* ���ƶ���ɫ�鴦 */
//		(*it).InfoChange(mHomog);
//		cout << (*it);
//		double end[6] = {(*it).m_center.x,(*it).m_center.y,BLOB_HEIGHT,-180,0,(*it).m_angle};
//		int ret = jakaRob.moveP(end, 5.0);
//		cout << "movep: " << ret << endl;
//		jakaRob.waitEndMove();
//
//		/* ��CVMOVE */
//		(*it).JakaMove(400, jakaRob);
//		jakaRob.waitEndMove();
//	}
//
//	jakaRob.login_out();//����
//}