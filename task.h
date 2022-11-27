#ifndef _TASK_H_
#define _TASK_H_

#include <iostream>
#include "opencv2/opencv.hpp"
#include "Eigen/Dense"
#include "JAKARobot_API.h"
#include "Blob.h"
#include "subFunctions.h"

/**
 * @brief ��е�����۱궨,��Ҫ�ֶ�����4���,�������homo_matrix.xml��
 * @param
*/
void Eye2Hand_Calibration(void);

/**
 * @brief ����CVMove����
 * @param
*/
//void move_test(void);

/**
 * @brief ������ͷ����һ֡ͼ��
 * @param  ���ո�ȡ�����βɼ�����C���ɼ���ǰ֡������
 * @return 0:ʧ��
*/
bool PhotoInput(void);

/**
 * @brief ������ץȡDemo
 * @param
*/
//void WorkDemo(void);
#endif