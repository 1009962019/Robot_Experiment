#ifndef _TASK_H_
#define _TASK_H_

#include <iostream>
#include "opencv2/opencv.hpp"
#include "Eigen/Dense"
#include "JAKARobot_API.h"
#include "Blob.h"
#include "subFunctions.h"

/**
 * @brief 机械臂手眼标定,需要手动输入4组点,结果存入homo_matrix.xml中
 * @param
*/
void Eye2Hand_Calibration(void);

/**
 * @brief 测试CVMove函数
 * @param
*/
//void move_test(void);

/**
 * @brief 由摄像头输入一帧图像
 * @param  按空格取消本次采集、按C键采集当前帧并保存
 * @return 0:失败
*/
bool PhotoInput(void);

/**
 * @brief 完整的抓取Demo
 * @param
*/
//void WorkDemo(void);
#endif