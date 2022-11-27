#include "stdafx.h"
#include <iostream>
#include "Blob.h"
#include "JAKARobot_API.h"
#include <windows.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include "subFunctions.h"

using namespace std;
using namespace Eigen;
using namespace cv;

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#elif defined(_MSC_VER)
#pragma warning(disable : 4996)
#endif
#pragma comment(lib, "ws2_32.lib")

#define  BUFF_SIZE	512
#define  MAX_BUFF_SIZE	2048			//��ȡ�Ļ�����������������Ҫ���ڴ��С��ʵ��ֵ��1500����

#ifndef M_PI
#define  M_PI 3.141516925
#endif

int Robot_Init(JAKAZuRobot& jakaRob)
{
	int is_enable = 0, is_powered = 0;
	int ret = jakaRob.login_in("192.168.1.100");
	if (ret != NO_ERROR)
	{
		printf("%s\n", jakaRob.m_errMsg);//��û���ӳɹ�,��ӡ����
		return 1;
	}
	else
	{
		jakaRob.getState(is_enable, is_powered);//���ʹ�ܺ͵�Դ״̬
		if (!is_powered)
		{
			ret = jakaRob.power(1);
			Sleep(6);
		}
		if (!is_enable)
		{
			ret = jakaRob.enable(1);
			Sleep(6);
		}
		jakaRob.initSetting();//��ʼ���趨ΪĬ��ֵ
		Sleep(10);
		return 1;
	}
}

void Robot_Print_Joints(JAKAZuRobot& jakaRob)
{
	double jointpos2[6];
	jakaRob.getJoints(jointpos2);
	cout << "joint1:" << jointpos2[0] << endl;
	cout << "joint2:" << jointpos2[1] << endl;
	cout << "joint3:" << jointpos2[2] << endl;
	cout << "joint4:" << jointpos2[3] << endl;
	cout << "joint5:" << jointpos2[4] << endl;
	cout << "joint6:" << jointpos2[5] << endl;
}

void Robot_Print_Pose(JAKAZuRobot& jakaRob)
{
	double jointpos2[6];
	jakaRob.getJoints(jointpos2);
	jakaRob.getTcpPose(jointpos2);
	//cout << "joint1:" << jointpos2[0] << endl;
	//cout << "joint2:" << jointpos2[1] << endl;
	//cout << "joint3:" << jointpos2[2] << endl;
	//cout << "joint4:" << jointpos2[3] << endl;
	//cout << "joint5:" << jointpos2[4] << endl;
	//cout << "joint6:" << jointpos2[5] << endl;
}

//�����������������
enum TEXTYPE { STRING, NUMBER, ARRAY };

/* @brief ���ҷ�����Ϣ�ж�Ӧ�ֶε�����
* @param recvInfo ������Ϣ
* @param fieldName Ҫ���ҵ��ֶ����ƺõ�
* @param fieldText ������ֶ�����
* @param txtType �ֶ��������ͣ�0���ַ�����1������2������
* @return 1�� �ɹ���0��ʧ��
*/
//JSON ֵ�����ǣ����֣������򸡵������ַ�������˫�����У��߼�ֵ��true �� false�����飨�ڷ������У������ڻ������У� null
int findFieldText(char *recvInfo, char *fieldName, char *fieldText, int txtType)
{
	int num = 0;
	char *ptr = strstr(recvInfo, fieldName);
	if (ptr != NULL)
	{
		switch (txtType)
		{
		case STRING:
			num = sscanf(ptr, "%*[^:]%*[^\"]\"%[^\"]", fieldText);
			break;
		case NUMBER:
			num = sscanf(ptr, "%*[^:]:%[^,]", fieldText);
			if (strchr(fieldText, '\"'))		//���������͵��ֶ����ݰ�������
			{
				int n = strlen(fieldText);
				memcpy(fieldText, fieldText + 1, n - 2);
				fieldText[n - 2] = '\0';
			}
			break;
		case  ARRAY:
			num = sscanf(ptr, "%*[^:]%*[^[][%[^]]", fieldText);
			break;
		default:
			break;
		}
	}

	return num;
}

JAKAZuRobot::JAKAZuRobot(){}

int JAKAZuRobot::login_in(const char* ip)
{
	WORD wVersinRequested;
	WSADATA wsaData;
	int err;
	wVersinRequested = MAKEWORD(2, 2);
	err = WSAStartup(wVersinRequested, &wsaData);
	if (err != 0) return ROBOT_ERROR; 
	if (LOBYTE(wsaData.wVersion) != 2 || HIBYTE(wsaData.wVersion) != 2)
	{
		WSACleanup();		
		return ROBOT_ERROR;
	}
	SOCKADDR_IN addrSrv;//�趨�������˵�IP�Ͷ˿� 	
	addrSrv.sin_addr.s_addr = inet_addr(ip);//���ػ�·��ַ	
	addrSrv.sin_family = AF_INET;
	addrSrv.sin_port = htons(10001);
	m_socketClient = socket(AF_INET, SOCK_STREAM, 0);//���������������Ϊ0�������Զ�ѡ��Э�顣
	if (m_socketClient == INVALID_SOCKET)
		sprintf(m_errMsg, "socket() called failed!, error code is %d", WSAGetLastError());
	int u = connect(m_socketClient, (SOCKADDR*)&addrSrv, sizeof(SOCKADDR));//��������������ӡ�	Sleep(2000);
	if (u == SOCKET_ERROR)
	{
		sprintf(m_errMsg, "connect() called failed!, error code is %d", WSAGetLastError());
		closesocket(m_socketClient);
		return ROBOT_ERROR;
	}

	//10000�˿ڵ�����
	addrSrv.sin_port = htons(10000);
	m_socketData = socket(AF_INET, SOCK_STREAM, 0);//���������������Ϊ0�������Զ�ѡ��Э�顣

	 //���û�������С������Ҫ�洢�ܶ����ݣ�ʵ�ʴ�С�����õ�2��
	int rcvcbuf_len = MAX_BUFF_SIZE;
	int len = sizeof(rcvcbuf_len);
	setsockopt(m_socketData, SOL_SOCKET, SO_RCVBUF, (char *)&rcvcbuf_len, len); 

	u = connect(m_socketData, (SOCKADDR*)&addrSrv, sizeof(SOCKADDR));//��������������ӡ�	Sleep(2000);
	if (u == SOCKET_ERROR)
	{
		sprintf(m_errMsg, "connect() called failed!, error code is %d", WSAGetLastError());
		closesocket(m_socketData);
		return ROBOT_ERROR;
	}

	return NO_ERROR;
}

int JAKAZuRobot::login_out()
{
	char sendCmd[BUFF_SIZE], recvBuff[BUFF_SIZE];
	strcpy(sendCmd, "{\"cmdName\":\"quit\"}");

	int rtn = send(m_socketClient, sendCmd, strlen(sendCmd), 0);
	rtn = recv(m_socketClient, recvBuff, 100, 0);
	if (rtn == SOCKET_ERROR)	return ROBOT_ERROR;

	int isOK = findFieldText(recvBuff, "errorCode", m_errMsg, NUMBER);
	rtn = atoi(m_errMsg);
	isOK = findFieldText(recvBuff, "errorMsg", m_errMsg);

	//�ر��׽���
	closesocket(m_socketClient);
	closesocket(m_socketData);
	//��ֹ�׽��ֿ��ʹ��
	WSACleanup();

	return  rtn;
}

int JAKAZuRobot::power(int isOn)
{
	char sendCmd[BUFF_SIZE], recvBuff[BUFF_SIZE];
	if (isOn > 0)
		strcpy(sendCmd, "{\"cmdName\":\"power_on\"}");
	else
		strcpy(sendCmd, "{\"cmdName\":\"power_off\"}");

	int rtn = send(m_socketClient, sendCmd, strlen(sendCmd), 0);
	//Sleep(6);
	rtn = recv(m_socketClient, recvBuff, BUFF_SIZE, 0);
	if (rtn == SOCKET_ERROR)	return ROBOT_ERROR;
	//Sleep(6);

	int isOK = findFieldText(recvBuff, "errorMsg", m_errMsg);
	char errs[8];
	isOK = findFieldText(recvBuff, "errorCode", errs, NUMBER);
	return   atoi(errs);
}

int JAKAZuRobot::enable(int isEnable)
{
	char sendCmd[BUFF_SIZE], recvBuff[BUFF_SIZE];
	if (isEnable > 0)
		strcpy(sendCmd, "{\"cmdName\":\"enable_robot\"}");
	else
		strcpy(sendCmd, "{\"cmdName\":\"disable_robot\"}");

	int rtn = send(m_socketClient, sendCmd, strlen(sendCmd), 0);
	rtn = recv(m_socketClient, recvBuff, BUFF_SIZE, 0);
	if (rtn == SOCKET_ERROR)	return ROBOT_ERROR;

	int isOK = findFieldText(recvBuff, "errorMsg", m_errMsg);
	char errs[8];
	isOK = findFieldText(recvBuff, "errorCode", errs, NUMBER);
	return   atoi(errs);
}

int JAKAZuRobot::getState(int &is_enable, int &is_powered)
{
	char sendCmd[] = "{\"cmdName\":\"get_robot_state\"}";
	char recvBuff[BUFF_SIZE];

	int rtn = send(m_socketClient, sendCmd, strlen(sendCmd), 0);
	rtn = recv(m_socketClient, recvBuff, BUFF_SIZE, 0);
	if (rtn == SOCKET_ERROR)	return ROBOT_ERROR;

	char errs[8];
	int isOK = findFieldText(recvBuff, "errorCode", errs, NUMBER);
	rtn = atoi(errs);
	if (rtn) 
	{
		isOK = findFieldText(recvBuff, "errorMsg", m_errMsg);
		return rtn;
	}

	char fieldText[BUFF_SIZE];
	isOK = findFieldText(recvBuff, "enable", fieldText);
	if (strstr(fieldText, "robot_enabled"))		is_enable = 1;
	else is_enable = 0;

	isOK = findFieldText(recvBuff, "power", fieldText);
	if (strstr(fieldText, "powered_on"))		is_powered = 1;
	else is_powered = 0;

	return  rtn;
}

int JAKAZuRobot::initSetting()
{
	int rtn;

	rtn = clearError();
	if (rtn) return rtn;

	rtn = setToolID(0);
	if (rtn) return rtn;

	rtn = setUserID(0);
	if (rtn) return rtn;

	double offset[3] = { 0.0f };
	rtn = setPayload(0.0, offset);
	if (rtn) return rtn;

	rtn = setRapidRate(1.0);
	if (rtn) return rtn;

	return NO_ERROR;
}

int JAKAZuRobot::clearError()
{
	int rtn;

	//�������
	char sendCmd[BUFF_SIZE] = "{\"cmdName\":\"clear_error\"}";
	char recvBuff[BUFF_SIZE];
	rtn = send(m_socketClient, sendCmd, strlen(sendCmd), 0);
	rtn = recv(m_socketClient, recvBuff, MAX_BUFF_SIZE, 0);

	if (rtn == SOCKET_ERROR)	return ROBOT_ERROR;
	else return ROBOT_NOERR;
}

int JAKAZuRobot::getData(char *all_data, int port)
{
	char sendCmd[] = "{\"cmdName\":\"get_data\"}";

	int rtn = send(m_socketClient, sendCmd, strlen(sendCmd), 0);
	rtn = recv(m_socketClient, all_data, MAX_BUFF_SIZE, 0);
	if (rtn == SOCKET_ERROR)	return ROBOT_ERROR;

	int isOK = findFieldText(all_data, "errorMsg", m_errMsg);
	char errs[8];
	isOK = findFieldText(all_data, "errorCode", errs, NUMBER);
	return   atoi(errs);
}

int JAKAZuRobot::getJoints(double *joint_angles, char *data_info /* = NULL */)
{
	char jAngles[BUFF_SIZE];

	int rtn = 0;
	if (data_info == NULL)
	{
		char recvBuff[BUFF_SIZE];
		char sendCmd[] = "{\"cmdName\":\"get_joint_pos\"}";
		rtn = send(m_socketClient, sendCmd, strlen(sendCmd), 0);
		rtn = recv(m_socketClient, recvBuff, BUFF_SIZE, 0);

		int isOK = findFieldText(recvBuff, "errorCode", m_errMsg, NUMBER);
		rtn = atoi(m_errMsg);
		if (rtn != 0)
		{
			isOK = findFieldText(recvBuff, "errorMsg", m_errMsg);
			return rtn;
		}

		isOK = findFieldText(recvBuff, "joint_pos", jAngles, ARRAY);
		isOK = sscanf(jAngles, "%lf,%lf,%lf,%lf,%lf,%lf",
			joint_angles, joint_angles+1, joint_angles+2, joint_angles+3, joint_angles+4, joint_angles+5);
	}
	else
	{
		int isOK = findFieldText(data_info, "joint_actual_position", jAngles, ARRAY);
		isOK = sscanf(jAngles, "%lf,%lf,%lf,%lf,%lf,%lf",
			joint_angles, joint_angles + 1, joint_angles + 2, joint_angles + 3, joint_angles + 4, joint_angles + 5);
	}

	return  rtn;
}

int JAKAZuRobot::getTcpPose(double *tcp_pose, char *data_info /* = NULL */)
{
	char poses[BUFF_SIZE];

	int rtn = 0;
	if (data_info == NULL)
	{
		char recvBuff[BUFF_SIZE];
		char sendCmd[] = "{\"cmdName\":\"get_tcp_pos\"}";
		rtn = send(m_socketClient, sendCmd, strlen(sendCmd), 0);
		rtn = recv(m_socketClient, recvBuff, BUFF_SIZE, 0);

		int isOK = findFieldText(recvBuff, "errorCode", m_errMsg, NUMBER);
		rtn = atoi(m_errMsg);
		if (rtn != 0)
		{
			isOK = findFieldText(recvBuff, "errorMsg", m_errMsg);
			return rtn;
		}

		isOK = findFieldText(recvBuff, "tcp_pos", poses, ARRAY);
		isOK = sscanf(poses, "%lf,%lf,%lf,%lf,%lf,%lf",
			tcp_pose, tcp_pose + 1, tcp_pose + 2, tcp_pose + 3, tcp_pose + 4, tcp_pose + 5);
	}
	else
	{
		int isOK = findFieldText(data_info, "\"actual_position\"", poses, ARRAY);
		isOK = sscanf(poses, "%lf,%lf,%lf,%lf,%lf,%lf",
			tcp_pose, tcp_pose + 1, tcp_pose + 2, tcp_pose + 3, tcp_pose + 4, tcp_pose + 5);
	}

	return  rtn;
}

int JAKAZuRobot::getDin(int &dio_in, int IOType /* = IO_CABINET */, char *data_info /* = NULL */)
{
	int rtn = 0, isOK;
	char strDins[BUFF_SIZE];
	dio_in = 0;

	if (IOType == IO_CABINET)
	{
		if (data_info == NULL)
		{
			char recvBuff[BUFF_SIZE];
			char sendCmd[] = "{\"cmdName\":\"get_digital_input_status\"}";
			int rtn = send(m_socketClient, sendCmd, strlen(sendCmd), 0);
			rtn = recv(m_socketClient, recvBuff, BUFF_SIZE, 0);

			isOK = findFieldText(recvBuff, "errorCode", m_errMsg, NUMBER);
			rtn = atoi(m_errMsg);
			if (rtn != 0)
			{
				isOK = findFieldText(recvBuff, "errorMsg", m_errMsg);
				return rtn;
			}

			isOK = findFieldText(recvBuff, "din_status", strDins, ARRAY);
		}
		else
		{
			isOK = findFieldText(data_info, "\"din\"", strDins, ARRAY);	//din�ֶ����ظ�
		}

		int ins[8] = { 0 };
		isOK = sscanf(strDins, "%d,%d,%d,%d,%d,%d,%d,%d,%*s",
			ins, ins + 1, ins + 2, ins + 3, ins + 4, ins + 5, ins + 6, ins + 7);	//ֻ��ȡǰ8��ֵ
		for (int i = 0; i < 8; i++)
		{
			dio_in |= (ins[i] << i);
		}
	}
	else
	{
		if (data_info == NULL)
		{
			char recvBuff[MAX_BUFF_SIZE];
			char sendCmd[] = "{\"cmdName\":\"get_data\"}";		//û�е������TOOL_IO��ָ��

			rtn = send(m_socketClient, sendCmd, strlen(sendCmd), 0);
			rtn = recv(m_socketClient, recvBuff, MAX_BUFF_SIZE, 0);

			isOK = findFieldText(recvBuff, "errorCode", m_errMsg, NUMBER);
			rtn = atoi(m_errMsg);
			if (rtn != 0)
			{
				isOK = findFieldText(recvBuff, "errorMsg", m_errMsg);
				return rtn;
			}

			isOK = findFieldText(recvBuff, "tio_din", strDins, ARRAY);
		}
		else
		{
			isOK = findFieldText(data_info, "tio_din", strDins, ARRAY);
		}

		int ins[2] = { 0 };
		isOK = sscanf(strDins, "%d,%d,%*s", ins, ins + 1);	//ֻ��ȡǰ8��ֵ
		dio_in = ins[0] + (ins[1] << 1);
	}

	return  rtn;
}

int JAKAZuRobot::setDout(int index, int value /* = 1 */, int IOType /* = IO_CABINET */)
{
	char sendCmd[BUFF_SIZE];
	sprintf(sendCmd, "{\"cmdName\":\"set_digital_output\",\"type\":%d,\"index\":%d,\"value\":%d}", 
		IOType, index, value);

	char recvBuff[BUFF_SIZE];
	int rtn = send(m_socketClient, sendCmd, strlen(sendCmd), 0);
	rtn = recv(m_socketClient, recvBuff, MAX_BUFF_SIZE, 0);

	int isOK = findFieldText(recvBuff, "errorMsg", m_errMsg);
	char errs[8];
	isOK = findFieldText(recvBuff, "errorCode", errs, NUMBER);
	return atoi(errs);
}

int JAKAZuRobot::addTool(double *tool_offset, char *tool_name, int tool_id /* = 1 */)
{
	char sendCmd[BUFF_SIZE]= "{\"cmdName\":\"set_tool_offsets\",\"tooloffset\":";
	char strParams[BUFF_SIZE];

	sprintf(strParams, "[%f,%f,%f,%f,%f,%f],\"id\":%d,\"name\":\"%s\"}",
		tool_offset[0], tool_offset[1], tool_offset[2], tool_offset[3], tool_offset[4], tool_offset[5], tool_id, tool_name);
	strcat(sendCmd, strParams);

	int rtn = send(m_socketClient, sendCmd, strlen(sendCmd), 0);
	if (rtn == SOCKET_ERROR)	return ROBOT_ERROR;

	char recvBuff[BUFF_SIZE];
	rtn = recv(m_socketClient, recvBuff, BUFF_SIZE, 0);
	if (rtn == SOCKET_ERROR)	return ROBOT_ERROR;

	int isOK = findFieldText(recvBuff, "errorMsg", m_errMsg);
	char errs[8];
	isOK = findFieldText(recvBuff, "errorCode", errs, NUMBER);
	return atoi(errs);
}

int JAKAZuRobot::setToolID(int tool_id /* = 0 */)
{
	char sendCmd[BUFF_SIZE];
	sprintf(sendCmd, "{\"cmdName\":\"set_tool_id\",\"tool_id\":%d}", tool_id);

	char recvBuff[BUFF_SIZE];
	int rtn = send(m_socketClient, sendCmd, strlen(sendCmd), 0);
	rtn = recv(m_socketClient, recvBuff, MAX_BUFF_SIZE, 0);

	int isOK = findFieldText(recvBuff, "errorMsg", m_errMsg);
	char errs[8];
	isOK = findFieldText(recvBuff, "errorCode", errs, NUMBER);
	return atoi(errs);
}

int JAKAZuRobot::getToolID(int &tool_id)
{
	char recvBuff[MAX_BUFF_SIZE];
	char sendCmd[] = "{\"cmdName\":\"get_data\"}";		//û�е������get_tool_id��ָ��

	int rtn = send(m_socketClient, sendCmd, strlen(sendCmd), 0);
	rtn = recv(m_socketClient, recvBuff, MAX_BUFF_SIZE, 0);

	int isOK = findFieldText(recvBuff, "errorCode", m_errMsg, NUMBER);
	rtn = atoi(m_errMsg);
	if (rtn != 0)
	{
		isOK = findFieldText(recvBuff, "errorMsg", m_errMsg);
	}
	else
	{
		char ids[8];
		isOK = findFieldText(recvBuff, "current_tool_id", ids, NUMBER);
		tool_id = atoi(ids);
	}

	return rtn;
}

int JAKAZuRobot::addUserCord(double *user_offset, char *user_name, int user_id /* = 1 */)
{
	char sendCmd[BUFF_SIZE] = "{\"cmdName\":\"set_user_offsets\",\"userffset\":";
	char strParams[BUFF_SIZE];

	sprintf(strParams, "[%f,%f,%f,%f,%f,%f],\"id\":%d,\"name\":\"%s\"}",
		user_offset[0], user_offset[1], user_offset[2], user_offset[3], user_offset[4], user_offset[5], user_id, user_name);
	strcat(sendCmd, strParams);

	int rtn = send(m_socketClient, sendCmd, strlen(sendCmd), 0);
	if (rtn == SOCKET_ERROR)	return ROBOT_ERROR;

	char recvBuff[BUFF_SIZE];
	rtn = recv(m_socketClient, recvBuff, BUFF_SIZE, 0);
	if (rtn == SOCKET_ERROR)	return ROBOT_ERROR;

	int isOK = findFieldText(recvBuff, "errorMsg", m_errMsg);
	char errs[8];
	isOK = findFieldText(recvBuff, "errorCode", errs, NUMBER);
	return atoi(errs);
}

int JAKAZuRobot::setUserID(int user_id /* = 0 */)
{
	char sendCmd[BUFF_SIZE];
	sprintf(sendCmd, "{\"cmdName\":\"set_user_id\",\"user_frame_id\":%d}", user_id);

	char recvBuff[BUFF_SIZE];
	int rtn = send(m_socketClient, sendCmd, strlen(sendCmd), 0);
	rtn = recv(m_socketClient, recvBuff, MAX_BUFF_SIZE, 0);

	int isOK = findFieldText(recvBuff, "errorMsg", m_errMsg);
	char errs[8];
	isOK = findFieldText(recvBuff, "errorCode", errs, NUMBER);
	return atoi(errs);
}

int JAKAZuRobot::getUserID(int &user_id)
{
	char recvBuff[MAX_BUFF_SIZE];
	char sendCmd[] = "{\"cmdName\":\"get_data\"}";		//û�е������get_tool_id��ָ��

	int rtn = send(m_socketClient, sendCmd, strlen(sendCmd), 0);
	rtn = recv(m_socketClient, recvBuff, MAX_BUFF_SIZE, 0);

	int isOK = findFieldText(recvBuff, "errorCode", m_errMsg, NUMBER);
	rtn = atoi(m_errMsg);
	if (rtn != 0)
	{
		isOK = findFieldText(recvBuff, "errorMsg", m_errMsg);
	}
	else
	{
		char ids[8];
		isOK = findFieldText(recvBuff, "current_user_id", ids, NUMBER);
		user_id = atoi(ids);
	}

	return rtn;
}

int JAKAZuRobot::setPayload(double mass, double *offset)
{
	char sendCmd[BUFF_SIZE];
	sprintf(sendCmd, "{\"cmdName\":\"set_payload\",\"mass\":%f,\"centroid\":[%f,%f,%f]}", 
		mass, offset[0], offset[1], offset[2]);

	char recvBuff[BUFF_SIZE];
	int rtn = send(m_socketClient, sendCmd, strlen(sendCmd), 0);
	rtn = recv(m_socketClient, recvBuff, MAX_BUFF_SIZE, 0);

	int isOK = findFieldText(recvBuff, "errorMsg", m_errMsg);
	char errs[8];
	isOK = findFieldText(recvBuff, "errorCode", errs, NUMBER);
	return atoi(errs);
}

int JAKAZuRobot::getPayload(double &mass, double *offset)
{
	char sendCmd[BUFF_SIZE] = "{\"cmdName\":\"get_payload\"}";

	char recvBuff[BUFF_SIZE];
	int rtn = send(m_socketClient, sendCmd, strlen(sendCmd), 0);
	rtn = recv(m_socketClient, recvBuff, MAX_BUFF_SIZE, 0);

	int isOK = findFieldText(recvBuff, "errorCode", m_errMsg, NUMBER);
	rtn = atoi(m_errMsg);
	if (rtn != 0)
	{
		isOK = findFieldText(recvBuff, "errorMsg", m_errMsg);
	}
	else
	{
		char txts[64];
		isOK = findFieldText(recvBuff, "mass", txts, NUMBER);
		mass = atof(txts);

		isOK = findFieldText(recvBuff, "centroid", txts, ARRAY);
		isOK = sscanf(txts, "%lf, %lf, %lf", offset, offset + 1, offset + 2);
	}

	return rtn;
}

int JAKAZuRobot::setRapidRate(double rapid_rate)
{
	char sendCmd[BUFF_SIZE];
	sprintf(sendCmd, "{\"cmdName\":\"rapid_rate\",\"rate_value\":%lf}", rapid_rate);

	char recvBuff[BUFF_SIZE];
	int rtn = send(m_socketClient, sendCmd, strlen(sendCmd), 0);
	rtn = recv(m_socketClient, recvBuff, MAX_BUFF_SIZE, 0);
	if (rtn == SOCKET_ERROR)	return ROBOT_ERROR;

	int isOK = findFieldText(recvBuff, "errorMsg", m_errMsg);
	char errs[8];
	isOK = findFieldText(recvBuff, "errorCode", errs, NUMBER);
	return atoi(errs);
}

int JAKAZuRobot::moveJ(const double *joint_pos, double speed, int move_mode)
{
	char sendCmd[BUFF_SIZE] = "{\"cmdName\":\"joint_move\",\"jointPosition\":";
	char strParams[BUFF_SIZE];

	sprintf(strParams, "[%f,%f,%f,%f,%f,%f],\"speed\":%f,\"relFlag\":%d}", 
		joint_pos[0], joint_pos[1], joint_pos[2], joint_pos[3], joint_pos[4], joint_pos[5],speed, move_mode);
	strcat(sendCmd, strParams);
	
	int rtn = send(m_socketClient, sendCmd, strlen(sendCmd), 0);
	if (rtn == SOCKET_ERROR)	return ROBOT_ERROR;

	char recvBuff[BUFF_SIZE];
	rtn = recv(m_socketClient, recvBuff, BUFF_SIZE, 0);
	if (rtn == SOCKET_ERROR)	return ROBOT_ERROR;

	int isOK = findFieldText(recvBuff, "errorMsg", m_errMsg);
	char errs[8];
	isOK = findFieldText(recvBuff, "errorCode", errs, NUMBER);
	return atoi(errs);
}

int JAKAZuRobot::moveP(const double *endPos, double speed)
{
	char sendCmd[BUFF_SIZE] = "{\"cmdName\":\"end_move\",\"endPosition\":";
	char strParams[BUFF_SIZE], recvBuff[BUFF_SIZE];

	sprintf(strParams, "[%f,%f,%f,%f,%f,%f],\"speed\":%f}",
		endPos[0], endPos[1], endPos[2], endPos[3], endPos[4], endPos[5], speed);
	strcat(sendCmd, strParams);

	int rtn = send(m_socketClient, sendCmd, strlen(sendCmd), 0);		//���������ַ���
	if (rtn == SOCKET_ERROR)	return ROBOT_ERROR;

	rtn = recv(m_socketClient, recvBuff, BUFF_SIZE, 0);
	if (rtn == SOCKET_ERROR)	return ROBOT_ERROR;

	int isOK = findFieldText(recvBuff, "errorMsg", m_errMsg);
	char errs[8];
	isOK = findFieldText(recvBuff, "errorCode", errs, NUMBER);
	return atoi(errs);
}

int JAKAZuRobot::moveL(const double *endPos, double speed, int move_mode)
{
	char sendCmd[BUFF_SIZE] = "{\"cmdName\":\"moveL\",\"cartPosition\":";
	char strParams[BUFF_SIZE], recvBuff[BUFF_SIZE];

	sprintf(strParams, "[%f,%f,%f,%f,%f,%f],\"speed\":%f,\"relFlag\":%d}",
		endPos[0], endPos[1], endPos[2], endPos[3], endPos[4], endPos[5], speed, move_mode);
	strcat(sendCmd, strParams);

	int rtn = send(m_socketClient, sendCmd, strlen(sendCmd), 0);		//���������ַ���
	if (rtn == SOCKET_ERROR)	return ROBOT_ERROR;

	rtn = recv(m_socketClient, recvBuff, BUFF_SIZE, 0);
	if (rtn == SOCKET_ERROR)	return ROBOT_ERROR;

	int isOK = findFieldText(recvBuff, "errorMsg", m_errMsg);
	char errs[8];
	isOK = findFieldText(recvBuff, "errorCode", errs, NUMBER);
	return atoi(errs);
}

int JAKAZuRobot::stopMotion()
{
	char sendCmd[BUFF_SIZE] = "{\"cmdName\":\"stop_program\"}";

	char recvBuff[BUFF_SIZE];
	int rtn = send(m_socketClient, sendCmd, strlen(sendCmd), 0);
	rtn = recv(m_socketClient, recvBuff, MAX_BUFF_SIZE, 0);
	if (rtn == SOCKET_ERROR)	return ROBOT_ERROR;

	int isOK = findFieldText(recvBuff, "errorMsg", m_errMsg);
	char errs[8];
	isOK = findFieldText(recvBuff, "errorCode", errs, NUMBER);
	return atoi(errs);
}

int JAKAZuRobot::waitEndMove(int waitTime)
{
	LARGE_INTEGER start_t, end_t, frequency;
	QueryPerformanceFrequency(&frequency);	//��ȡ��ʱ����ʱ��Ƶ��
	QueryPerformanceCounter(&start_t);		//��ü�������ʼֵ

	char recvBuff[MAX_BUFF_SIZE];
	int rtn = 0;
	double tt = 0, tt_thre = 100.0;

	/*�п��ܴ洢�˶�ǰ��״̬������ȱ�֤�ܹ�����˶���״̬��Ϣ,
	*�������tt_threʱ������˶����״̬����ô��ֱ�ӷ���*/
	do	
	{
		rtn = recv(m_socketData, recvBuff, MAX_BUFF_SIZE, 0);
		if (rtn < 0) return ROBOT_ERROR;

		char *ptr_inpos = strstr(recvBuff, "\"inpos\"");
		char *ptr_ain = NULL;
		if (ptr_inpos) 	ptr_ain = strstr(recvBuff, "\"ain\"");
		if (ptr_ain)		//ֻ�е���inpos���͡�ain���������ֶ�ǰ�����ʱ���ܱ�֤inpos���ݵ�������
		{
			char fieldText[8];
			int num = sscanf(ptr_inpos, "%*[^:]:%[^,]", fieldText);
			if (strstr(fieldText, "true")) rtn = 1;
			else rtn = 0;
		}

		QueryPerformanceCounter(&end_t);
		tt = (end_t.QuadPart - start_t.QuadPart)*1000.0 / frequency.QuadPart;
	} while (rtn == 1 && tt < tt_thre);	

	if (tt >= tt_thre) return ROBOT_NOERR;

	do 
	{
		rtn = recv(m_socketData, recvBuff, MAX_BUFF_SIZE, 0);
		if (rtn < 0) return ROBOT_ERROR;

		char *ptr_inpos = strstr(recvBuff, "\"inpos\"");
		char *ptr_ain = NULL;
		if (ptr_inpos) 	ptr_ain = strstr(ptr_inpos, "\"ain\"");
		if (ptr_ain)
		{
			char fieldText[8];
			int num = sscanf(ptr_inpos, "%*[^:]:%[^,]", fieldText);
			if (strstr(fieldText, "true")) rtn = 1;
			else rtn = 0;
		}

		QueryPerformanceCounter(&end_t);
		tt = (end_t.QuadPart - start_t.QuadPart)*1000.0 / frequency.QuadPart;
	} while (rtn == 0 && tt < waitTime);

	if (tt >= waitTime) return ROBOT_ERROR;
	else return (int)tt;
}

int JAKAZuRobot::enableCVMotion(int enable /* = 1 */)
{
	char sendCmd[64];

	sprintf(sendCmd, "{\"cmdName\":\"servo_move\",\"relFlag\":%d}", enable);

	char recvBuff[BUFF_SIZE];
	int rtn = send(m_socketClient, sendCmd, strlen(sendCmd), 0);
	rtn = recv(m_socketClient, recvBuff, MAX_BUFF_SIZE, 0);
	if (rtn == SOCKET_ERROR)	return ROBOT_ERROR;

	int isOK = findFieldText(recvBuff, "errorMsg", m_errMsg);
	char errs[8];
	isOK = findFieldText(recvBuff, "errorCode", errs, NUMBER);
	return atoi(errs);
}

int JAKAZuRobot::cvMove(double *positions, int pts_num, int coord_type /* = COORD_JOINT */, int move_mode /* = 0 */)
{
	char sendCmd[BUFF_SIZE], cmdName[64], recvBuff[BUFF_SIZE];

	if (coord_type == COORD_JOINT)
	{
		strcpy(cmdName, "{\"cmdName\":\"servo_j\",\"jointPosition\":");
	}
	else
	{
		strcpy(cmdName, "{\"cmdName\":\"servo_p\",\"catPosition\":");
	}

	LARGE_INTEGER start_t, end_t, frequency;
	QueryPerformanceFrequency(&frequency);	//��ȡ��ʱ����ʱ��Ƶ��
	double *ptr = positions;
	for (int i = 0; i < pts_num; i++)
	{
		QueryPerformanceCounter(&start_t);		//��ü�������ʼֵ
		sprintf(sendCmd, "%s[%lf,%lf,%lf,%lf,%lf,%lf],\"relFlag\":%d}",
			cmdName,ptr[0], ptr[1], ptr[2], ptr[3], ptr[4], ptr[5], move_mode); /* positionsֻ����һά����ĵ�ַ */
		ptr += 6;

		int rtn = send(m_socketClient, sendCmd, strlen(sendCmd), 0);		//���������ַ���
		if (rtn == SOCKET_ERROR)	return ROBOT_ERROR;

		rtn = recv(m_socketClient, recvBuff, BUFF_SIZE, 0);
		if (rtn == SOCKET_ERROR)	return ROBOT_ERROR;

		char errs[8];
		int isOK = findFieldText(recvBuff, "errorCode", errs, NUMBER);
		rtn = atoi(errs);
		if (rtn != 0)
		{
			isOK = findFieldText(recvBuff, "errorMsg", m_errMsg);
			//return rtn;													//�����˶�ʱ���ص��ַ������Ϲ�(Ӧ�����ڲ�������)����ֱ������
		}
		//printf("pt_num = %d\n", i);
		do
		{
			QueryPerformanceCounter(&end_t); //��ü�������ֵֹ
		} while ((end_t.QuadPart - start_t.QuadPart) * 1000.0 <= 8.0 * frequency.QuadPart);
	}

	return ROBOT_NOERR;
}

int JAKAZuRobot::fKinematics(double *joint_pos, double *endPos)
{
	char sendCmd[BUFF_SIZE] = "{\"cmdName\":\"kine_forward\",\"jointPosition\":";
	char strParams[BUFF_SIZE];

	sprintf(strParams, "[%f,%f,%f,%f,%f,%f]}",
		joint_pos[0], joint_pos[1], joint_pos[2], joint_pos[3], joint_pos[4], joint_pos[5]);
	strcat(sendCmd, strParams);

	int rtn = send(m_socketClient, sendCmd, strlen(sendCmd), 0);
	if (rtn == SOCKET_ERROR)	return ROBOT_ERROR;

	char recvBuff[BUFF_SIZE];
	rtn = recv(m_socketClient, recvBuff, BUFF_SIZE, 0);
	if (rtn == SOCKET_ERROR)	return ROBOT_ERROR;

	char errs[8];
	int isOK = findFieldText(recvBuff, "errorCode", errs, NUMBER);
	rtn = atoi(errs);
	if (rtn != 0)
	{
		isOK = findFieldText(recvBuff, "errorMsg", m_errMsg);
		return rtn;
	}

	char *posbuf = sendCmd;
	isOK = findFieldText(recvBuff, "cartPosition", posbuf, ARRAY);
	isOK = sscanf(posbuf, "%lf,%lf,%lf,%lf,%lf,%lf",
		endPos, endPos + 1, endPos + 2, endPos + 3, endPos + 4, endPos + 5);

	return rtn;
}

int JAKAZuRobot::iKinematics(double *ref_joints, double *endPos, double *joint_pos)
{
	char sendCmd[BUFF_SIZE] = "{\"cmdName\":\"kine_inverse\",\"jointPosition\":";
	char strParams[BUFF_SIZE];

	sprintf(strParams, "[%f,%f,%f,%f,%f,%f],\"cartPosition\":[%f,%f,%f,%f,%f,%f]}",
		ref_joints[0], ref_joints[1], ref_joints[2], ref_joints[3], ref_joints[4], ref_joints[5],
		endPos[0], endPos[1], endPos[2], endPos[3], endPos[4], endPos[5]);
	strcat(sendCmd, strParams);

	int rtn = send(m_socketClient, sendCmd, strlen(sendCmd), 0);
	if (rtn == SOCKET_ERROR)	return ROBOT_ERROR;

	char recvBuff[BUFF_SIZE];
	rtn = recv(m_socketClient, recvBuff, BUFF_SIZE, 0);
	if (rtn == SOCKET_ERROR)	return ROBOT_ERROR;

	char errs[8];
	int isOK = findFieldText(recvBuff, "errorCode", errs, NUMBER);
	rtn = atoi(errs);
	if (rtn != 0)
	{
		isOK = findFieldText(recvBuff, "errorMsg", m_errMsg);
		return rtn;
	}

	char *posbuf = sendCmd;
	isOK = findFieldText(recvBuff, "jointPosition", posbuf, ARRAY);
	isOK = sscanf(posbuf, "%lf,%lf,%lf,%lf,%lf,%lf",
		joint_pos, joint_pos + 1, joint_pos + 2, joint_pos + 3, joint_pos + 4, joint_pos + 5);

	return rtn;
}

//���:a*sin(theta)+b*cos(theta) = c;����2��q��
void slvTrigonometry(double a, double b, double c, double *q)
{
	q[0] = atan2(b, -a) - atan2(c, sqrt(a*a + b*b - c*c));
	q[1] = atan2(b, -a) - atan2(c, -sqrt(a*a + b*b - c*c));
}


int JAKAZuRobot::iKinematics(double T[4][4], double *joint_pos)
{
	double a2 = 0.246, a3 = 0.190;
	double d1 = 0.15055, d2 = 0.115, d3 = 0.1163;
	double d4 = 0.1175, d5 = 0.1175, d6 = 0.105;

	double q1[8], q2[8], q3[8], q4[8], q5[8], q6[8];
	char flag[8] = { 0 };		//���ÿ�����Ƿ����

	slvTrigonometry(T[0][3] - T[0][2]*d6, -T[1][3] + d6*T[1][2], -d2 + d3 - d4, q1);
	q5[0] = acos(T[0][2] * sin(q1[0]) - T[1][2] * cos(q1[0]));
	q5[1] = acos(T[0][2] * sin(q1[1]) - T[1][2] * cos(q1[1]));
	q5[2] = -q5[0]; q5[3] = -q5[1];

	q1[2] = q1[0]; q1[3] = q1[1];

	int nslv = 0;				//��Ч�����
	for (int i = 0; i < 4; i ++)
	{
		double aa = (T[0][1]*sin(q1[i]) - T[1][1]*cos(q1[i])) / -sin(q5[i]);
		double bb = (T[0][0]*sin(q1[i]) - T[1][0]*cos(q1[i])) / sin(q5[i]);
		q6[i] = atan2(aa, bb);

		double q234 = atan2(-T[2][2] / sin(q5[i]), -(cos(q1[i])*T[0][2] + sin(q1[i])*T[1][2]) / sin(q5[i]));

		aa = T[0][3] * cos(q1[i]) + T[1][3] * sin(q1[i]) - d5 * sin(q234) + d6 * sin(q5[i]) * cos(q234);
		bb = T[2][3] - d1 + d5 * cos(q234) + d6 * sin(q5[i]) * sin(q234);
		double tt = (aa * aa + bb * bb - a2 * a2 - a3 * a3) / 2 / a2 / a3;

		if (fabs(tt) <= 1.0)
		{
			q3[i] = acos(tt);
			double ab = a2 * a2 + a3 * a3 + 2 * cos(q3[i]) * a2 * a3;
			double s2 = (-a3*sin(q3[i])*aa + (a2 + a3*cos(q3[i]))*bb) / ab;
			double c2 = ((a2 + a3 * cos(q3[i])) * aa + a3 * sin(q3[i]) * bb) / ab;
			q2[i] = atan2(s2, c2);
			q4[i] = q234 - q2[i] - q3[i];

			flag[i] = flag[4 + i] = 1;		//�����Ч��
			nslv += 2;

			q3[4+i] = -q3[i];
			//ab = a2*a2 + a3*a3 + 2 * cos(q3[i])*a2*a3;
			s2 = (a3 * sin(q3[i]) * aa + (a2 + a3 * cos(q3[i])) * bb) / ab;
			c2 = ((a2 + a3 * cos(q3[i])) * aa - a3 * sin(q3[i]) * bb) / ab;
			q2[4+i] = atan2(s2, c2);
			q4[4+i] = q234 - q2[4+i] - q3[4+i];
			q1[4 + i] = q1[i];	q5[4 + i] = q5[i]; q6[4 + i] = q6[i];
		}
	}

	if (joint_pos == NULL)
	{
		joint_pos = new double[nslv];
	}
	double *ppos = joint_pos;
	for (int i = 0; i < 8; i ++)
	{
		if (flag[i])
		{
			ppos[0] = q1[i]; ppos[1] = q2[i]; ppos[2] = q3[i];
			ppos[3] = q4[i];ppos[4] = q5[i]; ppos[5] = q6[i];
			ppos += 6;
		}
	}

	//���ƽǶȷ�ΧΪ(-pi, pi]
	for (int i = 0; i < nslv*6; i++)
	{
		if (joint_pos[i] > M_PI) joint_pos[i] -= 2*M_PI;
		if (joint_pos[i] <= -M_PI) joint_pos[i] += 2*M_PI;
	}

	return nslv;
}

void JAKAZuRobot::Cal_iKine(double* joints_ref, double* end, double* joints_out)
{
	/* 1.����ĩ��λ�˵����������ʽ(4x4) */
	Vector3d transl(end[0] * 1e-3, end[1] * 1e-3, end[2] * 1e-3);//��λת��m
	Vector3d EurAngle(end[3] * PI / 180, end[4] * PI / 180, end[5] * PI / 180);//��λת��rad
	Isometry3d T0 = Isometry3d::Identity(); //����4x4����ξ���,��ʾĩ�˵�λ��
	Get_TransMtx(T0, transl, EurAngle);
	MatrixXd T1 = T0.matrix().transpose();
	double* a = T1.data();
	double endpos[4][4] ; //��ĩ��λ���Զ�ά����ķ�ʽ�洢
	memcpy(endpos, a, sizeof(double) * 16);

	/* 2.��ĩ��λ�������,������ �ο��ؽڿռ� ��ֵ��С�� */
	double qs[48];//�����е����(6x8)
	int num = this->iKinematics(endpos, qs);//��Ч���ĸ���

	//for (int i = 0; i < 6*num; ++i)
	//{
	//	qs[i] = qs[i] * 180 / PI; //rad->deg
	//}

	int num1 = 0;//��ɸ����ʣ���ĸ���
	double min = 2e25;
	double qs_new[24];
	double* qs_temp = qs_new;//tempָ�������Ŀ�ʼ
	for (int i = 0; i < num; ++i)//q1�ĽǶ�һ����2��,�ȸ���q1��ɸ
	{
		if (abs((qs + 6 * i)[0] - joints_ref[0]*3.14/180) < 1)
		{
			num1++;
			memcpy(qs_temp, qs + 6 * i, sizeof(double) * 6);
			qs_temp += 6;
		}
	}

	int idx = 0;
	qs_temp = qs_new;//tempָ�������Ŀ�ʼ
	for (int it = 0; it < num1; ++it)
	{
		double dist = L2_Norm(qs_temp, joints_ref);
		if(dist<min)
		{
			min = dist;
			idx = it;
		}
	}
	/* 3.����Ӧ�ĹؽڽǸ�ֵ,�����е�λת�� */
	memcpy(joints_out, qs_new + 6 * idx, sizeof(double) * 6);
	for (int i = 0; i < 6; ++i)
	{
		joints_out[i] = joints_out[i] * 180 / PI; //rad->deg
	}
}

void JAKAZuRobot::Crab_Blob(Blob& blob, double height_offset, double speed)
{
	/* 0.�ſ���צ */
	this->setDout(Crab_IO, Crab_Open, 0);

	/* 1.�˶���ɫ�����Ϸ� */
	double end[6] = { blob.m_center.x,blob.m_center.y,Crab_Height + 40,-180,0,blob.m_angle };
	int ret = this->moveP(end, speed);
	cout << "�����˶���ɫ�����Ϸ�: " << ret << endl;
	//this->waitEndMove(20000);//�����ȴ�20s
	Sleep(15000);

	/* 2.��ֱ�����˶�ץסɫ�� */
	end[2] = Crab_Height;
	ret = this->moveP(end, speed);
	cout << "���ڴ�ֱ�����˶�ץסɫ��: " << ret << endl;
	//this->waitEndMove(20000);//�����ȴ�
	Sleep(2500);

	this->setDout(Crab_IO, Crab_Close, 0);//�رռ���
	Sleep(Base_Delay);

	/* 3.������Ϣ����ɫ��һ���߶� */
	end[2] = Crab_Height + height_offset;
	ret = this->moveP(end, speed);
	cout << "��������ɫ��һ���߶�: " << ret << endl;
	//this->waitEndMove(20000);//�����ȴ�
	Sleep(2500);
}

void JAKAZuRobot::Transport_Blob(Blob& blob, double height_offset, const int cnt)
{
	/* 1.�˶��켣���� */
	height_offset = 0;
	double* end_arr = blob.Curve_Cal(cnt, height_offset);//������ĩ��λ��(XYZ+R(X)R(Y)R(Z))
	int cnt_new = _msize(end_arr) / sizeof(double) / 6;//�µ���ɢ�����
	double* joint_arr = new double[cnt_new * 6];//�����йؽڽǶ�
	
	double endpos[6] = { 208.018, 207.806, 4.6+87.3, -176.402, -0.743,-67.717 };
	double endendpos[6] = { 321, 209,92.99,-180,0,-70.8646 };//CVmove������
	double end5[6] = { 214.328,193.1,118.64,-179.245,6.581,5.536};
	/*double joints_ref[6] = { -55.83,54.743,-108.58,143.84,90,16.46 };*///��һ�εĹؽڽǶ�
	double end1[6] = { 115.64,327.84,3.757 + 87.3 ,-174.253,6.581,-42.117 };

	//int ret = this->getJoints(joints_ref);//��ȡ��ʼ�Ƕ�

	double* p_joint = joint_arr;
	double* p_end = end_arr;
	double* p_temp = p_joint;

	cout << "��ʼ�������" << endl;
	double joints_ref[6] = { 0,0,0,0,0,0 };
	//this->Cal_iKine(joints_ref, end_arr, p_joint);
	for (int i = 0; i < cnt_new; ++i)
	{
		if (i == 0)
		{
			this->Cal_iKine(joints_ref, p_end, p_joint);//��һ�εĳ�ʼ�ο��ؽ�������õ�
		}
		
		else
		{
			this->Cal_iKine(p_temp, p_end, p_joint);//���εĲο��ؽ����ϴε����
			p_temp += 6;
		}
		p_joint += 6;
		p_end += 6;
	}
	delete[] end_arr;
	cout << "���������" << endl;

	/* ������ؽ�˳ʱ���ٶ� (deg/s)*/
	//p_joint = joint_arr;
	//double* velocity = new double[6*(cnt_new - 1)];
	//double* v_ptr = velocity;
	//for (int i = 0; i < cnt_new-1; ++i)
	//{
	//	for (int j = 0; j < 6; ++j)
	//	{
	//		v_ptr[j] = 125*((p_joint + 6)[j] - p_joint[j]);
	//	}
	//	v_ptr += 6;
	//	p_joint += 6;
	//}
	//delete[] velocity;

	/* 2.�ر���צ */
	this->setDout(Crab_IO, Crab_Close, 0);
	Sleep(Base_Delay);

	/* 3.��ʼ�˶� */
	this->enableCVMotion(1);
	cout << "��ʼCV_MOVE" << endl;
	this->cvMove(joint_arr, cnt_new, COORD_JOINT, 0);
	//this->waitEndMove(20000);
	Sleep(15000);
	this->enableCVMotion(0);
	delete[] joint_arr;

}

void JAKAZuRobot::Lay_Blob(Blob& blob, double height_offset, double speed)
{
	/* 1.�򿪼��� */
	this->setDout(Crab_IO, Crab_Open, 0);
	Sleep(Base_Delay);

	/* 2.��Z���˶���ɫ�����Ϸ� */
	double end[6] = { blob.m_target.x,blob.m_target.y,Crab_Height + height_offset + 40,-180,0,blob.m_angle };
	int ret = this->moveP(end, speed);
	cout << "�����˶���ɫ�����Ϸ�: " << ret << endl;
	this->waitEndMove(20000);//�����ȴ�20s
}