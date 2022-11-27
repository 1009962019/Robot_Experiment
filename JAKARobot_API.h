#ifndef _JAKAAPI_H_
#define _JAKAAPI_H_

#include "Blob.h"
#include <Winsock2.h>

#ifdef DLLEXPORT_EXPORTS
#define DLLEXPORT_API __declspec(dllexport)
#else
#define DLLEXPORT_API __declspec(dllimport)
#endif

enum ERROR_TYPE {ROBOT_NOERR = 0, ROBOT_ERROR = -1};
enum IO_TYPE {IO_CABINET, IO_TOOL, IO_EXTEND};
enum COORD_TYPE {COORD_JOINT = 0, COORD_WORD, COORD_TOOL};	//Ĭ�ϵ���������ϵΪ��������ϵ

class Blob;//���Ƕ��ʹ����Ҫǰ������

class JAKAZuRobot
{
public:

	/**
	* @brief ��е�ۿ����๹�캯��
	*/
	JAKAZuRobot();

	/*******************************ϵͳָ��******************************/
	/**
	* @brief ���ӻ����˲���½
	* @param ip IP��ַ�ַ���
	* @return NO_ERROR �ɹ� ����ʧ��
	*/
	int login_in(const char* ip);

	/**
	* @brief �˳�����
	* @return NO_ERROR �ɹ� ����ʧ��
	*/
	int login_out();

	/**
	* @brief �����˵�Դ����
	* @param isOn:1��������Դ��0���رյ�Դ
	* @return NO_ERROR �ɹ� ����ʧ��
	*/
	int power(int isOn = 1);

	/**
	* @brief ʹ�ܿ���
	* @param enable��1����ʹ�ܣ�0����ʹ��
	* @return NO_ERROR �ɹ� ����ʧ��
	*/
	int enable(int isEnable = 1);

	/**
	* @brief ��ʼ�����ã����������ö���ΪĬ��ֵ
	* @return NO_ERROR �ɹ� ����ʧ��
	*/
	int initSetting();

	/**
	* @brief �����ײ��Ĵ���
	* @return NO_ERROR �ɹ� ����ʧ��
	*/
	int clearError();

	/**
	* @brief ��û����˵�ʹ�ܺ͵�Դ״̬
	* @param(out) is_enable��1��ʹ��״̬��0����ʹ��״̬ 
	* @param(out) is_powered��1���ϵ�״̬��0���µ�״̬
	* @return NO_ERROR �ɹ� ����ʧ��
	*/
	int getState(int &is_enable, int &is_powered);

	/*****************************��дָ��***************************************/
	/* @brief ��ȡ�������������ݵ��ַ�����
	*	ע����ͬʱ��Ҫ��ö������ֵ�ȵ��øú���������Ч
	* @param(out) all_data ����������ַ����������ַ����ַ�������2000
	* @param port �˿ںţ�Ĭ��Ϊ1����Ӧ10001������Ϊ10000��רΪ�����ݣ�
	* @return NO_ERROR �ɹ� ����ʧ��
	*/
	int getData(char *all_data, int port = 1);

	/* @brief ��ȡ�����˹ؽڽ�
	* @param(out) joint_angles �����˹ؽڽǣ���λ�Ƕ�
	* @param data_info �����ָ�벻Ϊ�գ���ô��Ҫ�ȵ���һ��getData�������ԭʼ����
	* @return NO_ERROR �ɹ� ����ʧ��
	*/
	int getJoints(double *joint_angles, char *data_info = NULL);

	/* @brief ��ȡ�����˹��ߵ�λ��
	* @param(out) tcp_pose �����˹ؽڽǣ���λ�Ƕ�
	* @param data_info �����ָ�벻Ϊ�գ���ô��Ҫ�ȵ���һ��getData�������ԭʼ����
	* @return NO_ERROR �ɹ� ����ʧ��
	*/
	int getTcpPose(double *tcp_pose, char *data_info = NULL);

	/* @brief ��ѯ��������������ڵ�״̬
	* @param(out) dio_in ��������˶�Ӧ��������ڵ�״̬����λ��ǣ��ӵ͵��ߣ�
	*					 �����2λΪ1����ô��Ӧ��DI1���±��0��ʼ��Ϊ��״̬
	* @param data_info �����ָ�벻Ϊ�գ���ô��Ҫ�ȵ���һ��getData�������ԭʼ����
	* @param IOType IO���ͣ�IO_CABINET(0)�����ƹ�IO������8;IO_TOOL(1)������IO������2
	* @return NO_ERROR �ɹ� ����ʧ��
	*/
	int getDin(int &dio_in, int IOType = IO_CABINET, char *data_info = NULL);

	/* @brief �������������(DO)��״̬
	* @param index DO��������0��ʼ��
	* @param value DO����ֵ��1��0��
	* @param IOType IO���ͣ�IO_CABINET(0)�����ƹ�IO;IO_TOOL(1)������IO
	* @return NO_ERROR �ɹ� ����ʧ��
	*/
	int setDout(int index, int value = 1, int IOType = IO_CABINET);

	/* @brief ����Զ���Ĺ�������ϵ
	* @param tool_offset ����ƫ�ƾ�������ͬλ�����������鳤��Ϊ6
	* @param tool_name ��������
	* @param id_num ���ߵ�ID��
	* @return NO_ERROR �ɹ� ����ʧ��
	*/
	int addTool(double *tool_offset, char *tool_name, int tool_id = 1);

	/* @brief ���ù�������ϵ
	* @param tool_id ���ߵ�ID�ţ�id=0�����ʾ���������ģ�
	*				������Ϊ�Զ���Ĺ�������ϵ��ͬaddTool�������ʹ��
	* @return NO_ERROR �ɹ� ����ʧ��
	*/
	int setToolID(int tool_id = 0);

	/* @brief ��ȡ��ǰ�Ĺ�������ϵID
	* @param tool_id(���) ���ߵ�ID��
	* @return NO_ERROR �ɹ� ����ʧ��
	*/
	int getToolID(int &tool_id);

	/* @brief ����Զ�����û�����ϵ����������ϵ��
	* @param user_offset ��������ϵ��ƫ�ƾ�������ͬλ�����������鳤��Ϊ6
	* @param user_name ��������
	* @param user_id ���ߵ�ID��
	* @return NO_ERROR �ɹ� ����ʧ��
	*/
	int addUserCord(double *user_offset, char *user_name, int user_id = 1);

	/* @brief ѡ���û�����ϵ
	* @param user_id �û�����ϵ��ID�ţ�id=0�����ʾ��������ϵ��
	*				������Ϊ�Զ���Ĺ�������ϵ��ͬaddUserCord�������ʹ��
	* @return NO_ERROR �ɹ� ����ʧ��
	*/
	int setUserID(int user_id = 0);

	/* @brief ��ȡ��ǰ���û�����ϵID
	* @param user_id(���) �û�����ϵ��ID��
	* @return NO_ERROR �ɹ� ����ʧ��
	*/
	int getUserID(int &user_id);

	/* @brief ����ĩ�˸���
	* @param mass ���ص���������λkg
	* @param offset ���ص�����[x,y,z]����λmm
	* @return NO_ERROR �ɹ� ����ʧ��
	*/
	int setPayload(double mass, double *offset);

	/* @brief ��ȡĩ�˸���
	* @param mass(out) ���ص���������λkg
	* @param offset(out) ���ص�����[x,y,z]����λmm
	* @return NO_ERROR �ɹ� ����ʧ��
	*/
	int getPayload(double &mass, double *offset);

	/* @brief ���û������ٶȱ���
	* @param rapid_rate �ٶȱ���[0,1]
	* @return NO_ERROR �ɹ� ����ʧ��
	*/
	int setRapidRate(double rapid_rate);

	/***************************�˶�ָ��******************************************/
	/* @brief �����˹ؽ��˶�
	* @param joint_pos ������6���ؽ��˶�Ŀ��λ��,��λ����
	* @param speed �����˹ؽ��˶��ٶȣ���λ����/s
	* @move_mode ָ���˶�ģʽ��0�������˶���1���������˶�
	* @return NO_ERROR �ɹ� ����ʧ��
	*/
	int  moveJ(const double *joint_pos, double speed, int move_mode = 0);

	/* @brief ������ĩ���˶�:�������˶�ѧ��Ȼ����moveJ
	* @param endPos ������ĩ��λ��[x,y,z,a,b,c],λ������Ϊrpy�ǣ�R=rz(c)ry(b)rx(a),�Ƕȵ�λ��
	* @param speed �����˹ؽ��˶��ٶȣ���λ����/s
	* @return NO_ERROR �ɹ� ����ʧ��
	*/
	int  moveP(const double *endPos, double speed);

	/* @brief ������ĩ���˶�ֱ���˶�
	* @param endPos ������ĩ��λ��[x,y,z,a,b,c],λ������Ϊrpy�ǣ�R=rz(c)ry(b)rx(a),�Ƕȵ�λ��
	* @param speed �����˹ؽ��˶��ٶȣ���λ��mm/s
	* @move_mode ָ���˶�ģʽ��0�������˶���1���������˶�
	* @return NO_ERROR �ɹ� ����ʧ��
	*/
	int  moveL(const double *endPos, double speed, int move_mode = 0);

	/* @brief ֹͣ�����˵��˶�
	* @return NO_ERROR �ɹ� ����ʧ��
	*/
	int stopMotion();

	/* @brief �ȴ��˶�����������ָ����˶��������ִ����һ��ָ��
	/* @param waitTime,��ȴ�ʱ�䣨��λ:ms)������������ʱ��
	* @return >=0 ����ʱ�䣨ms); < 0 ������ʱ
	*/
	int waitEndMove(int waitTime = 10000);

	/* @brief �����˽��������˶�ʹ��
	* @param enable 1�������룻0�����˳�
	* @return NO_ERROR �ɹ� ����ʧ��
	*/
	int enableCVMotion(int enable = 1);

	/* @brief ���������������˶�
	* @param positions ������λ�����ݣ�n*6�������ؽڽǻ�ѿ���λ�����������ԣ�ȡ��������ϵ����
	* @param pts_num λ�õ����
	* @param coord_type ��������ϵ��COORD_JOINT�����ؽڿռ䣬COORD_WORD������������ϵ��Ĭ�ϻ�������ϵ��
	* @param move_mode ָ���˶�ģʽ��0�������˶���1���������˶�
	* @return NO_ERROR �ɹ� ����ʧ��
	*/
	int cvMove(double *positions, int pts_num, int coord_type = COORD_JOINT, int move_mode = 0);

	/***************************�˶�ѧ******************************************/
	/* @brief ���������˶�ѧ
	* @param joint_pos �����˹ؽڽǣ������С6��
	* @param endPos(out) ������ĩ�˵�λ��
	* @return NO_ERROR �ɹ� ����ʧ��
	*/
	int fKinematics(double *joint_pos, double *endPos);

	/* @brief ���������˶�ѧ
	* @param ref_joints �����˲ο��ؽڽǣ������С6�����Ƽ��û�ѡȡ�����˵�ǰ�Ĺؽڽ���Ϊ�ο��ؽڽ�
	* @param endPos ������ĩ�˵�λ��
	* @param joint_pos(out) �����˹ؽڽ�
	* @return NO_ERROR �ɹ� ����ʧ��
	*/
	int iKinematics(double *ref_joints, double *endPos, double *joint_pos);

	//--------------------------------����Ϊ�Զ��庯��-------------------------------------------------
	/* @brief ���������˶�ѧ�������а���ʽ����
	* @param endPos ������ĩ�˵�λ��4*4����,��λ��m + rad
	* @param joint_pos(out) �����˹ؽڽǣ�-pi, pi]����ַ�������ⲿ���룬��С����Ϊ8*6
	* @return <0 ʧ�ܣ� ������������
	*/
	int iKinematics(double endPos[4][4], double *joint_pos);

	/**
	 * @brief �ڱ����ɵ�ǰ6��Ƕȼ������˶�ѧ
	 * @param joints_ref ��ǰ�����˵Ĺؽڽ�(6x1),��λ��deg
	 * @param end ��е��ĩ�˵�λ��(6x1):XYZ+ŷ����,��λ��mm + deg
	 * @param joints_out �����˵Ĺؽڽ�(6x1),��λ��deg
	 * @return 
	*/
	void Cal_iKine(double* joints_ref, double* end, double* joints_out);

	/**
	 * @brief ���˶���ɫ�����Ϸ�,Ȼ��ֱ�����˶�ץסɫ��,�� ������Ϣ ����ɫ��һ���߶�
	 * @param blob 
	 * @param height_offset ����ץȡɫ����������ĸ߶�ƫ��
	 * @param speed �ؽ��ٶ�(deg/s)
	*/
	void Crab_Blob(Blob& blob, double height_offset, double speed);

	/**
	 * @brief ͨ��CvMove��ɫ�����䵽ָ��λ��
	 * @param blob 
	 * @param height_offset ����ץȡɫ����������ĸ߶�ƫ��
	 * @param cnt �켣����ɢ�������
	*/
	void Transport_Blob(Blob& blob, double height_offset, const int cnt);

	/**
	 * @brief ���ɿ�ɫ��Ȼ���˶���ɫ�����Ϸ�
	 * @param blob 
	 * @param height_offset ����ץȡɫ����������ĸ߶�ƫ��
	 * @param speed �ؽ��ٶ�(deg/s)
	*/
	void Lay_Blob(Blob& blob, double height_offset, double speed);

public:
	char m_errMsg[128];			//���������Ϣ
	int blob_num[6] = {0,0,0,0,0,0}; //��ÿ����ɫ��ɫ��ĸ���
private:

	SOCKET m_socketClient;
	SOCKET m_socketData;		//����ʵʱ������ݵ�socket
};

//���Ե�ʱ��������������ɾ����
int findFieldText(char *recvInfo, char *fieldName, char *fieldText, int txtType = 0);

/**
 * @brief �����˳�ʼ��
 * @param jakaRob 
 * @return 
*/
int Robot_Init(JAKAZuRobot& jakaRob);

/**
 * @brief ��������˵ĹؽڽǶ�
 * @param jakaRob 
*/
void Robot_Print_Joints(JAKAZuRobot& jakaRob);

/**
 * @brief ��������˵�ĩ��λ��
 * @param jakaRob 
*/
void Robot_Print_Pose(JAKAZuRobot& jakaRob);
#endif