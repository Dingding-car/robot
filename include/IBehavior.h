// IBehavior.h: interface for the IBehavior class.
//
//////////////////////////////////////////////////////////////////////

#include <cstddef>  // 用于 NULL 定义

// Linux兼容的类型定义
typedef unsigned char UCHAR;
typedef unsigned short WORD;
typedef unsigned int UINT;
typedef int BOOL;
typedef double DOUBLE;
typedef float FLOAT;
typedef unsigned char BYTE;

// Linux兼容的布尔值定义
#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

class CVoyCmd;

class IBehavior
{
public:
	IBehavior() { m_pCmd = NULL; };
	virtual ~IBehavior() {};
	virtual void SetCmd(CVoyCmd *pCmd) { m_pCmd = pCmd; };														  // 设置指令类对象
	virtual void AfterUpdateInfrared(UCHAR *Infrared, BOOL *EnableIR, UINT nState) {};							  // 红外传感器信息更新后的处理函数
	virtual void AfterUpdateUSonic(DOUBLE *Ultrasonic, BOOL *EnableUS, UINT nState) {};							  // 超声传感器信息更新后的处理函数
	virtual void AfterUpdateVideoSample(BYTE *pBuffer, long lWidth, long lHeight, double dbTime, UINT nState) {}; // 主前视摄像装置接收到一桢图象后的处理函数
	virtual void AfterUpdateOverlook(BYTE *pBuffer, long lWidth, long lHeight, double dbTime, UINT nState) {};	  // 全局摄像装置接收到一桢图象后的处理函数
	virtual void AfterSendCommand(BYTE *pBuffer, int iLength, UINT nState) {};									  // 发送指令完毕后的处理函数
	virtual void AfterUpdateAttitude(FLOAT inAngle, FLOAT inXRoll, FLOAT inYRoll) {};							  // 姿态信息更新
	CVoyCmd *m_pCmd;																							  // 指令类对象指针
};