// IPhy.h: interface for the IPhy class.
//
//////////////////////////////////////////////////////////////////////


// Linux兼容的类型定义
typedef int BOOL;
typedef unsigned char UCHAR;
typedef unsigned short WORD;
typedef unsigned int UINT;

// Linux兼容的布尔值定义
#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

class CVoyCmd;

class IPhy
{
public:
	IPhy() { IsRuning = FALSE; };
	virtual ~IPhy() {};
	virtual void SetCmd(CVoyCmd *pCmd) {};						  // 设置指令类对象
	virtual void Send(const void *pBuffer, const int iLength) {}; // 发送指令（供其他对象调用）
	BOOL IsRuning;												  // 运行状态标记
	BOOL bSending;												  // 发送标记
};
