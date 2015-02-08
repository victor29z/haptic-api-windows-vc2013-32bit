

#include "StdAfx.h"
#define DLL_DELTA_CONTROL _declspec(dllexport)
#include "DeltaControl.h"
// #include "windows.h"

// lib
#include "omegafunc.h"
// #pragma	comment(lib, "SiUSBXp.lib")		// get in the lib
//#include "SiUSBXp.h"
#include	"CH375DLL.H"				// CH375的动态链接库
#pragma comment(lib,"CH375DLL")

#include <Eigen/Dense>		// 不在类中声明成员，只在CPP中使用，避免要求安装
using namespace Eigen;

// macro definition
// CMD_LENGTH
#define 		CMD_LENGTH_HOST2DEV			13
#define			CMD_LENGTH_DEV2HOST			14
// control word
#define CTW_COMMON			0
#define CTW_SIMPLE			1
#define	CTW_PMW_SETING		2
#define	CTW_OPEN_FORCE		3
#define	CTW_SET_POS			4
#define CTW_PID				5

// dev_feedback
#define	DEV_POS_STA			1
// const_value
#define PWM_SAF_MAX				2000		// 12B 4095 max
#define	PWM_MAX					4095
#define	POS_THRESH_MAX			40000		// force feedback force

//	#define	SP_SAFE_MAX				1000
#define	SP_SAFE_MAX				550			// for 0.5A change later


//	mechanic para
#define PI				3.14
//#define RD_RATIO		15.4		// reduction ratio R 57  r 3.7
#define	RD_RATIO		14.25		// for 6 dof
#define	RD_RATIO4		15.27
#define	RD_RATIO5		14.18
#define	RD_RATIO6		6.46


#define EN_SCALAR		2048		// IE2-512
//#define	EN_SCALAR		4096		// for the 2657 cxr
//#define	SC_TOR2PWM	870			// test para caculated from the data sheet
#define	SC_TOR2PWM		85
//	#define	T2CURRENT		93068			// the torque Nm to the target current set point in digital. 4095/0.044
//#define	T2CURRENT		7500			// 330 - 0.044
// #define	T2CURRENT		12500
#define		T2CURRENT		17600		// -400 -328 ---- 400 500 17600*800/828 = 17004
//#define		T2CURRENT		17004
#define		T2C_M4			22380
#define		T2C_M6			26070

#define 	T_MAX		10

// clock management
#define	INTERVAL_US		1500

// global for 

// global constant
const double t2cur[6]={327000,327000,327000,21622,21622,25193};
//const double t2cur[6]={17004,17004,17004,21622,21622,25193};
//const double t2cur[6]={1167075,1167075,1167075,21622,21622,25193};//transform the current scale to -32760~32760
const double trRatio[6] = {14.25,14.25, 14.25 ,15.27, 14.18, 6.46};
const double trBias[6] = {0.418645,0.418645,1.378161 ,0, 0, 0};

CDeltaUSBControl::CDeltaUSBControl(void)
: m_hUSBDevice(NULL),
m_bReadError(FALSE),
m_bWriteError(FALSE),
m_nDeviceNum(0),
m_state(0)
{
	memset(m_encoder,0,sizeof(m_encoder));
	memset(m_dF, 0, sizeof(double)*3);
	memset(m_dFTar, 0, sizeof(m_dFTar));
	memset(m_dRad,0, sizeof(m_dRad));
	memset(m_dT,0,sizeof(m_dT));
	memset(m_dHXTar,0,sizeof(m_dHXTar));
	memset(m_dHX,0,sizeof(m_dHX));
	m_dHX[2]=-0.15;

	memset(m_dTDisk,0,sizeof(m_dTDisk));

	memset(Out_Packet,0,sizeof(Out_Packet));
	memset(In_Packet,0,sizeof(In_Packet));
	memset(m_R,0,sizeof(m_R));
	m_R[0][0] = 1;
	m_R[1][1] = 1;
	m_R[2][2] = 1;				// set R to indentity matrix



	/*m_HDConfig[0] = 0.88; // Lb
	m_HDConfig[1] = 0.44; // La
	m_HDConfig[2] = 0.3; // R
	m_HDConfig[3] = 0.1;	// r*/

	// new parameters
	m_dHDConfig[0] = 0.146; // Lb
	m_dHDConfig[1] = 0.07; // La
	m_dHDConfig[2] = 0.0715; // R
	m_dHDConfig[3] = 0.037;	// r

	// system frequency
	m_liPerfStart.QuadPart=0;
	m_liPerfNow.QuadPart=0;

	QueryPerformanceFrequency(&m_liPerfFreq); 
}

CDeltaUSBControl::~CDeltaUSBControl(void)
{
}

/*BOOL CDeltaUSBControl::GetDeviceList(void)
{
	DWORD	dwNumDevices = 0;
	SI_DEVICE_STRING	devStr;

	SI_STATUS status = SI_GetNumDevices(&dwNumDevices);
	m_DeviceList.clear();	// clear it. Initial program doesn't contain it .

	if (status == SI_SUCCESS)
	{
		for (DWORD d = 0; d < dwNumDevices; d++)
		{
			status = SI_GetProductString(d, devStr, SI_RETURN_SERIAL_NUMBER);

			if (status == SI_SUCCESS)
			{
				string str = string(devStr);
				m_DeviceList.push_back(str);	
			}
		}
	}
	else
	{
		return FALSE;
	}

	return TRUE;
	// return 0;
}
*/
bool CDeltaUSBControl::ConnectDevice(int nInd)
{
	// for 372
	m_hUSBDevice = INVALID_HANDLE_VALUE;		// false handle -1;
	m_nDeviceNum = nInd;
	CH375CloseDevice( nInd );		// clear the device
	if ( CH375OpenDevice( nInd ) == INVALID_HANDLE_VALUE ) return FALSE;  /* 使用之前必须打开设备 */
	CH375ResetDevice(nInd);		// 设备号清理

	//CH375SetTimeout( 0, 5000, 5000 );  // 设置USB数据读写的超时,超过5000mS未完成读写将强制返回,避免一直等待下去
	CH375SetTimeout( 0, 1000, 1000 ); 
	// Get the initial tickCount
	QueryPerformanceCounter(&m_liPerfStart); 

	return TRUE;

	// for USBXpress
	/*
	if (!GetDeviceList())
	{
	//	AfxMessageBox(L"Error finding USB Device");
		return FALSE;		// for host to judge
	}
	
	// Following can be modified by using a Dlg for device selection; A function for a dlg get the number and feedback;
	m_nDeviceNum = 0;		// 0 for first
	m_sDeviceName = m_DeviceList[m_nDeviceNum];	// name for first
	
	SI_STATUS status = SI_Open((DWORD)m_nDeviceNum, &m_hUSBDevice);
	
	if (status != SI_SUCCESS)
	{
	//	CString sMessage;
	//	sMessage.Format(L"Error opening device: %s\n\nApplication is aborting.\nReset hardware and try again.", m_sDeviceName);
	//	if (AfxMessageBox(sMessage,MB_OK|MB_ICONEXCLAMATION))
	//	{
			// OnCancel();
			return FALSE;
	//	}
	}
	return TRUE;*/
	
}

bool CDeltaUSBControl::Set6AxisPos(UINT16  U16AxisPos[6],int & nErrCode)
{
	
	Out_Packet[0] = CTW_SET_POS;
	Out_Packet[1] =  U16AxisPos[0] & 0xff;
	Out_Packet[2] = (U16AxisPos[0]>>8) & 0xff;	// high 8 bit
	Out_Packet[3] =  U16AxisPos[1] & 0xff;
	Out_Packet[4] = (U16AxisPos[1]>>8) & 0xff;	// high 8 bit
	Out_Packet[5] =  U16AxisPos[2] & 0xff;
	Out_Packet[6] = (U16AxisPos[2]>>8) & 0xff;	// high 8 bit
	Out_Packet[7] =  U16AxisPos[3] & 0xff;
	Out_Packet[8] = (U16AxisPos[3]>>8) & 0xff;	// high 8 bit
	Out_Packet[9] =  U16AxisPos[4] & 0xff;
	Out_Packet[10] = (U16AxisPos[4]>>8) & 0xff;	// high 8 bit
	Out_Packet[11] =  U16AxisPos[5] & 0xff;
	Out_Packet[12] = (U16AxisPos[5]>>8) & 0xff;	// high 8 bit
	
	return IOMsg(nErrCode);

	/*	// USBXpress controller
	m_send[0] = CTW_SET_POS;
	m_send[1] = U16AxisPos[0] & 0xff;	// low 8 bit
	m_send[2] = (U16AxisPos[0]>>8) & 0xff;	// high 8 bit
	m_send[3] = U16AxisPos[1] & 0xff;	// low 8 bit
	m_send[4] = (U16AxisPos[1]>>8) & 0xff;	// high 8 bit
	m_send[5] = U16AxisPos[2] & 0xff;	// low 8 bit
	m_send[6] = (U16AxisPos[2]>>8) & 0xff;	// high 8 bit
	
	if (SendMsg() )
	{
		return TRUE;
	}
	return FALSE;*/
}

// origin setMotorgetpos
/*bool CDeltaUSBControl::SetMotorGetPosStatues(BYTE  bPWM[3], BYTE ctrlMotor, UINT16 AxisPos[3], BYTE & statu)
{

	
	m_send[0] = CTW_OPEN_FORCE;
	
	// change pwm clockwise 321
	//Clockwise321(bPWM);
//	Clockwise312(bPWM);

	m_send[1] = bPWM[0];
	m_send[2] = bPWM[1];
	m_send[3] = bPWM[2];
	m_send[4] = ctrlMotor;
	int i=0,j=0;
	//AxisPos[1]=100;

	if (SendMsg()&& GetMsg())
	{
		//i=m_res[1];
		//j=m_res[2]<<8;
		//AxisPos[0]= i&j;

		AxisPos[0] = m_res[1] | (m_res[2]<<8);
		AxisPos[1] = m_res[3] | (m_res[4]<<8);
		AxisPos[2] = m_res[5] | (m_res[6]<<8);
		statu= m_res[7];

		// AxisPos clockwise modification

		//Clockwise321(AxisPos);
		//Clockwise231(AxisPos);			// origin to rb. new value equal to origin sequence


		for (i=0;i<3;i++)
		{
			m_encoder[i]=AxisPos[i];	
			if (AxisPos[i]>50000)
				m_dRad[i] = -(double)(AxisPos[i]-65536)/EN_SCALAR/RD_RATIO*2*PI;
			else
				m_dRad[i] = -(double)AxisPos[i]/EN_SCALAR/RD_RATIO*2*PI;
		}
		m_state = statu;
		Encoder2handle(m_dRad,m_dHX,m_dHDConfig);		// calc Pos

		
		// coordination transformation
		Matrix3d matR;
		Vector3d vecHX;
		matR = Map<Matrix3d>(m_R[0]).transpose();			// col major needed to be transposed, m_R[0] point to the first element
		vecHX = matR*Map<Vector3d>(m_dHX);

		Map<Vector3d>(m_dHXTar,3) = vecHX;				// get back the vector value,tested suc in individual program

		return TRUE;
	}	
	return FALSE;	
}
*/

bool CDeltaUSBControl::SetMotorGetPosStatus(double dTorque[6], int &nErrCode)
{
	short shCurrent[6]={0};
	int i =0;
	memcpy(m_dT, dTorque, sizeof(m_dT));		// input the dTorque
	for(i=0;i<6;i++){
			//shCurrent[i] = dTorque[i]*T2CURRENT;		//get the current target value
		if(dTorque[i] > 0.1) dTorque[i] = 0.1;
		if(dTorque[i] < -0.1) dTorque[i] = -0.1;
		
		shCurrent[i] = dTorque[i]*::t2cur[i];
	}
	
	//shCurrent[4]= -shCurrent[4];			// change the direction for axis 5
	shCurrent[3] = -shCurrent[3];

	//if(CheckDAC(shCurrent))
	//	nErrCode = TORQUE_OVERRUN;
	Out_Packet[0] = CTW_PID;
	Out_Packet[1] =  shCurrent[0] & 0xff;
	Out_Packet[2] = (shCurrent[0]>>8) & 0xff;	// high 8 bit
	Out_Packet[3] =  shCurrent[1] & 0xff;
	Out_Packet[4] = (shCurrent[1]>>8) & 0xff;	// high 8 bit
	Out_Packet[5] =  shCurrent[2] & 0xff;
	Out_Packet[6] = (shCurrent[2]>>8) & 0xff;	// high 8 bit
	Out_Packet[7] =  shCurrent[3] & 0xff;
	Out_Packet[8] = (shCurrent[3]>>8) & 0xff;	// high 8 bit
	Out_Packet[9] =  shCurrent[4] & 0xff;
	Out_Packet[10] = (shCurrent[4]>>8) & 0xff;	// high 8 bit
	Out_Packet[11] =  shCurrent[5] & 0xff;
	Out_Packet[12] = (shCurrent[5]>>8) & 0xff;	// high 8 bit
	return IOMsg(nErrCode);	
}

bool CDeltaUSBControl::SetTorqueGetPosStatus(double dTorque[6], int &nErrCode)
{
	int i=0;
	short shCurrent[6]={0};

	memcpy(m_dTDisk, dTorque, sizeof(m_dTDisk));

/*	for(i=0;i<6;i++)			// to real torque
	{
		m_dT[i] = m_dTDisk[i]/RD_RATIO;
		shCurrent[i] = m_dT[i]*T2CURRENT;
	}
	if(CheckDAC(shCurrent))
		nErrCode = TORQUE_OVERRUN;
	Out_Packet[0] = CTW_PID;
	Out_Packet[1] =  shCurrent[0] & 0xff;
	Out_Packet[2] = (shCurrent[0]>>8) & 0xff;	// high 8 bit
	Out_Packet[3] =  shCurrent[1] & 0xff;
	Out_Packet[4] = (shCurrent[1]>>8) & 0xff;	// high 8 bit
	Out_Packet[5] =  shCurrent[2] & 0xff;
	Out_Packet[6] = (shCurrent[2]>>8) & 0xff;	// high 8 bit
	Out_Packet[7] =  shCurrent[3] & 0xff;
	Out_Packet[8] = (shCurrent[3]>>8) & 0xff;	// high 8 bit
	Out_Packet[9] =  shCurrent[4] & 0xff;
	Out_Packet[10] = (shCurrent[4]>>8) & 0xff;	// high 8 bit
	Out_Packet[11] =  shCurrent[5] & 0xff;
	Out_Packet[12] = (shCurrent[5]>>8) & 0xff;	// high 8 bit
	return IOMsg(nErrCode);		*/
	for(i=0;i<6;i++)
	{
		m_dT[i] = m_dTDisk[i]/(::trRatio[i]);
	}
	return SetMotorGetPosStatus(m_dT, nErrCode);
}
void CDeltaUSBControl::ReleaseDev(int nInd)
{
	int nErrCode;
	memset(m_dT,0,sizeof(m_dT));
	SetMotorGetPosStatus(m_dT, nErrCode);
	CH375CloseDevice( nInd );
	/*
	SI_SetTimeouts(INFINITE, INFINITE);
	SI_Close(m_hUSBDevice); 
	m_bReadError =FALSE;
	m_bWriteError = FALSE;
	return TRUE;
	*/
}

/*bool CDeltaUSBControl::SendMsg(void)
{
	DWORD dwBytesSucceed = 0;
	DWORD dwBytesWriteRequest = 7;			// desire of reading/writing size
	DWORD dwBytesReadRequest = 8;	

	SI_STATUS status = SI_SUCCESS;
	// Confirm an error hasn't already occurred
	if ( (m_bReadError == TRUE) || (m_bWriteError == TRUE) )
	{
		// Call base timer
		// CDialog::OnTimer(nIDEvent);

		// Further processing not possible
		return FALSE;		
	}
	
	status = SI_Write(m_hUSBDevice, m_send, dwBytesWriteRequest, &dwBytesSucceed);

	if (dwBytesSucceed != dwBytesWriteRequest || status != SI_SUCCESS)
	{
		m_bWriteError = TRUE;	// Note: Set error flag immediately so that multiple 
		return FALSE;

	}
	return TRUE;
}

bool CDeltaUSBControl::GetMsg(void)
{
	DWORD dwBytesSucceed = 0;
	DWORD dwBytesWriteRequest = 7;			// desire of reading/writing size
	DWORD dwBytesReadRequest = 8;	

	SI_STATUS status = SI_SUCCESS;
	// Confirm an error hasn't already occurred
	if ( (m_bReadError == TRUE) || (m_bWriteError == TRUE) )
	{
		// Call base timer
		// CDialog::OnTimer(nIDEvent);

		// Further processing not possible
		return FALSE;		
	}

	memset(m_res,0, sizeof(m_res));
	
	status = SI_Read(m_hUSBDevice, m_res,dwBytesReadRequest, &dwBytesSucceed);

	if (((dwBytesSucceed != dwBytesReadRequest) && (m_bReadError == FALSE)) || status != SI_SUCCESS)
	{
		m_bReadError = TRUE;	// Note: Set error flag immediately so that multiple 
		// message boxes do not queue up.
		return FALSE;
	}
	return TRUE;
}
*/
/*
bool  CDeltaUSBControl::SetFGetPosStatues(double dF[3],double dPos[3], BYTE & statu,double dT[3],bool & blOutRange)
{
	BYTE	bPwm[3]={0};
	BYTE	ctrlMotor = 0;
	UINT16	AxisPos[3]={0};
	BYTE	statues = 0;
	BOOL	outRange=FALSE;
	
	F2PwmPara(bPwm,ctrlMotor,dF,dT);		// m_dF
	blOutRange= !CheckPwm(bPwm);
	//Clockwise231(bPwm);
	if (SetMotorGetPosStatues(bPwm,ctrlMotor,AxisPos,statu))	// m_dHXTar res
	{
		GetPos(dPos);			// feedback the pos infor
		return TRUE;
	}
	return FALSE;

}*/
bool CDeltaUSBControl::SetFTGetPosStatus(double dF[3],double dTorque[3],int& nErrCode)
{
	int i=0;
	//memset(bPWM,0,sizeof(bPWM));	//clear
	// BCKWOARD for  handle motor forward	

	// calcu the target Force
	Matrix3d matR;
	Vector3d vecF;
	Matrix3d matRBase; // matRBase is to FD coordinate ** new part
	matRBase<< 0,0,-1,
		0, -1,0,
		-1, 0, 0;

	matR = Map<Matrix3d>(m_R[0]);// As the array is row majored, col majored needed to be transposed. Without transpose, the result equal to the R transposition or inverse( for rotaton matrix)
									
	// vecF = matR*Map<Vector3d>(dF);
	vecF = matRBase.transpose()*matR*Map<Vector3d>(dF);

	Map<Vector3d>(m_dF,3) = vecF;				// get back the vector value,tested suc in individual program. 
	// here for the motor setting point *************
	//Force2torque(m_dF,m_dT,m_dHX,m_dHDConfig);	// all the coordinate in origin. Put the result torque into the first 3 array element. 
	//Clockwise231(m_dT);			// to the device sequence
	// Clockwise312(m_dT);			// rb to up 
	//memcpy(m_dT+3,dTorque,sizeof(double)*3); // get the rotation torque into the last 3 element.
	//return SetMotorGetPosStatus(m_dT, nErrCode);

	// here for real torque**************************
	Force2torque(m_dF,m_dTDisk,m_dHX,m_dHDConfig);
	memcpy(m_dTDisk+3,dTorque,sizeof(double)*3); // get the rotation torque into the last 3 element.
	return SetTorqueGetPosStatus(m_dTDisk, nErrCode);
}

void	 CDeltaUSBControl::GetPos(double pos[3])
{
	//memcpy(pos,m_dHX,sizeof(m_dHX));
	memcpy(pos,m_dHXTar,sizeof(m_dHXTar));		// after transportation
}
/*void	CDeltaUSBControl::F2PwmPara(BYTE bPWM[3],BYTE &ctrlMotor,double dF[3],double dT[3])
{
	int i=0;
	//memset(bPWM,0,sizeof(bPWM));	//clear
	// BCKWOARD for  handle motor forward
	ctrlMotor = 0;

	// calcu the target Force
	Matrix3d matR;
	Vector3d vecF;
	matR = Map<Matrix3d>(m_R[0]);			// col major needed to be transposed, Without transpose, the result equal to the R transposition
	vecF = matR*Map<Vector3d>(dF);

	Map<Vector3d>(m_dF,3) = vecF;				// get back the vector value,tested suc in individual program
	Force2torque(m_dF,m_dT,m_dHX,m_dHDConfig);	// all the coordinate in origin
	//Clockwise231(m_dT);			// to the device sequence
	// Clockwise312(m_dT);			// rb to up 
	memcpy(dT,m_dT,sizeof(m_dT));

	for (i=0;i<3;i++)
	{		
		if(m_dT[i] != 0)
		{
			ctrlMotor |= BIT_MOTOR1_EN<<i;
			if (m_dT[i]<0)
			{
				bPWM[i] = -m_dT[i]*SC_TOR2PWM;			// convert to positive	
				ctrlMotor |= BIT_MOTOR1_FOR<<i;
			}
			else
			{
				bPWM[i] = m_dT[i]*SC_TOR2PWM;						
				ctrlMotor &= ~(BIT_MOTOR1_FOR<<i);
			}
		}
	}	
	// turn the torque sequence 

}*/
void	CDeltaUSBControl::GetRad(double dRad[6])
{
	memcpy(dRad,m_dRad,sizeof(m_dRad));
}
void	CDeltaUSBControl::GetRotationRad(double dRad[3])
{
	memcpy(dRad,m_dRad+3,sizeof(double)*3);		// last three element in m_dRad;
}
void	CDeltaUSBControl::GetState(unsigned char & state)
{
	state = m_state;
}

 bool	CDeltaUSBControl::CheckDAC(short shTorq[6])
{
	int i =0;
	double dFactor = 1;		// output to origin factor
	bool blOutRange =FALSE;
	//unsigned short u16Max = 0;
	short shMax = 0;
	for(i=0;i<6;i++)
	{
		if(shMax<abs(shTorq[i]))
			shMax = shTorq[i];
	}
	if(shMax> SP_SAFE_MAX)
	{
		blOutRange = true;
		dFactor = (double)SP_SAFE_MAX/shMax;
		for(i=0;i<6;i++)
			shTorq[i] = shTorq[i]* dFactor;		// set back to normal
	}
	return !blOutRange;
} 
bool CDeltaUSBControl::IOMsg(int& nErrCode)
{
	unsigned long	mTotal= 0, mLength = 0;
	int	i=0;
	nErrCode = 0;			// non error firstly.
	mTotal = CMD_LENGTH_HOST2DEV;
	// check interval time
	/*
	do
	{
		QueryPerformanceCounter(&m_liPerfNow);			
	}while((m_liPerfNow.QuadPart - m_liPerfStart.QuadPart)*1000000/m_liPerfFreq.QuadPart<INTERVAL_US);
	QueryPerformanceCounter(&m_liPerfStart);	
	*/
	// blocking write
	 if ( CH375WriteData( m_nDeviceNum, Out_Packet, &mTotal ) )  // 发送成功
	 {
	//while(!CH375WriteData( m_nDeviceNum, Out_Packet, &mTotal)); 
		 mLength = CMD_LENGTH_DEV2HOST;
		//if ( mTestCount == 0 ) Sleep( 200 );  // 考虑到之前单片机准备上传的数据可能未被计算机取走,导致首次回传有可能直接读到之前的数据而不是本次数据的取反,所以首次回传先等待单片机准备好取反数据
		if ( CH375ReadData( 0, In_Packet, &mLength ) )  // 接收成功
		{
			if ( mLength != CMD_LENGTH_DEV2HOST || mLength==0 ) {  // 长度错误
			//	mErrCnt++;
			//	printf( "S1-T%0ld-C%ld return length error: %ld (%ld)\n", mStep, mTestCount, mLength, mTotal );
				nErrCode = READ_ERROR;
			}
			else {
				//printf("The value returned is: %d %d %d %d %d %d\n", mReadBuf[0],mReadBuf[1],mReadBuf[2],mReadBuf[3],mReadBuf[4],mReadBuf[5]);
				switch (In_Packet[0])
				{
				case DEV_POS_STA:
				//	cout<<"dev state:"<<endl;
					{
						for (i=0;i<6;i++)
						{
							m_encoder[i] = In_Packet[2*i+1]|((In_Packet[2*i+2]<<8));
							//cout<<"enc1:	"<<encoder[i]<<endl;
							m_dRad[i] = -(double)(m_encoder[i]-32768)/EN_SCALAR/::trRatio[i]*2*PI + trBias[i];
							/*
							if (m_encoder[i]>32767)
								m_dRad[i] = -(double)(m_encoder[i]-65536)/EN_SCALAR/::trRatio[i]*2*PI;
							else
								m_dRad[i] = -(double)m_encoder[i]/EN_SCALAR/::trRatio[i]*2*PI;												
							*/
						}
						m_dRad[4] = - m_dRad[4]; // direction changing
						m_dRad[5] = m_dRad[5]*2;	// 4096 to 2048 scalar changing
						m_state = In_Packet[13];	

						Encoder2handle(m_dRad,m_dHX,m_dHDConfig);		// calc Pos


						// coordination transformation
						Matrix3d matR;
						Matrix3d matRBase; // matRBase is to FD coordinate ** new part
						matRBase<< 0,0,-1,
							0, -1,0,
							-1, 0, 0;

						Vector3d vecHX;
						matR = Map<Matrix3d>(m_R[0]).transpose();			// col major needed to be transposed, m_R[0] point to the first element
						//vecHX = matR*Map<Vector3d>(m_dHX);
						vecHX = matR*matRBase*Map<Vector3d>(m_dHX);

						Map<Vector3d>(m_dHXTar,3) = vecHX;				// get back the vector value,tested suc in individual program

						//	cout<<"state word:	"<<(int)devState<<endl;
						break;
					}
				default:
					//cout<<"no designed method for	"<<(int)In_Packet[0]<<endl;
					nErrCode = PAC_TYPE_ERROR;
					return FALSE;
					break;
				}				
			}
		}
		else {  // 读操作失败
			//mErrCnt++;
			//printf( "S1-T%0ld-C%ld CH375ReadData return error, length=%ld\n", mStep, mTestCount, mTotal );
			nErrCode = READ_ERROR;
			return FALSE;
		}
	}
	else {  // 写操作失败
		//mErrCnt++;
		//printf( "S1-T%0ld-C%ld CH375WriteData return error, length=%ld\n", mStep, mTestCount, mTotal );
		nErrCode = WRITE_ERROR;
		return FALSE;
	}
	return TRUE;
}
int  CDeltaUSBControl::testPri()
{
	return 1;
}
void CDeltaUSBControl::GetTorque(double T[6])
{
	//memcpy(pos,m_dHXTar,sizeof(m_dHXTar));		// after transportation
	//memcpy(T, m_dT, sizeof(m_dT));
	memcpy(T, m_dTDisk, sizeof(m_dTDisk));			
}
/*template<class T>
void CDeltaUSBControl::Clockwise321(T array[3])
{
	T arrayT;
	memcpy(arrayT,array,sizeof(array));
}*/

