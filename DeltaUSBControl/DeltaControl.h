#pragma once

// get in the lib and the h file

#ifdef DLL_DELTA_CONTROL
#else
#define DLL_DELTA_CONTROL _declspec(dllimport)
#endif

#include <vector>
using namespace std;

// command format
//device state 
#define ZEROBIT 	0x0001			// 是否0位置
#define BASEBTN 	0x0002
#define HANDLEBTN	0X0004

#define	MAX_FORCE	10			// N
#define	MAX_TORQUE	0.4			// axis torque
#define	MAX_SPIN_TORQUE	0.028	// spoon pin  max torque
#define	MAX_TORQUE_AXIS4 0.135
#define MAX_TORQUE_AXIS5 0.125
#define	MAX_TORQUE_AXIS6 0.049		// should be changed higher
//#define	MAX_TORQUE_AXIS6 0.080



// error code define
#define	READ_ERROR		1
#define	WRITE_ERROR		2
#define	PAC_TYPE_ERROR	3
#define	TORQUE_OVERRUN	4

// typedef	std::vector<string>	DeviceList;

class DLL_DELTA_CONTROL CDeltaUSBControl
{
public:
	CDeltaUSBControl(void);
	~CDeltaUSBControl(void);

private:
	HANDLE		m_hUSBDevice;
	BOOL		m_bReadError;
	BOOL		m_bWriteError;
	// USB data buffer
	//USB_iobuf	m_IObuffer;
	// DeviceList	m_DeviceList;
	unsigned int	m_nDeviceNum;		// 默认是0，可以进行开发
	//CString		m_sDeviceName;

	unsigned char Out_Packet[15];     // Last packet received from host
	unsigned char In_Packet[15];		// to keep it easy to trace, the buffer name keep the same with the device's.

	// parameter
	int				m_encoder[6];
	unsigned char	m_state;
	double			m_dRad[6];

	double m_dHX[3];				// 手柄位置,origin
	double m_dHXTar[3];				// target coordinates
	double m_dHDConfig[4];		// 手柄构型
	double m_dF[3],m_dFTar[3];			 // 力	
	double m_dT[6];			// first 3 indicate inverse kinemic result for input force F. As spin torque
	double m_dTDisk[6];		// the disk torque

	double m_R[3][3];		// Rotation matrix for the coordinate transformation. default is identity. From the origin to the Tar
							// In many cases, it's equal to 
							//	-1 0 0 
							//	0 -1 0
							//	0 0	-1
	// counter member
	LARGE_INTEGER m_liPerfStart; 
	LARGE_INTEGER m_liPerfNow;
	LARGE_INTEGER m_liPerfFreq;

	//BOOL		GetDeviceList();
	//BOOL		ConnectDevice();
	//BOOL		Set3AxisPos();
	//BOOL		SetMotorGetPos();

	// BOOL GetDeviceList(void);

public:
	/*************************************************
	Function:       // ConnectDevice
	Description:    // connect the nInd device to host. Set the index number of the controller device. 
	Calls:          // 
	Input:          // nInd; the devcie number from 0 to 65535, default 0;
	Output:         // 
	Return:         // return true if succeed ; else false
	Others:         // 其它说明
	*************************************************/
	bool ConnectDevice(int nInd=0);

	/*************************************************
	Function:       // Set6AxisPos
	Description:    // set the encoder data. for debug purpose only temporally.  
	Calls:          // 
	Input:          // U16AxisPos; the data array indicating the target encoder value. Read the device encoder and state value.
	Output:         // 
	Return:         // return true if succeed ; else false
	Others:         // 其它说明
	*************************************************/
	bool Set6AxisPos(unsigned short U16AxisPos[6],int & nErrCode);

	/*************************************************
	Function:       // SetMotorGetPosStatues
	Description:    // set the pwm ctrlMotor message to control the motor,get back the encoder position and dev statu
					// the CDeltaUSBControl object will save the encoder value and calc the Hx automatically; Each time called, the HX 
					// value will be refreshed and GetPos can feed back the latest pos infor.
	Calls:          // 
	Input:          // pwm; the pwm value for motors, 0 to 80; ctrlMotor, enable bit and forward bit control word;
	Output:         // AxisPos: 3 encoder value; statu: the dev statu; 
	Return:         // return true if succeed ;
	Others:         // 其它说明
	*************************************************/
	// bool SetMotorGetPosStatues(BYTE  bPWM[3], BYTE ctrlMotor, UINT16  AxisPos[3],BYTE & statu);
	/*************************************************
	Function:       // SetMotorGetPosStatues
	Description:    // set the torque for the motor,get back the encoder position and dev statu, save them in object member parameters. 
	// the CDeltaUSBControl object will save the encoder value and calculate the Hx automatically; Each time called, the HX 
	// value will be refreshed and GetPos can feed back the latest pos infor.
	Calls:          // 
	Input:          // dTorque; the torque value for each motors, 0 to 80; ctrlMotor, enable bit and forward bit control word;
	Output:         // ErrCode the error code 
	Return:         // return true if succeed ;
	Others:         // 其它说明
	*************************************************/
	bool SetMotorGetPosStatus(double dTorque[6], int &nErrCode);

	/*************************************************
	Function:       // SetTorqueGetPosStatus
	Description:    // set the torque for the device axis, the transmission ratio is applied to each motor. The return value is the actual
	torque applied to the axis. 
	// the CDeltaUSBControl object will save the encoder value and calculate the Hx automatically; Each time called, the HX 
	// value will be refreshed and GetPos can feed back the latest pos infor.
	Calls:          // 
	Input:          // dTorque; the torque value for each motors, 0 to 80; ctrlMotor, enable bit and forward bit control word;
	Output:         // ErrCode the error code 
	Return:         // return true if succeed ;
	Others:         // 其它说明
	*************************************************/
	bool SetTorqueGetPosStatus(double dTorque[6], int &nErrCode);
	
	/*************************************************
	Function:       // IOMsg
	Description:    // send out the Out_packet and receive the device In_Packet, refresh the pos and state, and rad infor;
					// Basic script function, the parameters needed are pre-defined.
	Calls:          // 
	Input:          // 
	Output:         // the error code; 1 for reading error. 2 for writting error
	Return:         // return true if succeed ;
	Others:         // 其它说明
	*************************************************/
	bool IOMsg(int& nErrCode);

	// string m_sDeviceName;

	/*************************************************
	Function:       // ReleaseDev
	Description:    // release device of nInd;
	Calls:          // 
	Input:          // nInd: Device index.
	Output:         // 
	Return:         // return true if succeed ;
	Others:         // 其它说明
	*************************************************/
	void ReleaseDev(int nInd=0);
	// bool ReleaseDev(void);
	//bool SendMsg(void);
	//bool GetMsg(void);

	// origin
	/*************************************************
	Function:       // SetFGetPosStatues
	Description:    // set the target force to the dev. pos and statu will be feeded back; the calcu result torque for 3 axis will be 
					// feeded back for check; 
	Calls:          // 
	Input:          // dF: the 3 coordinat forces;
	Output:         // 
	Return:         // return true if succeed ;
	Others:         // 其它说明
	*************************************************/
	//bool SetFGetPosStatues(double dF[3],double dPos[3], BYTE & statu,double dT[3],bool & blOutRange);

	/*************************************************
	Function:       // SetFTGetPosStatues
	Description:    // set the target force and torque to the dev. achieve the encoder and state information. 
	Calls:          // 
	Input:          // dF: the 3 coordinate forces; dTorque, the rotation structure torque.
	Output:         // nErrCode, error code.
	Return:         // return true if succeed ;
	Others:         // 其它说明
	*************************************************/
	bool SetFTGetPosStatus(double dF[3],double dTorque[3],int& nErrCode);

	/*************************************************
	Function:       // GetPos
	Description:    // Get the position saved in the  CDeltaUSBControl object
	Calls:          // 
	Input:          // pos[3]: coordination in handle reference frame
	Output:         // 
	Return:         // NA;
	Others:         // 其它说明
	*************************************************/
	void	GetPos(double pos[3]);
	/*************************************************
	Function:       // F2PwmPara
	Description:    // F is put in then the control para is generated. An interface for the Torque set to the 
					// feeded back for check; the torque is opposite to the motor so the motor direction is reversed.
	Calls:          // 
	Input:          // dF: the 3 coordinat forces;
	Output:         // 
	Return:         // return true if succeed ;
	Others:         // 其它说明	// 
	*************************************************/	
	// void	F2PwmPara(BYTE bPWM[3],BYTE &ctrlMotor,double dF[3], double dT[3]);
	/*************************************************
	Function:       // GetPos
	Description:    // Get the radian value saved in the  CDeltaUSBControl object
	Calls:          // 
	Input:          // 
	Output:         // dRad[3]: three axis rotation radin.
	Return:         // NA;
	Others:         // 其它说明
	*************************************************/
	void	GetRad(double dRad[6]);

	/*************************************************
	Function:       // GetRotationAngle
	Description:    // Get the radian structure angle in radian saved in the  CDeltaUSBControl object
	Calls:          // 
	Input:          // 
	Output:         // dRad[3]: three axis rotation radin.
	Return:         // NA;
	Others:         // 其它说明
	*************************************************/
	void	GetRotationRad(double dRad[3]);

	/*************************************************
	Function:       // GetState
	Description:    // Get the state of the device
	Calls:          // 
	Input:          // 
	Output:         // dRad[3]: three axis rotation radin.
	Return:         // NA;
	Others:         // 其它说明
	*************************************************/
	void	GetState(unsigned char & state);

	/*************************************************
	Function:       // CheckDAC
	Description:    // check the DAC value if the value out of the max torlerance pwm value; set the outrange value to the max threshold and 
					// shrink other force to the same scale making the output uniform
	Calls:          // 
	Input:          // pwm; the pwm value for check;
	Output:         // 
	Return:         // return true if no pwm out of range ;else false;
	Others:         // 其它说明
	*************************************************/
	bool	CheckDAC(short shTorq[6]);

	/*************************************************
	Function:       // CheckPwm
	Description:    // check the pwm value if the value out of the max torlerance pwm value; set the outrange value to the max threshold and 
	// shrink other force to the same scale making the output uniform
	Calls:          // 
	Input:          // pwm; the pwm value for check;
	Output:         // 
	Return:         // return true if no pwm out of range ;else false;
	Others:         // 其它说明
	*************************************************/
	template<class T>
	void Clockwise321(T array[3])
	{
		T arrayT[3];
		int i=0;
		memcpy(arrayT,array,sizeof(T)*3);
		for (i=0;i<3;i++)
		{
			array[i] = arrayT[2-i];
		}
	}
	template<class T>
	void Clockwise231(T array[3])		// the result  equal to the  input  index 231使用者， for get the input
	{
		T arrayT[3];
		int i=0;
		memcpy(arrayT,array,sizeof(T)*3);
		for (i=0;i<3;i++)
		{
			array[i] = arrayT[(i+1)%3];
		}
	}
	template<class T>
	void Clockwise312(T array[3])		// output equal to what sequence of input, trans to the output
	{
		T arrayT[3];
		int i=0;
		memcpy(arrayT,array,sizeof(T)*3);
		for (i=0;i<3;i++)
		{
			array[i] = arrayT[(i+2)%3];
		}
	}

	/*************************************************
	Function:       // SetRotation
	Description:    // set the Rotation matrix from the origin frame to the target frame
	Calls:          // 
	Input:          // rotation matrix in form of double[3][3]
	Output:         // 
	Return:         // return true if no pwm out of range ;else false;
	Others:         // 其它说明
	*************************************************/
	void inline SetRotation(double R[3][3])
	{
		memcpy(*m_R,*R,sizeof(m_R));
	}

		/*************************************************
	Function:       // Get the torque value setted to device.
	Description:    // set the Rotation matrix from the origin frame to the target frame
	Calls:          // 
	Input:          // 
	Output:         // The real torque applied to each axis.
	Return:         // return true if no pwm out of range ;else false;
	Others:         // 其它说明
	*************************************************/
	void	GetTorque(double T[6]);		// should be pri

	private:
		int testPri();

	/*************************************************
	Function:       // CalcuCoord
	Description:    // calculate the target coordinate 
	Calls:          // 
	Input:          // rotation matrix in form of double[3][3]
	Output:         // 
	Return:         // return true if no pwm out of range ;else false;
	Others:         // 其它说明
	*************************************************/
	//void CalcuCoord();

	/*************************************************
	Function:       // CalcuOriForce
	Description:    // According to the target force, calculate the force in the origin frame.
	Calls:          // 
	Input:          // rotation matrix in form of double[3][3]
	Output:         // 
	Return:         // return true if no pwm out of range ;else false;
	Others:         // 其它说明
	*************************************************/
	//void CalcuOriForce();
};
