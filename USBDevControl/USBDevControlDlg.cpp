// USBDevControlDlg.cpp : 实现文件
//

#include "stdafx.h"
#include "USBDevControl.h"
#include "USBDevControlDlg.h"
#include <afxmt.h>
//#include <WinSDDKVer.h>
#include <windows.h>

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// 用于应用程序“关于”菜单项的 CAboutDlg 对话框
MMRESULT g_wTimerID = 0;

// thread function and variable
double*		pArr=NULL;
UINT		ThreadFunc(LPVOID lpParam); 
bool		blServoTh=0;
CCriticalSection g_clsCriticalSection;
DWORD WINAPI Thread1Proc(  LPVOID lpParameter);
  

class CAboutDlg : public CDialog
{
public:
	CAboutDlg();

// 对话框数据
	enum { IDD = IDD_ABOUTBOX };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

// 实现
protected:
	DECLARE_MESSAGE_MAP()
public:
	//afx_msg void OnTimer(UINT_PTR nIDEvent);
};

CAboutDlg::CAboutDlg() : CDialog(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialog)
	ON_WM_TIMER()
END_MESSAGE_MAP()

// CUSBDevControlDlg 对话框

CUSBDevControlDlg::CUSBDevControlDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CUSBDevControlDlg::IDD, pParent)
	, m_nForceCtrl(0)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
	
	memset(m_dF, 0, sizeof(double)*3);
	memset(m_dRad,0, sizeof(m_dRad));
	memset(m_dT,0,sizeof(m_dT));

	memset(m_dTest,0,sizeof(m_dTest));		// test array
	memset(m_dRadServo,0,sizeof(m_dRadServo));
	/*m_HDConfig[0] = 0.88; // Lb
	m_HDConfig[1] = 0.44; // La
	m_HDConfig[2] = 0.3; // R
	m_HDConfig[3] = 0.1;	// r*/

	// new parameters
	m_dHDConfig[0] = 0.146; // Lb
	m_dHDConfig[1] = 0.07; // La
	m_dHDConfig[2] = 0.0715; // R
	m_dHDConfig[3] = 0.037;	// r

	// pid initial
	m_kp = 0.1;
	m_ki = 0.0001;
	//m_ki =0;
	m_up =0 ;
	// m_ui[6];
	memset(m_ui,0,sizeof(m_ui));
	//double	m_outpre[6];
	//double	m_outnow[6];
	memset(m_outpre,0,sizeof(m_outpre));
	memset(m_outnow,0,sizeof(m_outnow));
	m_outmax = 0.1;
	
	//double R[3][3] = {0,0,-1,0,-1,0,-1,0,0};// user direction. Outward x.Up z.
	//m_usbDev.SetRotation(R);
	/*
	hThread1 = CreateThread(NULL,       //默认安全级别  
                            0,          //默认栈大小  
                            Thread1Proc,//线程函数   
                            NULL ,       //函数没有参数  
                            0,          //创建后直接运行  
                            NULL);      //线程标识，不需要
							*/
}

void CUSBDevControlDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_SLID_MOTOR1, m_slidMotor1);
	//DDX_Control(pDX, IDC_SPIN_MOTOR1, m_spinMotor1);
	//DDX_Control(pDX, IDC_SPIN_MOTOR2, m_spinMotor2);
	//DDX_Control(pDX, IDC_SPIN_MOTOR3, m_spinMotor3);
	DDX_Control(pDX, IDC_SLID_MOTOR2, m_slidMotor2);
	DDX_Control(pDX, IDC_SLID_MOTOR3, m_slidMotor3);
	//DDX_Radio(pDX, IDC_RAD_MOTOR1, m_nMotor1Dir);
	//DDX_Radio(pDX, IDC_RAD_MOTOR2, m_nMotor2Dir);
	//DDX_Radio(pDX, IDC_RAD_MOTOR3, m_nMotor3Dir);
	//DDX_Check(pDX, IDC_CHK_MOTOR1, m_blMotor1);
	//DDX_Check(pDX, IDC_CHK_MOTOR2, m_blMotor2);
	//DDX_Check(pDX, IDC_CHK_MOTOR3, m_blMotor3);
	//	DDX_Text(pDX, IDC_ED_ENC1, m_nEncoder1);
	//	DDX_Text(pDX, IDC_ED_ENC2, m_nEncoder2);
	//	DDX_Text(pDX, IDC_ED_ENC3, m_nEncoder3);
	DDX_Control(pDX, IDC_LIST_DEV, m_listDev);
	DDX_Control(pDX, IDC_LIST_FORCE, m_listForce);
	DDX_Control(pDX, IDC_SLID_FX, m_slidFX);
	DDX_Control(pDX, IDC_SLID_FY, m_slidFY);
	DDX_Control(pDX, IDC_SLID_FZ, m_slidFZ);
	DDX_Control(pDX, IDC_SLID_MOTOR4, m_slidMotor4);
	DDX_Control(pDX, IDC_SLID_MOTOR5, m_slidMotor5);
	DDX_Control(pDX, IDC_SLID_MOTOR6, m_slidMotor6);
	DDX_Radio(pDX, IDC_RAD_F, m_nForceCtrl);
	//DDX_Control(pDX, IDC_SLID_SERVOM1, m_slidServoM1);
	//DDX_Control(pDX, IDC_SLID_SERVOM2, m_slidServoM2);
	//DDX_Control(pDX, IDC_SLID_SERVOM3, m_slidServoM3);
	//DDX_Control(pDX, IDC_SLID_SERVOM4, m_slidServoM4);
	//DDX_Control(pDX, IDC_SLID_SERVOM5, m_slidServoM5);
	//DDX_Control(pDX, IDC_SLID_SERVOM6, m_slidServoM6);
}

BEGIN_MESSAGE_MAP(CUSBDevControlDlg, CDialog)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	//}}AFX_MSG_MAP
	ON_BN_CLICKED(IDCANCEL, &CUSBDevControlDlg::OnBnClickedCancel)	
	ON_WM_HSCROLL()
	ON_BN_CLICKED(IDC_BTN_CONNECT_DEV, &CUSBDevControlDlg::OnBnClickedBtnConnectDev)
	ON_BN_CLICKED(IDC_BTN_IO_MSG, &CUSBDevControlDlg::OnBnClickedBtnIoMsg)
	ON_BN_CLICKED(IDC_BTN_TEST1, &CUSBDevControlDlg::OnBnClickedBtnTest1)
//	ON_BN_CLICKED(IDC_BTN_IO_MSG2, &CUSBDevControlDlg::OnBnClickedBtnIoMsg2)
//	ON_BN_CLICKED(IDC_BTN_SEQ_COMMU, &CUSBDevControlDlg::OnBnClickedBtnSeqCommu)
	//ON_WM_TIMER()
	//ON_BN_CLICKED(IDC_CHECK2, &CUSBDevControlDlg::OnBnClickedCheck2)
	ON_BN_CLICKED(IDC_BTN_MM_TIMER, &CUSBDevControlDlg::OnBnClickedBtnMmTimer)
	ON_MESSAGE(WM_UPDATE_IN_MESSAGE,OnUpdateInMessage)
	ON_BN_CLICKED(IDC_CHK_FOR, &CUSBDevControlDlg::OnBnClickedChkFor)
//	ON_BN_CLICKED(IDC_BTN_SETF, &CUSBDevControlDlg::OnBnClickedBtnSetf)
//	ON_BN_CLICKED(IDC_BTN_CLEAR_F, &CUSBDevControlDlg::OnBnClickedBtnClearF)
ON_BN_CLICKED(IDC_RAD_F, &CUSBDevControlDlg::OnBnClickedRadF)
ON_BN_CLICKED(IDC_RAD_ACT, &CUSBDevControlDlg::OnBnClickedRadAct)
ON_BN_CLICKED(IDC_BTN_TIMER1, &CUSBDevControlDlg::OnBnClickedBtnTimer1)
ON_WM_TIMER()
//ON_BN_CLICKED(IDC_BTN_BG_TH, &CUSBDevControlDlg::OnBnClickedBtnBgTh)
//ON_BN_CLICKED(IDC_BTN_SET_SERVO, &CUSBDevControlDlg::OnBnClickedBtnSetServo)
//ON_BN_CLICKED(IDC_BTN_BG_TH2, &CUSBDevControlDlg::OnBnClickedBtnBgTh2)
END_MESSAGE_MAP()


// CUSBDevControlDlg 消息处理程序

BOOL CUSBDevControlDlg::OnInitDialog()
{
	CDialog::OnInitDialog();

	// 将“关于...”菜单项添加到系统菜单中。

	// IDM_ABOUTBOX 必须在系统命令范围内。
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		CString strAboutMenu;
		strAboutMenu.LoadString(IDS_ABOUTBOX);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// 设置此对话框的图标。当应用程序主窗口不是对话框时，框架将自动
	//  执行此操作
	SetIcon(m_hIcon, TRUE);			// 设置大图标
	SetIcon(m_hIcon, FALSE);		// 设置小图标

	// TODO: 在此添加额外的初始化代码

	//SetDlgItemText(IDC_ED_MOTOR1,L"0");
	SetDlgItemInt(IDC_ED_FDB_ENC1,5000);
	SetDlgItemInt(IDC_ED_FDB_ENC2,5000);
	SetDlgItemInt(IDC_ED_FDB_ENC3,5000);

	SetDlgItemInt(IDC_ED_FX,0);
	SetDlgItemInt(IDC_ED_FY,0);
	SetDlgItemInt(IDC_ED_FZ,0);

	CRect listRect;
	int nTest;
	m_listDev.GetWindowRect(&listRect);
	nTest=m_listDev.InsertColumn(0, _T("Parameters"), LVCFMT_LEFT, listRect.Width()*3/8);	// suc 1, false -1;
	nTest=m_listDev.InsertColumn(1, _T("Value"), LVCFMT_LEFT, listRect.Width()*3/8);
	nTest=m_listDev.InsertColumn(2, _T("Unit"), LVCFMT_CENTER, listRect.Width()/4);
	
	m_listDev.InsertItem(0,L"X");
	m_listDev.SetItemText(0,2,_T("mm"));
	m_listDev.InsertItem(1,_T("Y"));
	m_listDev.SetItemText(1,2,_T("mm"));
	m_listDev.InsertItem(2,_T("Z"));
	m_listDev.SetItemText(2,2,_T("mm"));

	m_listDev.InsertItem(3,L"Enc1");
	m_listDev.SetItemText(3,2,_T("radian"));
	m_listDev.InsertItem(4,_T("Enc2"));
	m_listDev.SetItemText(4,2,_T("radian"));
	m_listDev.InsertItem(5,_T("Enc3"));
	m_listDev.SetItemText(5,2,_T("radian"));

	m_listDev.InsertItem(6,L"Enc4");
	m_listDev.SetItemText(6,2,_T("radian"));
	m_listDev.InsertItem(7,_T("Enc5"));
	m_listDev.SetItemText(7,2,_T("radian"));
	m_listDev.InsertItem(8,_T("Enc6"));
	m_listDev.SetItemText(8,2,_T("radian"));

	m_listDev.InsertItem(9,_T("State"));

	// force list control
	m_listForce.GetWindowRect(&listRect);
	nTest = m_listForce.InsertColumn(0, L"Parameters",LVCFMT_LEFT, listRect.Width()*3/8);
	nTest = m_listForce.InsertColumn(1, _T("Value"), LVCFMT_LEFT, listRect.Width()*3/8);
	nTest = m_listForce.InsertColumn(2, _T("Unit"), LVCFMT_CENTER, listRect.Width()/4);

	m_listForce.InsertItem(0,L"FX");
	m_listForce.SetItemText(0,2,_T("N"));
	m_listForce.InsertItem(1,_T("FY"));
	m_listForce.SetItemText(1,2,_T("N"));
	m_listForce.InsertItem(2,_T("FZ"));
	m_listForce.SetItemText(2,2,_T("N"));

	m_listForce.InsertItem(3,L"Torque1");
	m_listForce.SetItemText(3,2,_T("Nm"));
	m_listForce.InsertItem(4,_T("Torque2"));
	m_listForce.SetItemText(4,2,_T("Nm"));
	m_listForce.InsertItem(5,_T("Torque3"));
	m_listForce.SetItemText(5,2,_T("Nm"));

	m_listForce.InsertItem(6,L"Torque4");
	m_listForce.SetItemText(6,2,_T("Nm"));
	m_listForce.InsertItem(7,_T("Torque5"));
	m_listForce.SetItemText(7,2,_T("Nm"));
	m_listForce.InsertItem(8,_T("Torque6"));
	m_listForce.SetItemText(8,2,_T("Nm"));

	//slide control

	m_slidFX.SetPos(1);
	m_slidFY.SetPos(1);
	m_slidFZ.SetPos(1);
	m_slidMotor1.SetPos(1);
	m_slidMotor2.SetPos(1);
	m_slidMotor3.SetPos(1);
	m_slidMotor4.SetPos(1);
	m_slidMotor5.SetPos(1);
	m_slidMotor6.SetPos(1);



	m_slidFX.SetRange(-MAX_FORCE*10,MAX_FORCE*10);			// interger -100 to 100. Converting to -10 to 10 step 0.1;
	m_slidFY.SetRange(-MAX_FORCE*10,MAX_FORCE*10);
	m_slidFZ.SetRange(-MAX_FORCE*10,MAX_FORCE*10);
	m_slidMotor1.SetRange(-MAX_TORQUE*1000,MAX_TORQUE*1000);
	m_slidMotor2.SetRange(-MAX_TORQUE*1000,MAX_TORQUE*1000);
	m_slidMotor3.SetRange(-MAX_TORQUE*1000,MAX_TORQUE*1000);
	m_slidMotor4.SetRange(-MAX_TORQUE*1000,MAX_TORQUE*1000);
	m_slidMotor5.SetRange(-MAX_TORQUE*1000,MAX_TORQUE*1000);
	m_slidMotor6.SetRange(-MAX_TORQUE*1000,MAX_TORQUE*1000);




	//UpdateData(false);
	UpdateWindow();
	

	return TRUE;  // 除非将焦点设置到控件，否则返回 TRUE
}

void CUSBDevControlDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialog::OnSysCommand(nID, lParam);
	}
}

// 如果向对话框添加最小化按钮，则需要下面的代码
//  来绘制该图标。对于使用文档/视图模型的 MFC 应用程序，
//  这将由框架自动完成。

void CUSBDevControlDlg::OnPaint()
{
	if (IsIconic())		// if the window is minimized
	{
		CPaintDC dc(this); // 用于绘制的设备上下文

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 使图标在工作区矩形中居中
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// 绘制图标
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialog::OnPaint();
	}
}

//当用户拖动最小化窗口时系统调用此函数取得光标
//显示。
HCURSOR CUSBDevControlDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}


void CUSBDevControlDlg::OnBnClickedCancel()
{
	// TODO: 在此添加控件通知处理程序代码
	m_usbDev.ReleaseDev();// close device;
	DestroyTimer();		// release the mm timer
	OnCancel();
}

void CUSBDevControlDlg::OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar)
{
	// TODO: 在此添加消息处理程序代码和/或调用默认值
	/*m_spinMotor1.SetPos(m_slidMotor1.GetPos());
	m_spinMotor2.SetPos(m_slidMotor2.GetPos());
	m_spinMotor3.SetPos(m_slidMotor3.GetPos());*/

	int i =0;
	CString strT;
	
	// update list control
	m_dF[0] = m_slidFX.GetPos()*F2DOUBLE;
	m_dF[1] = m_slidFY.GetPos()*F2DOUBLE;
	m_dF[2] = m_slidFZ.GetPos()*F2DOUBLE;

	m_dT[0]=  m_slidMotor1.GetPos()*T2DOUBLE;
	m_dT[1]=  m_slidMotor2.GetPos()*T2DOUBLE;
	m_dT[2]=  m_slidMotor3.GetPos()*T2DOUBLE;
	m_dT[3]=  m_slidMotor4.GetPos()*T2DOUBLE;
	m_dT[4]=  m_slidMotor5.GetPos()*T2DOUBLE;
	m_dT[5]=  m_slidMotor6.GetPos()*T2DOUBLE;

	// list control
	for (i=0;i<3;i++)
	{
		strT.Format(L"%3.1f",m_dF[i]);
		m_listForce.SetItemText(i,1,strT);
	}
	for (i=0;i<6;i++)
	{
		strT.Format(L"%5f",m_dT[i]);
		m_listForce.SetItemText(i+3,1,strT);
	}

	CDialog::OnHScroll(nSBCode, nPos, pScrollBar);
}

void CUSBDevControlDlg::OnBnClickedBtnConnectDev()
{
	// TODO: 在此添加控件通知处理程序代码
		UpdateData(true);
	CButton * pChkConnect = (CButton*)GetDlgItem(IDC_CHK_CONNECTED);
	bool blDevConnected = pChkConnect->GetCheck();
	if (blDevConnected)
	{
		m_usbDev.ReleaseDev();	// parameter default 0;
		SetDlgItemText(IDC_BTN_CONNECT_DEV,L"connect device");
		pChkConnect->SetCheck( BST_UNCHECKED);
	} 
	else
	{
		if(m_usbDev.ConnectDevice())	// default nInd 0
		{
			SetDlgItemText(IDC_BTN_CONNECT_DEV,L"disconnect device");
			pChkConnect->SetCheck( BST_CHECKED);
			
		}
	}	
}

void CUSBDevControlDlg::OnBnClickedBtnIoMsg()
{
	// TODO: 在此添加控件通知处理程序代码
	CString	strTemp= L"", strT;
	int		i = 0, nT= 0;
	int static nCount=0;
	double	m_dRtRad[3] = {0};
	// test value
//	BYTE	bTA,bTB;
//	double	dT=0;
//	bTA=4; bTB=5;
//	dT = (double)bTA/bTB;;
//	bTA = 5;
	// my new edition parameters
	int		nErrCode = 0, nPosT = 0;	
	if(m_usbDev.SetTorqueGetPosStatus(m_dT,nErrCode))
	{
	//	nCount++;
		m_usbDev.GetPos(m_dHX);
		m_usbDev.GetRad(m_dRad);
		m_usbDev.GetRotationRad(m_dRtRad);
		m_usbDev.GetState(m_statu);
		m_usbDev.GetTorque(m_dT);
	}
	UpdateData(TRUE);	
	//if (((CButton*)GetDlgItem(IDC_CHK_FOR))->GetCheck())
	if(!m_nForceCtrl)		// index 0 , for force control mode;
	{
		//if(m_usbDev.SetFTGetPosStatues(m_dF,m_dHX,m_statu,m_dT,blOutRange))
		if(m_usbDev.SetFTGetPosStatus(m_dF,m_dT+3,nErrCode));	// last 3 torque
		{
			nCount++;
			m_usbDev.GetPos(m_dHX);
			m_usbDev.GetRad(m_dRad);
			m_usbDev.GetState(m_statu);
			m_usbDev.GetTorque(m_dT);
		}
	}
	else
	{		
		//if(m_usbDev.SetMotorGetPosStatus(m_dT,nErrCode));	// last 3 torque
		// use the real torque here
		if(m_usbDev.SetTorqueGetPosStatus(m_dT,nErrCode))
		{
			nCount++;
			m_usbDev.GetPos(m_dHX);
			m_usbDev.GetRad(m_dRad);
			m_usbDev.GetState(m_statu);
			m_usbDev.GetTorque(m_dT);
		}
	//	if (((CButton*)GetDlgItem(IDC_CHK_MOTOR1))->GetCheck())
		//UpdateData(TRUE);
		//this->PostMessage(WM_UPDATE_IN_MESSAGE,0,0);
	//	if(!m_nMotor1Dir)	
	//	bPwm[0]= m_spinMotor1.GetPos();
	//	bPwm[1]= m_spinMotor2.GetPos();
	//	bPwm[2]= m_spinMotor3.GetPos();
		// no force now. It can be added here.
	}
	// UPDATE UI
	if (REFRESH_RATE<= nCount)
	{
		// ver 2.0 for 580
		// dev statu
		for (i=0;i<3;i++)
		{
			strT.Format(L"%5f",m_dHX[i]);
			m_listDev.SetItemText(i,1,strT);			
		}
		for (i=0;i<6;i++)
		{
			strT.Format(L"%5f",m_dRad[i]);
			m_listDev.SetItemText(i+3,1,strT);
		}
		strT.Format(L"%d",m_statu);
		m_listDev.SetItemText(9,1,strT);		
		// first 3 axis slide refresh

		nPosT = m_dT[0]/T2DOUBLE;		// first 3 axis slide
		m_slidMotor1.SetPos(nPosT);
		nPosT = m_dT[1]/T2DOUBLE;
		m_slidMotor2.SetPos(nPosT);
		nPosT = m_dT[2]/T2DOUBLE;
		m_slidMotor3.SetPos(nPosT);

		nCount =0;
	}	
}

void CUSBDevControlDlg::OnBnClickedBtnTest1()
{
	// TODO: 在此添加控件通知处理程序代码
	// test the double to int ,success
	/*short	tarV = 0,outV = 0;
	double	dT  =-0.021 ;
	UINT8		u8ATest[2]={0};
	tarV = dT*7500;
	u8ATest[0]= tarV&0xff;
	u8ATest[1]= (tarV>>8)&0xff;	
	outV = (u8ATest[1]<<8)| (u8ATest[0]);*/

	// test the thread parameter passing
	DWORD code;
	int i=0;
	CString strT;
	//GetExitCodeThread(m_pThread->m_hThread , &code);
	//if(code==STILL_ACTIVE)
	//{
	//	//线程仍在执行
	//	;	
	//}
//	else 
	{
		//线程停止执行
		g_clsCriticalSection.Lock();
		for (i=0;i<6;i++)
		{
			strT.Format(L"%5f",m_dTest[i]);
			m_listDev.SetItemText(i,1,strT);			
		}
		g_clsCriticalSection.Unlock();
	
	}


	// test the us counter*************************
	/*int i;
	LARGE_INTEGER m_liPerfFreq={0}; 
	//获取每秒多少CPU Performance Tick 
	QueryPerformanceFrequency(&m_liPerfFreq); 
	LARGE_INTEGER m_liPerfStart={0}; 
	QueryPerformanceCounter(&m_liPerfStart); 
	for(i=0; i< 100; i++) 
	//	cout << i << endl; 
		TRACE("%d\n",i);
	LARGE_INTEGER liPerfNow={0}; 
	// 计算CPU运行到现在的时间 
	QueryPerformanceCounter(&liPerfNow); 
	int time=( ((liPerfNow.QuadPart - m_liPerfStart.QuadPart) * 1000)/m_liPerfFreq.QuadPart); 
	char buffer[100]; 
	sprintf(buffer,"執行時間 %d millisecond\n",time); 
	// cout<<buffer<<endl; 
	TRACE("%s",buffer);
	i=0;*/
	/*BYTE	pwm[3]={0};
	BYTE	motorCtrl = 0;
	UINT16 Axis[3]={5};
	BYTE	statues = 2;

	m_usbDev.SetMotorGetPosStatues(pwm,motorCtrl,Axis,statues);*/

	// check the pos IO***********************
	//	UINT16 fdbPos[3]={5000,5000,5000};
	//	ChkPosIO(fdbPos);

	// check the continue device I/O in while circle to chekc the maxium fraquency. ************b
	/*
	BYTE	bPwm[3]={0};
	BYTE	ctrlMotor = 0;
	UINT16 AxisPos[3]={0};
	BYTE	statues = 0;
	CString	strTemp= L"";
	while(1)
	{		
		if (((CButton*)GetDlgItem(IDC_CHK_MOTOR1))->GetCheck())
			ctrlMotor |= BIT_MOTOR1_EN;	
		else
			ctrlMotor &= ~ BIT_MOTOR1_EN;

		if (((CButton*)GetDlgItem(IDC_CHK_MOTOR2))->GetCheck())
			ctrlMotor |= BIT_MOTOR2_EN;	
		else
			ctrlMotor &= ~ BIT_MOTOR2_EN;

		if (((CButton*)GetDlgItem(IDC_CHK_MOTOR3))->GetCheck())
			ctrlMotor |= BIT_MOTOR3_EN;	
		else
			ctrlMotor &= ~ BIT_MOTOR3_EN;

		bPwm[0]= m_spinMotor1.GetPos();
		bPwm[1]= m_spinMotor2.GetPos();
		bPwm[2]= m_spinMotor3.GetPos();
		if (m_usbDev.SetMotorGetPosStatues(bPwm,ctrlMotor,AxisPos,statues))
		{
			SetDlgItemInt(IDC_ED_ENC1,AxisPos[0]);
			SetDlgItemInt(IDC_ED_ENC2,AxisPos[1]);
			SetDlgItemInt(IDC_ED_ENC3,AxisPos[2]);
			SetDlgItemInt(IDC_ED_STATUES,statues);
		}	
		if (statues & BASEBTN)
			break;
	}*/
	// test result is 1 millisecond per circle *****************e
	// test for F output
	//double R[3][3]={1,2,3,4,5,6,7,8,9};
	// m_usbDev.SetRotation(R);		// setting succeed

//	TestIo();

	// test for the clockwiseChange	***********succeed
	//double array[3]={1.2,2.4,3.8};
	//m_usbDev.Clockwise321<double>(array);
	//m_usbDev.Clockwise321<double>(array);
}
//void CALLBACK CUSBDevControlDlg::SendFun(UINT wTimerID, UINT msg, DWORD dwUser, DWORD dwl, DWORD dw2)
//{
//	CUSBDevControlDlg* pdcpackerdlg = (CUSBDevControlDlg*)dwUser;	
//	//int i =1;
//	//i=i+2;
//	UINT16	chkPos[3];
//
//	if (((CButton*)pdcpackerdlg->GetDlgItem(IDC_CHK_POS_FDB))->GetCheck())
//	{			
//		chkPos[0]=pdcpackerdlg->GetDlgItemInt(IDC_ED_FDB_ENC1);
//		chkPos[1]=pdcpackerdlg->GetDlgItemInt(IDC_ED_FDB_ENC2);
//		chkPos[2]=pdcpackerdlg->GetDlgItemInt(IDC_ED_FDB_ENC3);
//		pdcpackerdlg->ChkPosIO(chkPos);
//	} 
//	else
//	{
//		pdcpackerdlg->OnBnClickedBtnIoMsg();
//	}	
//
//}

void CUSBDevControlDlg::MMTimerHandler(UINT nIDEvent) // called every elTime milliseconds
{
	// do what you want to do, but quickly
	int i=0;
	i=1;
	//OnBnClickedBtnIoMsg();
}

void CALLBACK TimerFunction(UINT wTimerID, UINT msg,
							DWORD dwUser, DWORD dw1, DWORD dw2)
{
	// This is used only to call MMTimerHandler

	// Typically, this function is static member of CTimersDlg

	CUSBDevControlDlg* obj = (CUSBDevControlDlg*) dwUser;
	obj->MMTimerHandler(wTimerID);
}

bool  CUSBDevControlDlg::CreateTimer()
{ 
	     TIMECAPS   tc;   
	     UINT wTimerRes; 
	 
	     //设置多媒体定时器  
		     if(timeGetDevCaps(&tc,sizeof(TIMECAPS))!=TIMERR_NOERROR)//向机器申请一个多媒体定时器       
		        return false;
	 
		     //获得机器允许的时间间隔（一般可达到1毫秒）   
		     wTimerRes=min(max(tc.wPeriodMin,1),tc.wPeriodMax);   // 1 or min < max
	 
		     //定时器开始工作   
		     timeBeginPeriod(wTimerRes);   
	 
		     //每过6毫秒调用回调函数timerback(),wTimerID为定时器ID.TIME_PERIODIC表周期性调用，TIME_ONESHOT表只产生一次事件   
		   //  g_wTimerID = timeSetEvent(6,  wTimerRes, (LPTIMECALLBACK)SendFun,  (DWORD)this, TIME_PERIODIC);  
			//  g_wTimerID = timeSetEvent(1,  wTimerRes, (LPTIMECALLBACK)SendFun,  (DWORD)this, TIME_PERIODIC); 
			
			 // 网上原型
			/* m_idEvent = timeSetEvent(
				 m_elTime,
				 resolution,
				 TimerFunction,
				 (DWORD)this,
				 TIME_PERIODIC);*/
			  g_wTimerID = timeSetEvent(
				  1,
				  wTimerRes,
				  TimerFunction,		// call back function
				  (DWORD)this,			// specified user data
				  TIME_PERIODIC);
		  if(g_wTimerID == 0)
		       return false;
	 
		  return true;
 }
void CUSBDevControlDlg::DestroyTimer()
 {
	     if (g_wTimerID)
		     {
				 timeKillEvent(g_wTimerID);
		         g_wTimerID = 0;
			 }
 }

void CUSBDevControlDlg::OnBnClickedBtnMmTimer()
{
	// TODO: 在此添加控件通知处理程序代码
	UpdateData(true);

	if (((CButton*)GetDlgItem(IDC_CHK_MM_TIMER))->GetCheck())
	{
		//KillTimer(TIMER_SEQ);
		//CreateTimer();
		DestroyTimer();
		SetDlgItemText(IDC_BTN_MM_TIMER,L"set mm timer");
		((CButton*)GetDlgItem(IDC_CHK_MM_TIMER))->SetCheck(false);		
	} 
	else
	{
		CreateTimer();
		// SetTimer(TIMER_SEQ,10,NULL);
		SetDlgItemText(IDC_BTN_MM_TIMER,L"stop seq commu");
		((CButton*)GetDlgItem(IDC_CHK_MM_TIMER))->SetCheck(TRUE);
	}
}
LRESULT CUSBDevControlDlg::OnUpdateInMessage(WPARAM wParam, LPARAM lParam)
{
	UpdateData(TRUE);
	return 0;
}
void CUSBDevControlDlg::OnBnClickedRadMotor1()
{
	// TODO: 在此添加控件通知处理程序代码
	UpdateData(TRUE);
}

void CUSBDevControlDlg::OnBnClickedRadMotor1b()
{
	// TODO: 在此添加控件通知处理程序代码
	UpdateData(TRUE);
}

void CUSBDevControlDlg::OnBnClickedRadMotor2()
{
	// TODO: 在此添加控件通知处理程序代码
		UpdateData(TRUE);
}

void CUSBDevControlDlg::OnBnClickedRadMotor2b()
{
	// TODO: 在此添加控件通知处理程序代码
		UpdateData(TRUE);
}

void CUSBDevControlDlg::OnBnClickedRadMotor3()
{
	// TODO: 在此添加控件通知处理程序代码
		UpdateData(TRUE);
}

void CUSBDevControlDlg::OnBnClickedRadMotor3b()
{
	// TODO: 在此添加控件通知处理程序代码
		UpdateData(TRUE);
}

void CUSBDevControlDlg::OnBnClickedChkMotor1()
{
	// TODO: 在此添加控件通知处理程序代码
			UpdateData(TRUE);
}

void CUSBDevControlDlg::OnBnClickedChkMotor2()
{
	// TODO: 在此添加控件通知处理程序代码
			UpdateData(TRUE);
}

void CUSBDevControlDlg::OnBnClickedChkMotor3()
{
	// TODO: 在此添加控件通知处理程序代码
			UpdateData(TRUE);
}

void CUSBDevControlDlg::OnBnClickedChkFor()
{
	// TODO: 在此添加控件通知处理程序代码
	if (((CButton*)GetDlgItem(IDC_CHK_FOR))->GetCheck())		// force on
	{
		//(CButton*)GetDlgItem(IDC_BTN_SETF)->EnableWindow(TRUE);
		//(CButton*)GetDlgItem(IDC_BTN_CLEAR_F)->EnableWindow(TRUE);

		GetDlgItem(IDC_RAD_F)->EnableWindow(TRUE);
		GetDlgItem(IDC_RAD_ACT)->EnableWindow(TRUE);
		((CButton*)GetDlgItem(IDC_RAD_F))->SetCheck(TRUE);
		((CButton*)GetDlgItem(IDC_RAD_ACT))->SetCheck(FALSE);

		// force control enable
		m_slidFX.EnableWindow(TRUE);
		m_slidFY.EnableWindow(TRUE);
		m_slidFZ.EnableWindow(TRUE);
		// torque control disable
		m_slidMotor1.EnableWindow(FALSE);
		m_slidMotor2.EnableWindow(FALSE);
		m_slidMotor3.EnableWindow(FALSE);

		m_slidMotor4.EnableWindow(TRUE);
		m_slidMotor5.EnableWindow(TRUE);
		m_slidMotor6.EnableWindow(TRUE);

	}
	else
	{
		// clear
		m_slidFX.SetPos(0);
		m_slidFY.SetPos(0);
		m_slidFZ.SetPos(0);
		m_slidMotor1.SetPos(0);
		m_slidMotor2.SetPos(0);
		m_slidMotor3.SetPos(0);
		m_slidMotor4.SetPos(0);
		m_slidMotor5.SetPos(0);
		m_slidMotor6.SetPos(0);

		memset(m_dF,0,sizeof(m_dF));	// clear the force variable
		memset(m_dT,0,sizeof(m_dT));

		// enable
		GetDlgItem(IDC_RAD_F)->EnableWindow(FALSE);
		GetDlgItem(IDC_RAD_ACT)->EnableWindow(FALSE);
		m_slidFX.EnableWindow(FALSE);
		m_slidFY.EnableWindow(FALSE);
		m_slidFZ.EnableWindow(FALSE);
		m_slidMotor1.EnableWindow(FALSE);
		m_slidMotor2.EnableWindow(FALSE);
		m_slidMotor3.EnableWindow(FALSE);
		m_slidMotor4.EnableWindow(FALSE);
		m_slidMotor5.EnableWindow(FALSE);
		m_slidMotor6.EnableWindow(FALSE);

		//Invalidate();
		RedrawWindow();
	}
}

void CUSBDevControlDlg::OnBnClickedBtnSetf()
{
	// TODO: 在此添加控件通知处理程序代码
	int nF; 
	//double dT;
	//nT = GetDlgItemInt(IDC_ED_FX);
	//dT = nT/1000.0;
	/*nF = GetDlgItemInt(IDC_ED_FX);
	m_dF[0] = nF/1000.0;
	nF = GetDlgItemInt(IDC_ED_FY);
	m_dF[1] = nF/1000.0;
	nF = GetDlgItemInt(IDC_ED_FZ);
	m_dF[2] = nF/1000.0;*/	
}

void CUSBDevControlDlg::OnBnClickedBtnClearF()
{
	// TODO: 在此添加控件通知处理程序代码
	//memset(m_dF,0,sizeof(m_dF));
	//memset(m_dT,0,sizeof(m_dT));			// clear force
	//SetDlgItemInt(IDC_ED_FX,0);
	//SetDlgItemInt(IDC_ED_FY,0);
	//SetDlgItemInt(IDC_ED_FZ,0);

}

void CUSBDevControlDlg::OnBnClickedRadF()
{
	// TODO: 在此添加控件通知处理程序代码
	m_slidFX.EnableWindow(TRUE);
	m_slidFY.EnableWindow(TRUE);
	m_slidFZ.EnableWindow(TRUE);
	m_slidMotor1.EnableWindow(FALSE);
	m_slidMotor2.EnableWindow(FALSE);
	m_slidMotor3.EnableWindow(FALSE);
}

void CUSBDevControlDlg::OnBnClickedRadAct()
{
	// TODO: 在此添加控件通知处理程序代码
	m_slidFX.EnableWindow(FALSE);
	m_slidFY.EnableWindow(FALSE);
	m_slidFZ.EnableWindow(FALSE);
	m_slidMotor1.EnableWindow(TRUE);
	m_slidMotor2.EnableWindow(TRUE);
	m_slidMotor3.EnableWindow(TRUE);
}

void CUSBDevControlDlg::OnBnClickedBtnTimer1()
{
	// TODO: 在此添加控件通知处理程序代码
	UpdateData(true);

	if (((CButton*)GetDlgItem(IDC_CHK_TIMER1))->GetCheck())
	{
		//KillTimer(TIMER_SEQ);
		//CreateTimer();
	//	DestroyTimer();
		KillTimer(TIMER1);
		SetDlgItemText(IDC_BTN_TIMER1,L"set timer1");
		((CButton*)GetDlgItem(IDC_CHK_TIMER1))->SetCheck(false);		
	} 
	else
	{
	//	CreateTimer();
		// SetTimer(TIMER_SEQ,10,NULL);
	
		SetTimer(TIMER1,100,NULL);
		SetDlgItemText(IDC_BTN_TIMER1,L"kill timer1");
		((CButton*)GetDlgItem(IDC_CHK_TIMER1))->SetCheck(TRUE);
	}
}


void CUSBDevControlDlg::OnTimer(UINT_PTR nIDEvent)
{
	// TODO: 在此添加消息处理程序代码和/或调用默认值
	OnBnClickedBtnIoMsg();
	CDialog::OnTimer(nIDEvent);
}




void CUSBDevControlDlg::GetServoTorque(double servoRad[6], double fdbRad[6],double torque[6])
{	
	double ref = 0,fdb=0,err=0;
	int i=0;
	// test first route
	/* 
	ref = servoRad[0];
	fdb = fdbRad[0];
	err = ref - fdb;
	m_up = err*m_kp;
	m_ui[0]= m_ui[0]+ err* m_ki;	// 0 should change to i for cycling

	if(m_ui[0] > m_outmax)
		m_ui[0] = m_outmax;
	if (m_ui[0]< -m_outmax)
	{
		m_ui[0] = -m_outmax;
	}
	m_outpre[0] = m_outnow[0];
	m_outnow[0] = m_up +m_ui[0];
	
	if(m_outnow[0]>m_outmax)
		m_outnow[0] = m_outmax;
	if (m_outnow[0]<-m_outmax)
	{
		m_outnow[0] = -m_outmax;
	}
	torque[0] = m_outnow[0];	
	*/
	// change to 6 circle
	for (i=0;i<6;i++)
	{
		ref = servoRad[i];
		fdb = fdbRad[i];
		err = ref - fdb;
		m_up = err*m_kp;
		m_ui[i]= m_ui[i]+ err* m_ki;	// 0 should change to i for cycling

		if(m_ui[i] > m_outmax)
			m_ui[i] = m_outmax;
		if (m_ui[i]< -m_outmax)
		{
			m_ui[i] = -m_outmax;
		}
		m_outpre[i] = m_outnow[i];
		m_outnow[i] = m_up +m_ui[i];

		if(m_outnow[i]>m_outmax)
			m_outnow[i] = m_outmax;
		if (m_outnow[i]<-m_outmax)
		{
			m_outnow[i] = -m_outmax;
		}
		torque[i] = m_outnow[i];	
	}

}
 /* 
 void PIA (void)							// Read the swith to change command
{
	//	ref = DesCur1;	
	if(index1<6)
	{
		ref = DesCur[index1];
		fdb = CurFdb1;
		//		test area
		//	fdb = -100;
		err = ref - fdb;
		up = err*kp[index1];
		uia[index1] = uia[index1] + err * ki[index1];
		if(uia[index1] >= outmax)
		{
			uia[index1] = outmax;
		}
		if(uia[index1] <= outmin)
		{
			uia[index1] = outmin;
		}
		//	outpre = up + uia;
		//	out = outpre;
		outpre[index1]= out[index1];
		out[index1] = up + uia[index1];
		if(out[index1] >= outmax)
		{
			out[index1] = outmax;
		}
		if(out[index1] <= outmin)
		{
			out[index1] = outmin;
		}
	}
}
*/

DWORD WINAPI Thread1Proc(  LPVOID lpParameter)  
{  
    CDeltaUSBControl usbDev;
	usbDev.ConnectDevice();
	int err;
	double m_dF[3];			 // 力
	double m_dT[6];			// 力矩
	while(TRUE)  
    {  
        if(usbDev.SetFTGetPosStatus(m_dF,m_dT+3,err));	// last 3 torque
		{
			
		}
    }  
  
    return 0;  
}  