#pragma once
#include "afxwin.h"

class CMyThread :
	public CWinThread
{
public:
	CMyThread(void);
	~CMyThread(void);
};
