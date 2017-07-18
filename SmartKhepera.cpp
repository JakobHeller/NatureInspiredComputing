#include "SmartKhepera.h"



CSmartKhepera::CSmartKhepera()
{
	m_pUtil = new CKheperaUtility();
	m_pControl = new CController(m_pUtil);
	m_pOperate = new COperator(m_pUtil);
	m_pValues = new CValueSystem(m_pUtil);
}

CSmartKhepera::~CSmartKhepera()
{
	StopLearning();
	StopRobot();
	
	delete m_pValues;
	delete m_pOperate;
	delete m_pControl;
	delete m_pUtil;
}


void CSmartKhepera::StartRobot()
{
	m_pControl->Start();
	m_pOperate->Start();
}

void CSmartKhepera::StopRobot()
{
    StopLearning();
	m_pOperate->Stop();
	m_pControl->Stop();
	m_pUtil->SetSpeed({ {0,0} });
}

void CSmartKhepera::StartLearning()
{
	m_pValues->Start();
}

void CSmartKhepera::StopLearning()
{
	m_pValues->Stop();
}

void CSmartKhepera::StartVerbosity()
{
	m_pUtil->SetVerbosity(true);
}

void CSmartKhepera::StopVerbosity()
{
	m_pUtil->SetVerbosity(false);
}
