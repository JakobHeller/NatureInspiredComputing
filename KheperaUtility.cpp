#include <iostream>
#include <iomanip>

#include "KheperaUtility.h"

CKheperaUtility::CKheperaUtility()
{
	m_pKhep = new KheperaInterface("/dev/ttyUSB0");
	m_pKhep->setLEDState(0, LEDState::On);

	m_LastResult = SIOSet(Int8(), SSpeed(MAX_SPEED, MAX_SPEED));
	m_LastCorrectedResult = SIOSet(Int8(), SSpeed(MAX_SPEED, MAX_SPEED));

	m_bVerbose = false;
    
    m_rGenerator = std::default_random_engine(std::random_device{}());
}

CKheperaUtility::~CKheperaUtility()
{
	m_pKhep->setLEDState(0, LEDState::Off);
	m_pKhep->setSpeed(0, 0);
	delete m_pKhep;
}

Int8 CKheperaUtility::GetSensorData()
{
	ScopedMutexLocker lock(m_KheperaMutex);
	Int8 data;

	try
	{
		data = m_pKhep->getProximitySensors();
	}
	catch (...)
	{
		printf("Error reading sensor data!");
		return GetLastNetworkResult().sensors;
	}

	return data;
}

void CKheperaUtility::SetSpeed(Int2 newSpeed)
{
	ScopedMutexLocker lock(m_KheperaMutex);
	try
	{
		m_pKhep->setSpeed(newSpeed.data[0], newSpeed.data[1]);
	}
	catch (...)
	{
		printf("Error setting speed!");
	}	
}

void CKheperaUtility::SetNetworkResult(SIOSet results)
{
	void* t = this;
	ScopedMutexLocker lock(m_ResultMutex);
	m_LastResult = results;

	// output info
	if (m_bVerbose)
	{
        std::cout << "Controller's results:" << std::endl;
                
        for(int i = 0; i < 8; i++)
        {
            std::cout << " " << std::setfill(' ') << std::setw(4) << results.sensors.data[i];
        }
        
        std::cout << " ==> L: " << results.speed.left << " R: " << results.speed.right;
        std::cout << std::endl;
	}
}

SIOSet CKheperaUtility::GetLastNetworkResult()
{
	void* t = this;
	ScopedMutexLocker lock(m_ResultMutex);
	return m_LastResult;
}

void CKheperaUtility::SetCorrectedResult(SIOSet results)
{
	ScopedMutexLocker lock(m_CorrectedResultMutex);
	m_LastCorrectedResult = results;


	// output info
	if (m_bVerbose)
	{
        std::cout << "ValueSystem's results:" << std::endl;
                
        for(int i = 0; i < 8; i++)
        {
            std::cout << " " << std::setfill(' ') << std::setw(4) << results.sensors.data[i];
        }
        
        std::cout << " ==> L: " << results.speed.left << " R: " << results.speed.right;
        std::cout << std::endl;
	}
}

SIOSet CKheperaUtility::GetLastCorrectedResult()
{
	ScopedMutexLocker lock(m_CorrectedResultMutex);
	return m_LastCorrectedResult;
}

double CKheperaUtility::GetUniformRandom(double min, double max)
{
	std::uniform_real_distribution<double> rnd(min, max);
	return rnd(m_rGenerator);
}

void CKheperaUtility::SetVerbosity(bool bVerbose)
{
	m_bVerbose = bVerbose;
}
