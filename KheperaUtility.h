#ifndef __KheperaUtility_H__
#define __KheperaUtility_H__

#include <random>
#include <mutex>

#include "KheperaInterface.h"

// custom types
struct SSpeed
{
	double left;
	double right;
	SSpeed(double l = 0, double r = 0) { left = l; right = r; }
};

struct SIOSet
{
	Int8 sensors;
	SSpeed speed;
	SIOSet(Int8 sens = Int8(), SSpeed spd = SSpeed()) { sensors = sens; speed = spd; }
	SIOSet(const SIOSet &other) 
	{
		for (int i = 0; i < 8; i++)
		{
			this->sensors.data[i] = other.sensors.data[i];
		}
		this->speed.left = other.speed.left;
		this->speed.right = other.speed.right;
	}
};

typedef std::lock_guard<std::mutex> ScopedMutexLocker;

// value defines
#define CLOSE_SENSOR_VAL 1024
#define FAR_SENSOR_VAL 0
#define SENSOR_VAL_RANGE abs(FAR_SENSOR_VAL - CLOSE_SENSOR_VAL)
#define MAX_SPEED 20

class CKheperaUtility
{
public:
	CKheperaUtility();
	~CKheperaUtility();

	// for khepera interaction
	Int8 GetSensorData();
	void SetSpeed(Int2 newSpeed);

	// for controller output
	void SetNetworkResult(SIOSet results);
	SIOSet GetLastNetworkResult();

	// for value system output 
	void SetCorrectedResult(SIOSet results);
	SIOSet GetLastCorrectedResult();

	// randomness
	double GetUniformRandom(double min = 0, double max = 1);

	// info dumping
	void SetVerbosity(bool bVerbose);

private:
	KheperaInterface* m_pKhep;
	std::default_random_engine m_rGenerator;
	SIOSet m_LastResult;
	SIOSet m_LastCorrectedResult;

	std::mutex m_KheperaMutex;
	std::mutex m_ResultMutex;
	std::mutex m_CorrectedResultMutex;

	bool m_bVerbose;
};

#endif