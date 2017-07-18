#include "ValueSystem.h"

#define PI 3.14159265

CValueSystem::CValueSystem(CKheperaUtility * pUtil) : CThreadableBase(pUtil)
{
}

void CValueSystem::DoCycle()
{
	// evaluate current results
	SIOSet controllerResults = m_pUtil->GetLastNetworkResult();
	SIOSet correction = Correct(controllerResults);
	m_pUtil->SetCorrectedResult(correction);
}

SIOSet CValueSystem::Correct(SIOSet calculated)
{
	// convert speed
	SDirectionalSpeed directional = ToDirectional(calculated.speed);

	int limit = 200; //SafetyDistance(directional.speed);

	// evaluate sensor data in movement direction
	int proximity;
	if (abs(directional.angle) < PI / 2)
	{	// front half
		double subIndex = fmax(0, ((PI / 2 - (directional.angle)) / (PI / 5)) - 1);
		proximity = fmax(calculated.sensors.data[(int)floor(subIndex)], calculated.sensors.data[(int)ceil(subIndex)]);
	}
	else if (abs(directional.angle) > (5 * PI / 6))
	{	// mid back
		proximity = fmax(calculated.sensors.data[6], calculated.sensors.data[7]);
	}
	else if (directional.angle > 0)
	{	// left back
		proximity = fmax(calculated.sensors.data[0], calculated.sensors.data[7]);
	}
	else
	{	// right back
		proximity = fmax(calculated.sensors.data[5], calculated.sensors.data[6]);
	}

	if (proximity > limit)
	{	// don't go there, you'll collide!
		// go anywhere else, just not there. Preferably forward-ish.
		directional.angle += m_pUtil->GetUniformRandom(-PI/4, PI/4);
	}
	
    if (directional.speed < MAX_SPEED/2)
	{	// you're fine, go faster
		directional.speed = abs(directional.speed) + m_pUtil->GetUniformRandom(0.5, 1);
		directional.speed *= 1.2;
	}
    
	SIOSet correction = calculated;
	correction.speed = ToComponents(directional);

	return correction;
}

int CValueSystem::SafetyDistance(double speed)
{
    int dist = 600 * (1 - (speed / (double)MAX_SPEED));
	//int dist = fmin(900, CLOSE_SENSOR_VAL*exp(- speed / (double)MAX_SPEED));
	return dist;
}

CValueSystem::SDirectionalSpeed CValueSystem::ToDirectional(SSpeed components)
{
	SDirectionalSpeed directional;
	double straight = (components.left + components.right) / 2; // cos(a)*v
	double turn = components.right - straight; // sin(a)*v
	
	directional.angle = atan2(turn, straight);
	directional.speed = straight / cos(directional.angle);

	return directional;
}

SSpeed CValueSystem::ToComponents(SDirectionalSpeed direction)
{
	SSpeed components;

	components.left = direction.speed * (cos(direction.angle) - sin(direction.angle));
	components.right = direction.speed * (cos(direction.angle) + sin(direction.angle));

	return components;
}
