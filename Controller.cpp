#include <math.h>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <iomanip>
using namespace std;
#include "Controller.h"
#include <sstream>
#include <vector>

#define DISTANCE_LIMIT (0.5 * CLOSE_SENSOR_VAL)

bool NodeActivitySort(SNode a, SNode b);

CController::CController(CKheperaUtility * pUtil) : CThreadableBase(pUtil)
{
	m_Sigma = 0.1;
	m_LearnWeight = 0.3;

	/*
	// generate nodes for network
	for (int n = 0; n < 2*NODE_COUNT; n++)
	{
		Int8 center;
		for (int i = 0; i < INPUT_COUNT; i++)
		{
			center.data[i] = round(m_pUtil->GetUniformRandom(FAR_SENSOR_VAL, CLOSE_SENSOR_VAL));
		}

		SNode node;
		node.center = center;
		node.lWeight = m_pUtil->GetUniformRandom(-MAX_SPEED, MAX_SPEED);
		node.rWeight = m_pUtil->GetUniformRandom(-MAX_SPEED, MAX_SPEED);
		node.activity = 1;
		m_NetworkNodes.push_back(node);
	}
	*/
	// pre-train
	CreateTrainingData();
	Train();
}

void CController::LoadNodesFromFile(std::string path)
{
    //clean all other nodes! 
    m_NetworkNodes.resize(0); //TODO This is arguably not the best option
    
    file.open (path);
    if (file.is_open())
        {
            while ( getline( file, line ) ) 
                {   
                    std::stringstream ss(line);
                    for ( int iter=0 ; iter<8; iter++ ) 
                        {
                            ss >> temp_read_int.data[iter];
                            
                        } 
                    for ( int iter=0 ; iter<2; iter++ ) 
                        {
                            ss >> temp_read_double.data[iter];
                        }
                    AddNode(temp_read_int, temp_read_double.data[0] , temp_read_double.data[1]);
                }
                
        }
    else cout << "Unable to open file";
            
    file.close();

            
}

void CController::SaveNodesToFile(std::string path)
{
    //opens the file and writes line by line a new safe overwrites the old data
    file.open (path);
    for (auto it = m_NetworkNodes.begin(); it != m_NetworkNodes.end(); it++)
    {
 
       if (file.is_open())
            {
                for(int i = 0; i < INPUT_COUNT; i++)
                    {
                    file << " " << it->center.data[i];
                    }
                    file << " " << it->lWeight << " " << it->rWeight << std::endl;
            }
        else cout << "Unable to open file";
            
    }file.close();
    
    
   
        
}

void CController::DoCycle()
{
	// evaluate current sensor data
	Int8 sensorData = m_pUtil->GetSensorData();

#ifdef SIM_ONLY
	sensorData = m_TrainingData[round(m_pUtil->GetUniformRandom(0, m_TrainingData.size()-1))].sensors;
#endif

	SIOSet result = Evaluate(sensorData);
	m_pUtil->SetNetworkResult(result);

	// get value system's correction
	SIOSet ideal = m_pUtil->GetLastCorrectedResult();
	Adapt(ideal);

	// check for surplus of nodes
	if (m_NetworkNodes.size() > NODE_COUNT)
	{
		Forget();
	}
}

void CController::Adapt(SIOSet ideal)
{
	SIOSet current = Evaluate(ideal.sensors);

	for (int n = 0; n < m_NetworkNodes.size(); n++)
	{
		double act = RbfBase(ideal.sensors, m_NetworkNodes[n].center);
		m_NetworkNodes[n].lWeight += (ideal.speed.left - current.speed.left) * act * m_LearnWeight;
		m_NetworkNodes[n].rWeight += (ideal.speed.right - current.speed.right) * act * m_LearnWeight;
	}
}

double CController::NodeMaxDimensionalDistance(SNode nodeA, SNode nodeB)
{
    double dist = 0;
    for(int i = 0; i < INPUT_COUNT; i++)
    {
        dist = fmax(dist, abs(nodeA.center.data[i] - nodeB.center.data[i]));
    }
    return dist;
}

double CController::RbfBase(Int8 sensors, Int8 nodeCenter)
{
	double sqdist = 0;
	for (int i = 0; i < INPUT_COUNT; i++)
	{
		double diff = (sensors.data[i] - nodeCenter.data[i]) / (double)SENSOR_VAL_RANGE;
		sqdist += pow(diff, 2);
	}
	return exp(-sqrt(sqdist) / m_Sigma);
}

void CController::CreateTrainingData()
{
	m_TrainingData.clear();

	Int8 sensors;

	// free field
	sensors.data[0] = FAR_SENSOR_VAL;
	sensors.data[1] = FAR_SENSOR_VAL;
	sensors.data[2] = FAR_SENSOR_VAL;
	sensors.data[3] = FAR_SENSOR_VAL;
	sensors.data[4] = FAR_SENSOR_VAL;
	sensors.data[5] = FAR_SENSOR_VAL;
	sensors.data[6] = FAR_SENSOR_VAL;
	sensors.data[7] = FAR_SENSOR_VAL;
	m_TrainingData.push_back(SIOSet(sensors, SSpeed(MAX_SPEED, MAX_SPEED)));

	// corridor
	sensors.data[0] = DISTANCE_LIMIT;
	sensors.data[1] = 0.5*DISTANCE_LIMIT;
	sensors.data[2] = FAR_SENSOR_VAL;
	sensors.data[3] = FAR_SENSOR_VAL;
	sensors.data[4] = 0.5*DISTANCE_LIMIT;
	sensors.data[5] = DISTANCE_LIMIT;
	sensors.data[6] = FAR_SENSOR_VAL;
	sensors.data[7] = FAR_SENSOR_VAL;
	m_TrainingData.push_back(SIOSet(sensors, SSpeed(MAX_SPEED, MAX_SPEED)));

	// back to dead end
	sensors.data[0] = DISTANCE_LIMIT;
	sensors.data[1] = FAR_SENSOR_VAL;
	sensors.data[2] = FAR_SENSOR_VAL;
	sensors.data[3] = FAR_SENSOR_VAL;
	sensors.data[4] = FAR_SENSOR_VAL;
	sensors.data[5] = DISTANCE_LIMIT;
	sensors.data[6] = DISTANCE_LIMIT;
	sensors.data[7] = DISTANCE_LIMIT;
	m_TrainingData.push_back(SIOSet(sensors, SSpeed(MAX_SPEED, MAX_SPEED)));


	// soft left corner
	sensors.data[0] = DISTANCE_LIMIT;
	sensors.data[1] = DISTANCE_LIMIT;
	sensors.data[2] = FAR_SENSOR_VAL;
	sensors.data[3] = FAR_SENSOR_VAL;
	sensors.data[4] = FAR_SENSOR_VAL;
	sensors.data[5] = FAR_SENSOR_VAL;
	sensors.data[6] = FAR_SENSOR_VAL;
	sensors.data[7] = FAR_SENSOR_VAL;
	m_TrainingData.push_back(SIOSet(sensors, SSpeed(MAX_SPEED, 0.5*MAX_SPEED)));

	// left corner
	sensors.data[0] = DISTANCE_LIMIT;
	sensors.data[1] = DISTANCE_LIMIT;
	sensors.data[2] = DISTANCE_LIMIT;
	sensors.data[3] = FAR_SENSOR_VAL;
	sensors.data[4] = FAR_SENSOR_VAL;
	sensors.data[5] = FAR_SENSOR_VAL;
	sensors.data[6] = FAR_SENSOR_VAL;
	sensors.data[7] = FAR_SENSOR_VAL;
	m_TrainingData.push_back(SIOSet(sensors, SSpeed(MAX_SPEED, -MAX_SPEED)));

	// frontal left
	sensors.data[0] = FAR_SENSOR_VAL;
	sensors.data[1] = FAR_SENSOR_VAL;
	sensors.data[2] = DISTANCE_LIMIT;
	sensors.data[3] = DISTANCE_LIMIT;
	sensors.data[4] = FAR_SENSOR_VAL;
	sensors.data[5] = FAR_SENSOR_VAL;
	sensors.data[6] = FAR_SENSOR_VAL;
	sensors.data[7] = FAR_SENSOR_VAL;
	m_TrainingData.push_back(SIOSet(sensors, SSpeed(-0.5*MAX_SPEED, -MAX_SPEED)));

	// frontal right
	sensors.data[0] = FAR_SENSOR_VAL;
	sensors.data[1] = FAR_SENSOR_VAL;
	sensors.data[2] = FAR_SENSOR_VAL;
	sensors.data[3] = DISTANCE_LIMIT;
	sensors.data[4] = DISTANCE_LIMIT;
	sensors.data[5] = FAR_SENSOR_VAL;
	sensors.data[6] = FAR_SENSOR_VAL;
	sensors.data[7] = FAR_SENSOR_VAL;
	m_TrainingData.push_back(SIOSet(sensors, SSpeed(-MAX_SPEED, -0.5*MAX_SPEED)));

	// soft right corner
	sensors.data[0] = FAR_SENSOR_VAL;
	sensors.data[1] = FAR_SENSOR_VAL;
	sensors.data[2] = FAR_SENSOR_VAL;
	sensors.data[3] = FAR_SENSOR_VAL;
	sensors.data[4] = DISTANCE_LIMIT;
	sensors.data[5] = DISTANCE_LIMIT;
	sensors.data[6] = FAR_SENSOR_VAL;
	sensors.data[7] = FAR_SENSOR_VAL;
	m_TrainingData.push_back(SIOSet(sensors, SSpeed(0.5*MAX_SPEED, MAX_SPEED)));

	// right corner
	sensors.data[0] = FAR_SENSOR_VAL;
	sensors.data[1] = FAR_SENSOR_VAL;
	sensors.data[2] = FAR_SENSOR_VAL;
	sensors.data[3] = DISTANCE_LIMIT;
	sensors.data[4] = DISTANCE_LIMIT;
	sensors.data[5] = DISTANCE_LIMIT;
	sensors.data[6] = FAR_SENSOR_VAL;
	sensors.data[7] = FAR_SENSOR_VAL;
	m_TrainingData.push_back(SIOSet(sensors, SSpeed(-MAX_SPEED, MAX_SPEED)));
}

void CController::Train()
{
	for (int i = 0; i < TRAINING_CYCLES * m_TrainingData.size(); i++)
	{
		Adapt(m_TrainingData[i%m_TrainingData.size()]);
	}
}

void CController::AddNode(Int8 sensors, double left, double right)
{
    SNode node;
    node.center = sensors;
    node.lWeight = left;
    node.rWeight = right;
    node.activity = 1;
    
    int dist = 0;
    for (int n = 0; n < m_NetworkNodes.size(); n++)
	{
        dist = fmax(dist, NodeMaxDimensionalDistance(node, m_NetworkNodes[n]));
	}
    
    if(dist > MAX_DIMENSION_DISTANCE || m_NetworkNodes.size() == 0)
    {
        m_NetworkNodes.push_back(node);
        std::cout << "Added new node at";
        for(int i = 0; i < INPUT_COUNT; i++)
        {
            std::cout << " " << std::setfill(' ') << std::setw(4) << node.center.data[i];
        }
        std::cout << std::endl;
    }
}

void CController::Forget()
{
    printf("Too much knowledge! Must forget!");
	std::sort(m_NetworkNodes.begin(), m_NetworkNodes.end(), NodeActivitySort);
    
    int count = 0;
    for (auto it = m_NetworkNodes.begin(); it != m_NetworkNodes.end(); it++)
    {
        count++;
        if(count > NODE_COUNT)
        {
            printf("Forgetting node at %d %d %d %d %d %d \n",
            it->center.data[0], 
            it->center.data[1], 
            it->center.data[2], 
            it->center.data[3], 
            it->center.data[4], 
            it->center.data[5]);
        }
    }
    
	m_NetworkNodes.resize(NODE_COUNT);
}

SIOSet CController::Evaluate(Int8 sensors)
{
	SIOSet out;
	out.sensors = sensors;
	double totalActivation = 0;

	for (int n = 0; n < m_NetworkNodes.size(); n++)
	{
		SNode node = m_NetworkNodes[n];
		double act = RbfBase(sensors, node.center);
		m_NetworkNodes[n].activity *= NODE_ACTIVITY_DECAY_FACTOR;
		m_NetworkNodes[n].activity += act;
		out.speed.left += act * node.lWeight;
		out.speed.right += act * node.rWeight;
		totalActivation += act;
	}

	if (totalActivation > 0)
	{
		out.speed.left /= totalActivation;
		out.speed.right /= totalActivation;
	}

	// add extra nodes if activation is too low
	if (totalActivation < TOTAL_ACTIVATION_LIMIT)
	{
        AddNode(sensors,
            m_pUtil->GetUniformRandom(-MAX_SPEED, MAX_SPEED), 
            m_pUtil->GetUniformRandom(-MAX_SPEED, MAX_SPEED));
	}

	return out;
}

void CController::ListNodes()
{
    std::cout << "These are my nodes:" << std::endl;
    for (auto it = m_NetworkNodes.begin(); it != m_NetworkNodes.end(); it++)
    {
        std::cout << "Node at";
        for(int i = 0; i < INPUT_COUNT; i++)
        {
            std::cout << " " << std::setfill(' ') << std::setw(4) << it->center.data[i];
        }
        std::cout << " => L: " << it->lWeight << " R: " << it->rWeight << std::endl;
    }
}

// helper
bool NodeActivitySort(SNode a, SNode b)
{
	return a.activity > b.activity;
}