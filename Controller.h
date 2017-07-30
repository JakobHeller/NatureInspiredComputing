#ifndef __Controller_H__
#define __Controller_H__

#include "ThreadableBase.h"
#include <fstream>
#include <iomanip>
using namespace std;

typedef struct { double data[2];  } Double2;
struct SNode { Int8 center; double lWeight; double rWeight; double activity; };

#define INPUT_COUNT 8
#define NODE_COUNT 100
#define TRAINING_CYCLES 100
#define NODE_ACTIVITY_DECAY_FACTOR 0.9
#define TOTAL_ACTIVATION_LIMIT 0.5
#define MAX_DIMENSION_DISTANCE 25

class CController : public CThreadableBase
{
public:
	CController(CKheperaUtility* pUtil);

    void LoadNodesFromFile(std::string path);
    void SaveNodesToFile(std::string path);
    void ListNodes();
    
    std::fstream file;
    std::string line;
    Int8 temp_read_int;
    Double2 temp_read_double;
    
protected:
	virtual void DoCycle();

private:
	SIOSet Evaluate(Int8 sensors);
	void Adapt(SIOSet ideal);

	// rbf network functions
    double NodeMaxDimensionalDistance(SNode nodeA, SNode nodeB);
	double RbfBase(Int8 sensors, Int8 nodeCenter);
	void CreateTrainingData();
	void Train();
    void AddNode(Int8, double left, double right);
	void Forget();

private:
	std::vector<SNode> m_NetworkNodes;
	double m_Sigma;
	double m_LearnWeight;
	std::vector<SIOSet> m_TrainingData;
};

#endif