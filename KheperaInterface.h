#ifndef __KHEPERAINTERFACE_H
#define __KHEPERAINTERFACE_H

#define SIM_ONLY

#ifndef SIM_ONLY 
	#include <termios.h>
#endif // !SIM_ONLY


#include <string>
using std::string;

typedef enum ledstate
{
	Off = 0,
	On,
	Toggle
} LEDState;

typedef struct { int data[2];  } Int2;
typedef struct { int data[6];  } Int6;
typedef struct { int data[8];  } Int8;
typedef struct { int data[16]; } Int16;
typedef struct { int data[32]; } Int32; 
typedef struct { int data[64]; } Int64;

class KheperaInterface
{
private:
	FILE* f;	
		#ifndef SIM_ONLY 
			struct termios oldtio;	
		#endif // !SIM_ONLY

	char buffer[10000];
	bool simulate, hasgripper, hascamera;
	
	void setupSerial(const char* dev);
	void sendCommand(const char* command, char* answer, int maxAnswerLength);
	bool parseIntList(char* text, int expectedCount, int result[]);
	
public:
		#ifndef SIM_ONLY 
			KheperaInterface(const string& device = "/dev/ttyUSB0", bool simulate = false);
		#else
			KheperaInterface(const string& device = "/dev/ttyUSB0", bool simulate = true);
		#endif // !SIM_ONLY

	~KheperaInterface();

	bool hasGripper();
	bool hasCamera();

	
	// Main KheperaII control methods
	
	void setSpeedControllerPID(int proportional, int integral, int differential); //A
	string getVersion(); //B
	void setPosition(int leftPos, int rightPos); //C
	void setSpeed(int leftMotor, int rightMotor); //D
	Int2 getSpeed(); //E
	void setPositionControllerPID(int proportional, int integral, int differential); //F
	void setPositionCounter(int leftCounter, int rightCounter); //G
	Int2 getPosition(); //H
	int getADInput(int channel); //I
	void setSpeedControllerProfile(int maxLeftSpeed, int leftAccel, int maxRightSpeed, int rightAccel); //J
	Int6 getMotionControllerStatus(); //K
	void setLEDState(int number, LEDState state); //L
	//M - unused
	Int8 getProximitySensors(); //N
	Int8 getAmbientSensors(); //O
	void setMotorPWM(int leftPWM, int rightPWM); //P
	//Q - unused
	int readExtensionBusByte(int address); //R
	//S - unused
	//T - turret subprotocol (see below, Gripper and Camera commands)
	//U,V - unused
	void writeExtensionBusByte(int address, int value); //W
	//X,Y,Z - unused


	// Gripper turrent control methods
	
	string getGripperVersion(); //T,1,B
	//T,1,C - unused  
	void setGripperState(bool opened); //T,1,D
	void setGripperPosition(int position); //T,1,E
	int getGripperImpedance(); //T,1,F
	int getGripperPhotoSensor(); //T,1,G
	bool getGripperState(); //T,1,H (additional parameter is 0)
	int getGripperPosition(); //T,1,H (additional parameter is 1)
	//T,1,I - unused
	int getGripperJumpers(); //T,1,J
	// ... T,1,Z - unused


	// Camera turret control methods
	
	string getCameraVersion(); //T,2,B
	//T,2,C ... T,2,K - unused
	int getCameraLightIntensity(); //T,2,L
	Int32 getCameraImageLowRes(); //T,2,M
	Int64 getCameraImage(); //T,2,N
	int getCameraMaxIntensityPixelIndex(); //T,2,O
	int getCameraMinIntensityPixelIndex(); //T,2,P
	Int8 getCameraEightPixels(int startPixelIndex); //T,2,Q
	Int16 getCameraSixteenPixels(int startPixelIndex); //T,2,R
	Int32 getCameraTwoSubScannedImage(); //T,2,S
	Int16 getCameraFourSubScannedImage(); //T,2,T
	void setCameraReadingPeriod(int period); //T,2,U
	// ... T,2,Z - unused
};

#endif

