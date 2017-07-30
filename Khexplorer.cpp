// Khexplorer.cpp : Defines the entry point for the console application.
//

#include <iostream>

#include "SmartKhepera.h"

void ListCommands();

static const std::string s_StartRunningCmd = "go";
static const std::string s_StopRunningCmd = "stop";
static const std::string s_StartVSCmd = "smart";
static const std::string s_StopVSCmd = "stupid";
static const std::string s_StartInfoCmd = "babble";
static const std::string s_StopInfoCmd = "quiet";
static const std::string s_Help = "help";
static const std::string s_EndProgram = "exit";

static const std::string s_StopOperator = "hobble";
// TODO: reset nodes
static const std::string s_SaveRBFNodesCmd = "save";
static const std::string s_LoadRBFNodesCmd = "load";

static const std::string path = "/home/jakob/RBF.txt";

int main()
{
	CSmartKhepera khexplore;

	// print welcome and help
	std::cout << "Welcome to the Khexplorer. ";
	ListCommands();

	// run and listen for instructions until exit
	std::string command;
	do
	{
		std::cout << "What would you like to do? ";
		std::cin >> command;

		if (command == s_StartRunningCmd) khexplore.StartRobot();
		if (command == s_StopRunningCmd) khexplore.StopRobot();
		if (command == s_StartVSCmd) khexplore.StartLearning();
		if (command == s_StopVSCmd) khexplore.StopLearning();
		if (command == s_StartInfoCmd) khexplore.StartVerbosity();
		if (command == s_StopInfoCmd) khexplore.StopVerbosity();
		if (command == s_StopOperator) khexplore.StopMoving();
        if (command == s_SaveRBFNodesCmd) khexplore.SaveNodes(path);
        if (command == s_LoadRBFNodesCmd) khexplore.LoadNodes(path);
		if (command == s_Help) ListCommands();
	} while (command != s_EndProgram);

	// stop our robot!
	khexplore.StopLearning();
	khexplore.StopRobot();

	std::cout << "Goodbye!" << std::endl;
    return 0;
}

void ListCommands()
{
	std::cout << "To issue a command, please type into the console and press \'enter\'." << std::endl;
	std::cout << "Possible commands: " << std::endl;
	std::cout << "   " << s_Help << "         : lists this help" << std::endl;
	std::cout << "   " << s_EndProgram << "         : ends the program" << std::endl;
	std::cout << "   " << s_StartRunningCmd << "           : starts the robot" << std::endl;
	std::cout << "   " << s_StopRunningCmd << "         : stops the robot" << std::endl;
	std::cout << "   " << s_StartVSCmd << "        : enables robot learning" << std::endl;
	std::cout << "   " << s_StopVSCmd << "       : disables robot learning" << std::endl;
	std::cout << "   " << s_StartInfoCmd << "       : enables info dumping" << std::endl;
	std::cout << "   " << s_StopInfoCmd << "        : disables info dumping" << std::endl;
    std::cout << "   " << s_SaveRBFNodesCmd << "         : save the RBF nodes to file" << std::endl;
    std::cout << "   " << s_LoadRBFNodesCmd << "         : Load the RBF nodes from file" << std::endl;
    
	std::cout << "   " << s_StopOperator << "       : disables setting speeds" << std::endl;
    
	std::cout << std::endl;
}