#Example of the Base.cpp

###Base.cpp

``` cpp
#include <iostream>

#include <thread>
#include <chrono>
#include "ros/ros.h"
#include "Base.h"
#include "engine/AlicaClock.h"

//Choose one
#include "communication/AlicaRosCommunication.h"
#include "communication/AlicaDummyCommunication.h"

#include "robotmovement/RobotMovement.h"

using namespace std;
using namespace msl;

namespace msl
{

	Base::Base(string roleSetName, string masterPlanName, string roleSetDir)
	{
		ae = new alica::AlicaEngine();

		//Will be create from the PlanDesigner is domainspecific stuff for your robot
		//Creators which need the engine for example to call behaviours
		bc = new alica::BehaviourCreator();
		cc = new alica::ConditionCreator();
		uc = new alica::UtilityFunctionCreator();
		crc = new alica::ConstraintCreator();
		
		//ROS Communicator
		//For ROS use this
		ae->setCommunicator(new alicaRosProxy::AlicaRosCommunication(ae));

		//DummyCommunication has only empty methods so cant communicate with other robots
		//Without ros use this
		ae->setCommunicator(new alicaRosProxy::AlicaDummyCommunication(ae));
		
		//WORLDMODEL 
		wm = MSLWorldModel::get();

		//FOR CARPE NOCTEM
		RobotMovement::readConfigParameters();
		
		//The engine method will initialize all parameters and classes which it needs
		//Only call init isnt enough this will only initialize stuff and parse plans, beh etc., you have to call start ae->start();  
		ae->init(bc, cc, uc, crc, roleSetName, masterPlanName, roleSetDir, false);
	}

	void Base::start()
	{
		//After init the engine you should call start
		//The PlanBase will now tick 
		ae->start();
	}

	Base::~Base()
	{
		//Shutdown the engine will delete all pointer in the engine terminate it
		ae->shutdown();
		delete ae->getCommunicator();
		delete ae;
		delete cc;
		delete bc;
		delete uc;
		delete crc;
	}

} /* namespace msl */

// Arguments for the main should be the path MasterPlan name, RoleSet directory and the correct RoleSet. 
int main(int argc, char** argv)
{
	cout << "Initing Ros" << endl;

	//Dont need if you are not using ROS
	ros::init(argc, argv, "AlicaEngine");

	cout << "Starting Base" << endl;
	if (argc < 2)
	{
		cout << "Usage: Base -m [Masterplan] -rd [rolesetdir]" << endl;
		return 0;
	}

	string masterplan = "";
	string rolesetdir = "";
	string roleset = "";

	for (int i = 1; i < argc; i++)
	{
		if (string(argv[i]) == "-m" || string(argv[i]) == "-masterplan")
		{
			masterplan = argv[i + 1];
			i++;
		}

		if (string(argv[i]) == "-rd" || string(argv[i]) == "-rolesetdir")
		{
			rolesetdir = argv[i + 1];
			i++;
		}
		if (string(argv[i]) == "-r" || string(argv[i]) == "-roleset")
		{
			roleset = argv[i + 1];
			i++;
		}
	}
	if (masterplan.size() == 0 || rolesetdir.size() == 0)
	{
		cout << "Usage: Base -m [Masterplan] -rd [rolesetdir]" << endl;
		return 0;
	}
	cout << "Masterplan is: " << masterplan << endl;
	cout << "Rolset Directory is: " << rolesetdir << endl;
	cout << "Rolset is: " << roleset << endl;

	cout << "Constructing Base" << endl;
	Base* base = new Base(roleset, masterplan, rolesetdir);

	base->start();
	
	//If ROS --> need || !ROS --> comment out
	while (ros::ok())
	{
		std::chrono::milliseconds dura(500);
		std::this_thread::sleep_for(dura);
	}

	return 0;
}
```
###Base.h

``` cpp
#ifndef BASE_H_
#define BASE_H_

#include <iostream>

#include "engine/AlicaEngine.h"
#include "BehaviourCreator.h"
#include "ConditionCreator.h"
#include "UtilityFunctionCreator.h"
#include "ConstraintCreator.h"
#include "MSLWorldModel.h"

using namespace std;

namespace msl
{

	class Base
	{
	public:
		Base(string roleSetName, string masterPlanName, string roleSetDir);
		virtual ~Base();

		void start();

		alica::AlicaEngine* ae;
		alica::BehaviourCreator* bc;
		alica::ConditionCreator* cc;
		alica::UtilityFunctionCreator* uc;
		alica::ConstraintCreator* crc;
		MSLWorldModel* wm;

	protected:
		//blabla...
	};

} /* namespace msl */

#endif /* BASE_H_ */

```
