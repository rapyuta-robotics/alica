/*
 * BasicBehaviour.h
 *
 *  Created on: Jun 4, 2014
 *      Author: Stefan Jakob
 */

#ifndef BASICBEHAVIOUR_H_
#define BASICBEHAVIOUR_H_

using namespace std;

#include "engine/IRobotID.h"

#include <string>
#include <iostream>
#include <map>
#include <memory>
#include <list>
#include <thread>
#include <chrono>
#include <condition_variable>
#include <vector>

namespace supplementary
{
	class Timer;
	class ITrigger;
}

namespace alica
{
	class Variable;
	class RunningPlan;
	class BehaviourConfiguration;
	class EntryPoint;

	/**
	 * The base class for all behaviours. All Behaviours must inherit from this class.
	 */
	class BasicBehaviour
	{
	public:
		BasicBehaviour(string name);
		virtual ~BasicBehaviour();
		virtual void run(void* msg) = 0;
		const string getName() const;
		void setName(string name);
		shared_ptr<map<string, string>> getParameters();
		void setParameters(shared_ptr<map<string, string>> parameters);
		shared_ptr<list<Variable*>> getVariables();
		Variable* getVariablesByName(string name);
		void setVariables(shared_ptr<list<Variable*>> variables);
		bool stop();
		bool start();
		int getDelayedStart() const;
		void setDelayedStart(long msDelayedStart);
		int getInterval() const;
		void setInterval(long msInterval);
		shared_ptr<RunningPlan> getRunningPlan();
		void setRunningPlan(shared_ptr<RunningPlan> runningPlan);
		bool isSuccess() const;
		void setSuccess(bool success);
		bool isFailure() const;
		void setFailure(bool failure);

		bool getParameter(string key, string& valueOut);
		void setTrigger(supplementary::ITrigger* trigger);

		void sendLogMessage(int level, string& message);
	protected:
		/**
		 * The name of this behaviour.
		 */
		string name;
		/**
		 * Parameters are behaviour configuration specific fixed values. They are set before the behaviour is activated.
		 */
		shared_ptr<map<string, string>> parameters;
		/**
		 * The set of Variables attached to this behaviours as defined by the BehaviourConfiguration.
		 */
		shared_ptr<list<Variable*>> variables;
		/**
		 * The running plan representing this behaviour within the PlanBase.
		 */
		shared_ptr<RunningPlan> runningPlan;
		chrono::milliseconds msInterval;
		chrono::milliseconds msDelayedStart;
		/**
		 * is always true except when the behaviour is shutting down
		 */
		bool started;
		bool callInit;


		/**
		 * Tells us whether the behaviour is currently running (or active)
		 */
		bool running;

		thread* runThread; /** < executes the runInternal and thereby the abstract run method */
		supplementary::Timer* timer; /** < triggers the condition_variable of the runThread, if this behaviour is timer triggered, alternative to behaviourTrigger*/
		supplementary::ITrigger* behaviourTrigger; /** triggers the condition_variable of the runThread, if this behaviour is event triggered, alternative to timer */
		const alica::IRobotID* getOwnId();

		/**
		 * Called whenever a basic behaviour is started, i.e., when the corresponding state is entered.
		 * Override for behaviour specific initialisation.
		 */
		virtual void initialiseParameters()
		{
		};

		EntryPoint* getParentEntryPoint(string taskName);

		EntryPoint* getHigherEntryPoint(string planName, string taskName);

		shared_ptr<vector<const alica::IRobotID*>> robotsInEntryPointOfHigherPlan(EntryPoint* ep);

		shared_ptr<vector<const alica::IRobotID*>> robotsInEntryPoint(EntryPoint* ep);

	private:
		void runInternal();
		void initInternal();

		mutex runCV_mtx;
		/**
		 * The Success flag. Raised by a behaviour to indicate it reached whatever it meant to reach.
		 */
		bool success;
		/**
		 * The Failure flag. Raised by a behaviour to indicate it has failed in some way.
		 */
		bool failure;

	protected:
		condition_variable runCV;

	};
} /* namespace alica */

#endif /* BASICBEHAVIOUR_H_ */
