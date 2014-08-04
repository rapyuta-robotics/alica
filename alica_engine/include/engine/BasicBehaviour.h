/*
 * BasicBehaviour.h
 *
 *  Created on: Jun 4, 2014
 *      Author: Stefan Jakob
 */

#ifndef BASICBEHAVIOUR_H_
#define BASICBEHAVIOUR_H_

using namespace std;

#include <string>
#include <iostream>
#include <map>
#include <memory>
#include <list>
#include <thread>
#include <chrono>
#include <condition_variable>
namespace supplementary {
	class Timer;
}

namespace alica
{
	class Variable;
	class RunningPlan;
	class BehaviourConfiguration;

	class BasicBehaviour
	{
	public:
		BasicBehaviour();
		BasicBehaviour(string name);
		virtual ~BasicBehaviour();
		virtual void run(void* msg) = 0;
		const string getName() const;
		void setName(string name);
		shared_ptr<map<string,string>> getParameters();
		void setParameters(shared_ptr<map<string,string>> parameters);
		shared_ptr<list<Variable*>> getVariables();
		void setVariables(shared_ptr<list<Variable*>> variables);
		void start();
		void stop();
		bool pause();
		bool restart();
		int getDelayedStart() const;
		void setDelayedStart(long msDelayedStart);
		int getInterval() const;
		void setInterval(long msInterval);
		const shared_ptr<RunningPlan>& getRunningPlan() const;
		void setRunningPlan(const shared_ptr<RunningPlan>& runningPlan);
		bool isSuccess() const;
		bool isFailure() const;

	protected:
		string name;
		shared_ptr<map<string,string>> parameters;
		shared_ptr<list<Variable*>> variables;
		shared_ptr<RunningPlan> runningPlan;
		chrono::milliseconds msInterval;
		chrono::milliseconds msDelayedStart;
		bool running, started, callInit, success, failure;
		thread* runThread; /** < executes the runInternal and thereby the abstract run method */
		supplementary::Timer* timer; /** < triggers the condition_variable of the runThread, if this behaviour is timer triggered */
		int getOwnId();

		/**
		 * Called whenever a basic behaviour is started, i.e., when the corresponding state is entered.
		 * Override for behaviour specific initialisation.
		 */
		virtual void initialiseParameters () {};

	private:
		mutex runCV_mtx;
		condition_variable runCV;
		void runInternal();
		void initInternal();


	};
} /* namespace alica */

#endif /* BASICBEHAVIOUR_H_ */
