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

namespace alica
{
	class Variable;
	class RunningPlan;

	class BasicBehaviour
	{
	public:
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
		int getDueTime() const;
		void setDueTime(int dueTime);
		int getPeriod() const;
		void setPeriod(int period);
		const shared_ptr<RunningPlan>& getRunningPlan() const;
		void setRunningPlan(const shared_ptr<RunningPlan>& runningPlan);

	protected:
		string name;
		shared_ptr<map<string,string>> parameters;
		shared_ptr<list<Variable*>> variables;
		shared_ptr<RunningPlan> runningPlan;
		int period;
		int dueTime;
		int getOwnId();

	private:


	};
} /* namespace alica */

#endif /* BASICBEHAVIOUR_H_ */
