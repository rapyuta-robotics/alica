/*
 * AbstractPlan.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef ABSTRACTPLAN_H_
#define ABSTRACTPLAN_H_

using namespace std;

#include <list>
#include <SystemConfig.h>
#include <string>
#include <algorithm>

#include "AlicaElement.h"

namespace alica
{

	class Variable;
	class PreCondition;
	class RuntimeCondition;
	class UtilityFunction;

	class AbstractPlan : public AlicaElement
	{
	public:
		AbstractPlan();
		virtual ~AbstractPlan();

		bool containsVar(const Variable* v);
		bool containsVar(string name);

		bool isMasterPlan() const;
		void setMasterPlan(bool isMasterPlan);
		virtual string toString() const;
		unsigned long getAuthorithyTimeInterval() const;
		void setAuthorithyTimeInterval(unsigned long authorithyTimeInterval);
		const virtual string& getFileName() const;
		virtual void setFileName(const string& fileName);
		list<Variable*>& getVariables();
		void setVariables(const list<Variable*>& variables);
		RuntimeCondition* getRuntimeCondition();
		void setRuntimeCondition(RuntimeCondition* runtimeCondition);
		PreCondition* getPreCondition();
		void setPreCondition(PreCondition* preCondition);
		const UtilityFunction* getUtilityFunction() const;
		void setUtilityFunction(UtilityFunction* utilityFunction);
		double getUtilityThreshold() const;
		void setUtilityThreshold(double utilityThreshold = 1.0);

	private:
		unsigned long authorithyTimeInterval;
		RuntimeCondition* runtimeCondition;
		PreCondition* preCondition;
		UtilityFunction* utilityFunction;

	protected:
		string fileName;
		bool masterPlan;
		list<Variable*> variables;
		double utilityThreshold = 1.0;
	};

} /* namespace Alica */

#endif /* ABSTRACTPLAN_H_ */
