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
#include "Variable.h"
#include "RuntimeCondition.h"
#include "PreCondition.h"
#include "../UtilityFunction.h"
namespace alica
{
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
		const RuntimeCondition* getRuntimeCondition() const;
		void setRuntimeCondition(const RuntimeCondition* runtimeCondition);
		const PreCondition* getPreCondition() const;
		void setPreCondition(const PreCondition* preCondition);
		const UtilityFunction& getUtilityFunction() const;
		void setUtilityFunction(const UtilityFunction& utilityFunction);
		double getUtilityThreshold() const;
		void setUtilityThreshold(double utilityThreshold = 1.0);

	private:
		unsigned long authorithyTimeInterval;
		const RuntimeCondition* runtimeCondition;
		const PreCondition* preCondition;
		UtilityFunction utilityFunction;

	protected:
		string fileName;
		bool masterPlan;
		list<Variable*> variables;
		double utilityThreshold = 1.0;
	};

} /* namespace Alica */

#endif /* ABSTRACTPLAN_H_ */
