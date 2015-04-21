/*
 * AbstractPlan.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef ABSTRACTPLAN_H_
#define ABSTRACTPLAN_H_


#include <list>
#include <SystemConfig.h>
#include <string>
#include <algorithm>

#include "AlicaElement.h"
#include "engine/IAlicaClock.h"

using namespace std;
namespace alica
{

	class Variable;
	class PreCondition;
	class RuntimeCondition;
	class UtilityFunction;

	/**
	 * Super class of plans, plantypes and behaviourconfigurations.
	 */
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
		alicaTime getAuthorityTimeInterval() const;
		void setAuthorityTimeInterval(alicaTime authorityTimeInterval);
		const virtual string& getFileName() const;
		virtual void setFileName(const string& fileName);
		shared_ptr<list<Variable*>> getVariables();
		void setVariables(shared_ptr<list<Variable*>> variables);
		RuntimeCondition* getRuntimeCondition();
		void setRuntimeCondition(RuntimeCondition* runtimeCondition);
		PreCondition* getPreCondition();
		void setPreCondition(PreCondition* preCondition);
		shared_ptr<UtilityFunction> getUtilityFunction();
		void setUtilityFunction(shared_ptr<UtilityFunction> utilityFunction);
		double getUtilityThreshold() const;
		void setUtilityThreshold(double utilityThreshold = 1.0);

	private:
		alicaTime authorityTimeInterval;
		/**
		 * This plan's runtime condition.
		 */
		RuntimeCondition* runtimeCondition;
		/**
		 * This plan's precondition
		 */
		PreCondition* preCondition;
		/**
		 * This plan's Utility function
		 */
		shared_ptr<UtilityFunction> utilityFunction;

	protected:
		string fileName;
		/**
		 *  Whether this plan is marked as a MasterPlan.
		 */
		bool masterPlan;
		shared_ptr<list<Variable*>> variables;
		/**
		 * The utility threshold, the higher, the less likely dynamic changes are.
		 */
		double utilityThreshold = 1.0;
	};

} /* namespace Alica */

#endif /* ABSTRACTPLAN_H_ */
