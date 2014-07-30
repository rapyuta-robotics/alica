/*
 * DefaultUtilityFunction.cpp
 *
 *  Created on: Jul 30, 2014
 *      Author: Stefan Jakob
 */

#include <engine/DefaultUtilityFunction.h>
#include "engine/Assignment.h"
#include "engine/RunningPlan.h"
#include "engine/UtilityInterval.h"

namespace alica
{

	DefaultUtilityFunction::DefaultUtilityFunction(Plan* plan) :
			UtilityFunction("DefaultUtility", list<USummand*>(), 1.0, 0.0, plan)
	{
	}

	DefaultUtilityFunction::~DefaultUtilityFunction()
	{
	}

	double DefaultUtilityFunction::eval(RunningPlan* newRP, RunningPlan* oldRP)
	{
		if (newRP->getAssignment() == nullptr)
		{
			cerr << "DefUF: The Assignment of the RunningPlan is null!" << endl;
			throw new exception();
		}
		// Invalid Assignments have an Utility of -1 changed from 0 according to specs
		if (!newRP->getAssignment()->isValid())
		{
			return -1.0;
		}
		UtilityInterval* sumOfUI = new UtilityInterval(0.0, 0.0);
		double sumOfWeights = 0.0;

		// Sum up priority summand
		UtilityInterval* prioUI = this->getPriorityResult(newRP->getAssignment());
		sumOfUI->setMax(sumOfUI->getMax() + this->priorityWeight * prioUI->getMax());
		sumOfUI->setMin(sumOfUI->getMin() + this->priorityWeight * prioUI->getMin());
		sumOfWeights += this->priorityWeight;

		if (oldRP != nullptr && this->similarityWeight > 0.0)
		{
			// Sum up similarity summand
			UtilityInterval* simUI = this->getSimilarity(newRP->getAssignment(), oldRP->getAssignment());
			sumOfUI->setMax(sumOfUI->getMax() + this->similarityWeight * simUI->getMax());
			sumOfUI->setMin(sumOfUI->getMin() + this->similarityWeight * simUI->getMin());
			sumOfWeights += this->similarityWeight;
		}

		// Normalize to 0..1
		if (sumOfWeights > 0.0)
		{
			sumOfUI->setMax(sumOfUI->getMax() / sumOfWeights);
			sumOfUI->setMin(sumOfUI->getMin() / sumOfWeights);

			if ((sumOfUI->getMax() - sumOfUI->getMin()) > DIFFERENCETHRESHOLD)
			{
				cerr << "DefUF: The Min and Max utility differs more than " << DIFFERENCETHRESHOLD
						<< " for a complete Assignment!" << endl;
			}
			return sumOfUI->getMax();
		}

		return 0.0;
	}

	UtilityInterval* DefaultUtilityFunction::eva(IAssignment* newAss, IAssignment* oldAss)
	{
		UtilityInterval* sumOfUI = new UtilityInterval(0.0, 0.0);
		double sumOfWeights = 0.0;

		// Sum up priority summand
		UtilityInterval* prioUI = this->getPriorityResult(newAss);
		sumOfUI->setMax(sumOfUI->getMax() + this->priorityWeight * prioUI->getMax());
		sumOfUI->setMin(sumOfUI->getMin() + this->priorityWeight * prioUI->getMin());
		sumOfWeights += this->priorityWeight;
#ifdef UFDEBUG
		cout << "DF: prioUI.Min = " << prioUI->getMin() << endl;
		cout << "DF: prioUI.Max = " << prioUI->getMax() << endl;
		cout << "DF: priorityWeight = " << priorityWeight << endl;
#endif
		if (oldAss != nullptr && this->similarityWeight > 0.0)
		{
			// Sum up similarity summand
			UtilityInterval* simUI = this->getSimilarity(newAss, oldAss);
			sumOfUI->setMax(sumOfUI->getMax() + this->similarityWeight * simUI->getMax());
			sumOfUI->setMin(sumOfUI->getMin() + this->similarityWeight * simUI->getMin());
			sumOfWeights += this->similarityWeight;
		}

		// Normalize to 0..1
		if (sumOfWeights > 0.0)
		{
			sumOfUI->setMax(sumOfUI->getMax() / sumOfWeights);
			sumOfUI->setMin(sumOfUI->getMin() / sumOfWeights);
			return sumOfUI;
		}

		sumOfUI->setMin(0.0);
		sumOfUI->setMax(0.0);
		return sumOfUI;
	}

	string DefaultUtilityFunction::toString()
	{
		stringstream ss;
		ss << this->name << ": prioW: " << this->priorityWeight << " simW: " << this->similarityWeight << endl;
		return ss.str();

	}

} /* namespace alica */
