/*
 * USummand.h
 *
 *  Created on: Jul 9, 2014
 *      Author: Stefan Jakob
 */

#ifndef USUMMAND_H_
#define USUMMAND_H_

using namespace std;

#include <vector>
#include <string>
#include <sstream>
#include <map>

#include "engine/AlicaEngine.h"
#include "engine/PlanRepository.h"

namespace alica
{

	class UtilityInterval;
	class EntryPoint;
	class IAssignment;

	/**
	 * Abstract super class for domain dependent utility summands.
	 */
	class USummand
	{
	public:

		virtual ~USummand()	{}
		/**
		 * Searches every needed entrypoint in the hashtable of the xmlparser
		 * and stores it in the relevant entrypoint vector. This will increase the
		 * performance of the evaluation of this utility summand.
		 */
		virtual void init()
		{
			// init relevant entrypoint vector
			this->relevantEntryPoints = vector<EntryPoint*>(this->relevantEntryPointIds.size());
			// find the right entrypoint for each id in relevant entrypoint id
			map<long, EntryPoint*> elements = AlicaEngine::getInstance()->getPlanRepository()->getEntryPoints();
			EntryPoint* curEp;
			for(int i = 0; i < this->relevantEntryPoints.size(); ++i)
			{
				auto iter = elements.find(this->relevantEntryPointIds[i]);
				if(iter != elements.end())
				{
					curEp = iter->second;
				}
				else
				{
					cerr << "Could not find Entrypoint " << this->relevantEntryPointIds[i] << " Hint is: " << this->name << endl;
					throw new exception();
				}
				if(curEp != nullptr)
				{
					this->relevantEntryPoints[i] = curEp;
				}
			}
		}
		string toString()
		{
			stringstream ss;
			ss << this->name << ": Weight " << this->weight << "EntryPoints: ";
			for(int i = 0; i < this->relevantEntryPointIds.size(); ++i)
			{
				ss << this->relevantEntryPointIds[i] << " ";
			}
			ss << endl;
			return ss.str();
		}
		double getWeight() const
		{
			return weight;
		}
		/**
		 * Evaluates the utilityfunction summand
		 * @return The result of the evaluation
		 */
		virtual UtilityInterval* eval(IAssignment* ass);
		/**
		 * Cache every data for the current evaluation, to
		 * assure consistency over the complete current evaluation.
		 */
		virtual void cacheEvalData();
		virtual pair<vector<double>, double>* differentiate(IAssignment* newAss)
		{
			return nullptr;
		}
		void setWeight(double weight)
		{
			this->weight = weight;
		}

	protected:
		UtilityInterval ui;
		vector<long> relevantEntryPointIds;
		/**
		 * Weight of this UtilitySummand
		 */
		double weight;
		string name;
		long id;
		string info;
		vector<EntryPoint*> relevantEntryPoints;

	};

} /* namespace alica */

#endif /* USUMMAND_H_ */
