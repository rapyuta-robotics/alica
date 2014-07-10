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

	class USummand
	{
	public:

		virtual ~USummand()	{}
		virtual void init()
		{
			this->relevantEntryPoints = vector<EntryPoint*>(this->relevantEntryPointIds.size());
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
		virtual UtilityInterval* eval(IAssignment* ass);
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
		double weight;
		string name;
		long id;
		string info;
		vector<EntryPoint*> relevantEntryPoints;

	};

} /* namespace alica */

#endif /* USUMMAND_H_ */
