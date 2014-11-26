/*
 * ResultEntry.h
 *
 *  Created on: Nov 24, 2014
 *      Author: Philipp
 */

#ifndef RESULTENTRY_H_
#define RESULTENTRY_H_

#include <list>
#include <map>
#include <vector>
#include <mutex>
#include <memory>

using namespace std;


namespace alica
{
	class AlicaEngine;
	struct SolverVar;
	class Variable;


	class ResultEntry
	{
	public:
		ResultEntry(int robotId, AlicaEngine* ae);
		virtual ~ResultEntry();

		int getId();
		void addValue(long vid, double val);
		void clear();
		shared_ptr<vector<SolverVar*>> getCommunicatableResults(long ttl4Communication);
		double getValue(long vid, long ttl4Usage);
		shared_ptr<vector<double>> getValues(shared_ptr<vector<Variable*>> query, long ttl4Usage);

		class VarValue
		{
		public:
			long id;
			double val;
			ulong lastUpdate;

			VarValue(long vid, double v, ulong now)
			{
				this->id = vid;
				this->val = v;
				this->lastUpdate = now;
			}
		};

	protected:
		int id;
		AlicaEngine* ae;
		map<long, shared_ptr<VarValue>> values;
		mutex valueLock;
	};

} /* namespace alica */

#endif /* RESULTENTRY_H_ */
