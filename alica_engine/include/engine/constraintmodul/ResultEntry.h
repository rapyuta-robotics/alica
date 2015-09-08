/*
 * ResultEntry.h
 *
 *  Created on: Nov 24, 2014
 *      Author: Philipp Sperber
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
		void addValue(long vid, shared_ptr<vector<uint8_t>> result);
		void clear();
		shared_ptr<vector<SolverVar*>> getCommunicatableResults(long ttl4Communication);
		shared_ptr<vector<uint8_t>> getValue(long vid, long ttl4Usage);
		shared_ptr<vector<shared_ptr<vector<uint8_t>>>> getValues(shared_ptr<vector<Variable*>> query, long ttl4Usage);

		class VarValue
		{
		public:
			long id;
			shared_ptr<vector<uint8_t>> val;
			ulong lastUpdate;

			VarValue(long vid, shared_ptr<vector<uint8_t>> v, ulong now)
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
