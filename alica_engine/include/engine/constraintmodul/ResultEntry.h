#pragma once

#include "engine/IRobotID.h"

#include <list>
#include <map>
#include <vector>
#include <mutex>
#include <memory>

namespace alica
{
	class AlicaEngine;
	struct SolverVar;
	class Variable;


	class ResultEntry
	{
	public:
		ResultEntry(const alica::IRobotID* robotId, const AlicaEngine* ae);
		virtual ~ResultEntry();

		const alica::IRobotID* getId();
		void addValue(long vid, std::shared_ptr<std::vector<uint8_t>> result);
		void clear();
		std::shared_ptr<std::vector<SolverVar*>> getCommunicatableResults(long ttl4Communication);
		std::shared_ptr<std::vector<uint8_t>> getValue(long vid, long ttl4Usage);
		std::shared_ptr<std::vector<std::shared_ptr<std::vector<uint8_t>>>> getValues(std::shared_ptr<std::vector<Variable*>> query, long ttl4Usage);

		class VarValue
		{
		public:
			long id;
			std::shared_ptr<std::vector<uint8_t>> val;
			ulong lastUpdate;

			VarValue(long vid, std::shared_ptr<std::vector<uint8_t>> v, ulong now)
			{
				this->id = vid;
				this->val = v;
				this->lastUpdate = now;
			}
		};

	protected:
		const alica::IRobotID* id;
		const AlicaEngine* ae;
		std::map<long, std::shared_ptr<VarValue>> values;
		std::mutex valueLock;
	};

} /* namespace alica */
