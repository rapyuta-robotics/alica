/*
 * IResultStore.h
 *
 *  Created on: Oct 10, 2014
 *      Author: Philipp Sperber
 */

#ifndef IRESULTSTORE_H_
#define IRESULTSTORE_H_

#include <memory>
#include <vector>

using namespace std;

namespace alica
{
	class AlicaEngine;
	class Variable;
	struct SolverResult;

	class IVariableSyncModule : public enable_shared_from_this<IVariableSyncModule>
	{
	public:
		virtual ~IVariableSyncModule() {};

		virtual void init() = 0;
		virtual void close() = 0;
		virtual void clear() = 0;
		virtual void onSolverResult(shared_ptr<SolverResult> msg) = 0;

		virtual void postResult(long vid, shared_ptr<vector<uint8_t>>& result) = 0;
		virtual shared_ptr<vector<shared_ptr<vector<shared_ptr<vector<uint8_t>>>>>> getSeeds(shared_ptr<vector<Variable*>> query, shared_ptr<vector<shared_ptr<vector<double>>>> limits) = 0;
	};
} /* namespace alica */

#endif /* RESULTSTORE_H_ */
