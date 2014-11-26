/*
 * IResultStore.h
 *
 *  Created on: Oct 10, 2014
 *      Author: Philipp
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

	class IResultStore : public enable_shared_from_this<IResultStore>
	{
	public:
		virtual ~IResultStore() {};

		virtual void init() = 0;
		virtual void close() = 0;
		virtual void clear() = 0;
		virtual void onSolverResult(shared_ptr<SolverResult> msg) = 0;

		virtual void postResult(long vid, double result) = 0;
		virtual shared_ptr<vector<vector<double>>> getSeeds(shared_ptr<vector<Variable*>> query, shared_ptr<vector<vector<double>>> limits) = 0;
	};
} /* namespace alica */

#endif /* RESULTSTORE_H_ */
