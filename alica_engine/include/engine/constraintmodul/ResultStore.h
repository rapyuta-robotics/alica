/*
 * ResultStore.h
 *
 *  Created on: Oct 10, 2014
 *      Author: Philipp
 */

#ifndef RESULTSTORE_H_
#define RESULTSTORE_H_

#include <memory>
#include <vector>

#include <NotifyTimer.h>

#include "engine/constraintmodul/IResultStore.h"

using namespace std;

namespace alica
{
	class Variable;
	class ResultEntry;
	class IAlicaCommunication;

	class ResultStore : public IResultStore
	{
	public:
		ResultStore(AlicaEngine* ae);
		virtual ~ResultStore();

		virtual void init();
		virtual void close();
		virtual void clear();
		virtual void onSolverResult(shared_ptr<SolverResult> msg);

		void publishContent();
		virtual void postResult(long vid, double result);
		virtual shared_ptr<vector<shared_ptr<vector<double>>>> getSeeds(shared_ptr<vector<Variable*>> query, shared_ptr<vector<shared_ptr<vector<double>>>> limits);

	protected:
		supplementary::NotifyTimer<ResultStore>* timer;
		long ttl4Communication;
		long ttl4Usage;

		class VotedSeed
		{
		public:
			VotedSeed(int dim, shared_ptr<vector<double>> v);

			bool takeVector(shared_ptr<vector<double>> v, vector<double>& scaling, double distThreshold);

			shared_ptr<vector<double>> values;
			vector<int> supporterCount;
			int totalSupCount;
			int dim;
		};

	private:
		AlicaEngine* ae;
		int ownId;
		IAlicaCommunication* communicator;
		bool running;
		bool communicationEnabled;

		vector<shared_ptr<ResultEntry>> store;
		shared_ptr<ResultEntry> ownResults;
		double distThreshold;
	};
} /* namespace alica */

#endif /* RESULTSTORE_H_ */
