/*
 * AlicaEngine.h
 *
 *  Created on: Mar 3, 2014
 *      Author: Stephan Opfer
 */

#ifndef ALICAENGINE_H_
#define ALICAENGINE_H_

using namespace std;

#include <string>

#include <SystemConfig.h>

namespace alica
{
	class PlanRepository;
	class Plan;
	class IPlanParser;
	class IBehaviourPool;

	class AlicaEngine
	{
	public:
		static AlicaEngine* getInstance();
		void init(string roleSetName, string masterPlanName, string roleSetDir, bool stepEngine);
		void start();
		bool getStepEngine();
		void abort(string msg);
		template<typename T> void abort(string msg, const T tail);
		unique_ptr<PlanRepository> getPlanRepository();
		unique_ptr<IBehaviourPool> getBehaviourPool();

	protected:
		supplementary::SystemConfig* sc;
		Plan* masterPlan;

	private:
		// private constructur/ destructor because of singleton
		AlicaEngine();
		~AlicaEngine();

		bool stepEngine;
		void setStepEngine(bool stepEngine);

		PlanRepository* planRepository;
		IPlanParser* planParser;
		IBehaviourPool* behaviourPool;

	};

	template<typename T>
	void AlicaEngine::abort(string msg, const T tail)
	{
		stringstream ss;
		ss << msg << tail;
		AlicaEngine::abort(ss.str());
	}

} /* namespace Alica */

#endif /* ALICAENGINE_H_ */
