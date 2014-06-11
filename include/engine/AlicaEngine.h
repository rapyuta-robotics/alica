/*
 * AlicaEngine.h
 *
 *  Created on: Mar 3, 2014
 *      Author: Stephan Opfer
 */

#ifndef ALICAENGINE_H_
#define ALICAENGINE_H_

#include <string>

#include <SystemConfig.h>
#include "engine/PlanRepository.h"
#include "engine/IPlanParser.h"
#include "engine/parser/PlanParser.h"
#include "engine/model/Plan.h"

using namespace std;

namespace alica
{
	class PlanRepository;
	class AlicaEngine
	{
	public:
		static AlicaEngine* getInstance();
		void init(string roleSetName, string masterPlanName, string roleSetDir,
		bool stepEngine);
		void start();bool getStepEngine();
		void abort(string msg);
		template<typename T> void abort(string msg, const T tail);
		shared_ptr<PlanRepository> getPlanRepository();

	protected:
		supplementary::SystemConfig* sc;
		Plan* masterPlan;

	private:
		// private constructur/ destructor because of singleton
		AlicaEngine();
		~AlicaEngine();

		bool stepEngine;
		void setStepEngine(bool stepEngine);

		shared_ptr<PlanRepository> planRepository;
		shared_ptr<IPlanParser> planParser;
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
