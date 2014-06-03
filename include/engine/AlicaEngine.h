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

	class AlicaEngine
	{
	public:
		static AlicaEngine* getInstance();
		void init(string roleSetName, string masterPlanName, string roleSetDir,
		bool stepEngine);
		void start();bool getStepEngine();
		void abort (string msg);

	protected:
		supplementary::SystemConfig* sc;
		shared_ptr<Plan> masterPlan;

	private:
		// private constructur/ destructor because of singleton
		AlicaEngine();
		~AlicaEngine();

		bool stepEngine;
		void setStepEngine(bool stepEngine);

		shared_ptr<PlanRepository> planRepository;
		shared_ptr<IPlanParser> planParser;
	};

} /* namespace Alica */

#endif /* ALICAENGINE_H_ */
