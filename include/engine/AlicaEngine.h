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

using namespace std;

namespace alica
{

class AlicaEngine
{
public:
	AlicaEngine();
	virtual ~AlicaEngine();
	void Init(string roleSetName, string masterPlanName, string roleSetDir, bool stepEngine);
	void Start();
	bool GetStepEngine();

protected:
	supplementary::SystemConfigPtr sc;

private:
	bool stepEngine;
	shared_ptr<PlanRepository> planRepository;

	void SetStepEngine(bool stepEngine);

};

} /* namespace Alica */

#endif /* ALICAENGINE_H_ */
