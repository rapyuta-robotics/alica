/*
 * AlicaEngine.h
 *
 *  Created on: Mar 3, 2014
 *      Author: Stephan Opfer
 */

#ifndef ALICAENGINE_H_
#define ALICAENGINE_H_

#include <string>

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

};

} /* namespace Alica */

#endif /* ALICAENGINE_H_ */
