/*
 * CycleManager.h
 *
 *  Created on: Jul 10, 2014
 *      Author: Paul Panin
 */

#ifndef CYCLEMANAGER_H_
#define CYCLEMANAGER_H_

namespace alica
{
	class RunningPlan;

	class CycleManager
	{
	public:
		CycleManager(RunningPlan* p);
		virtual ~CycleManager();
		//TODO nicht implementiert
		void update();
		bool isOverridden();
		bool setAssignment(RunningPlan* r);
		bool mayDoUtilityCheck();
	};

} /* namespace supplementary */

#endif /* CYCLEMANAGER_H_ */
