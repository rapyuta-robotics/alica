/*
 * IAlicaCommunication.h
 *
 *  Created on: Jun 24, 2014
 *      Author: Stephan Opfer
 */

#ifndef IALICACOMMUNICATION_H_
#define IALICACOMMUNICATION_H_

namespace alica
{

	class IAlicaCommunication
	{
	public:
		IAlicaCommunication();
		virtual ~IAlicaCommunication();

		virtual void sendAllocationAuthority() = 0;
		virtual void sendBehaviourEngineInfo() = 0;
		virtual void sendPlanTreeInfo() = 0;
		virtual void sendRoleSwitch() = 0;
		virtual void sendSolverResult() = 0;
		virtual void sendSyncReady() = 0;
		virtual void sendSyncTalk() = 0;


	};

} /* namespace alica */

#endif /* IALICACOMMUNICATION_H_ */
