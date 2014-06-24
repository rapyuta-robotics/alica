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

		virtual void SendAllocationAuthority() = 0;
		virtual void SendBehaviourEngineInfo() = 0;
		virtual void SendPlanTreeInfo() = 0;
		virtual void SendRoleSwitch() = 0;
		virtual void SendSolverResult() = 0;
		virtual void SendSyncReady() = 0;
		virtual void SendSyncTalk() = 0;


	};

} /* namespace alica */

#endif /* IALICACOMMUNICATION_H_ */
