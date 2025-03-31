#pragma once

#include "communication/AlicaRosCommunicationCommon.h"

#include <ros/callback_queue.h>
#include <ros/ros.h>

using namespace alica;

namespace alicaRosProxy
{

class AlicaRosCommunication : public AlicaRosCommunicationCommon
{
public:
    AlicaRosCommunication(const AlicaCommunicationHandlers& callbacks, ros::CallbackQueue& cb_queue = *ros::getGlobalCallbackQueue());
    virtual ~AlicaRosCommunication();

private:
    ros::NodeHandle _nh;

    ros::Subscriber _syncReadySubscriber;
    ros::Subscriber _planTreeInfoSubscriber;
    ros::Subscriber _solverResultSubscriber;
    ros::Subscriber _presenceAnnouncementSubscriber;
    ros::Subscriber _allocationAuthorityInfoSubscriber;
    ros::Subscriber _syncTalkSubscriber;
    ros::Subscriber _presenceQuerySubscriber;
};

} /* namespace alicaRosProxy */
