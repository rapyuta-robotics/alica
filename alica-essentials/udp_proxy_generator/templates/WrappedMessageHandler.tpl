#pragma once

#include <ros/ros.h>

#include "TTBWorldModel.h"
<?messageIncludes?>

namespace ttb {
    class WrappedMessageHandler {
    private:
        int robotID;
        ros::NodeHandle n;
        // get incoming wrapped messages and publish them (unwrapped) on the local ros core

    public:
        <?rosPublisherDecl?>
        <?rosMessageHandler?>

        void init(int& id) {
            this->robotID = id;

            <?subscriptions?>

            <?advertisement?>

            // wrappedMessagesSubscribers.push_back(nh.subscribe("/wrapped", 10,
            // &WrappedMessageHandler::onWrappedMessage, (TTBWorldModel*)those));
        }
    };
}
