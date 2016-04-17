//
// Created by marci on 17.04.16.
//

#ifndef SUPPLEMENTARY_WRAPPEDMESSAGEHANDLER_H
#define SUPPLEMENTARY_WRAPPEDMESSAGEHANDLER_H

#include <ros/ros.h>

#include "TTBWorldModel.h"
<?messageIncludes?>

namespace ttb
{

    class WrappedMessageHandler
    {
    private:
        int robotID;
        ros::NodeHandle* n;
        // get incoming wrapped messages and publish them (unwrapped) on the local ros core





    public:
        <?rosPublisherDecl?>

        <?rosMessageHandler?>
        void init(int& id, ros::NodeHandle* nh, TTBWorldModel* those)
        {
            this->robotID = id;
            this->n = nh;

            <?subscriptions?>

            <?advertisement?>

            //wrappedMessagesSubscribers.push_back(nh.subscribe("/wrapped", 10, &WrappedMessageHandler::onWrappedMessage, (TTBWorldModel*)those));
        }


    };

}

#endif //SUPPLEMENTARY_WRAPPEDMESSAGEHANDLER_H
