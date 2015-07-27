# ReceiverID 0 is reserved as wildcard for all robots. If it is > 0, it represents the ID of the robot.
int32 receiverId

# The following commands are defined
uint8 START = 0
uint8 STOP = 1

uint8 cmd