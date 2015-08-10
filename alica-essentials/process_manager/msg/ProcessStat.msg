# RobotId determines the ROBOT-Environment Variables, under which the reported process is running.
int32 robotId

# Process ID from the Processes.conf file.
int32 processKey

# Parameter Set ID from the Processes.conf file
int32 paramSet

# State has to be set to one of the values above. 
uint8 state

# cpu is the percentage of required cpu resources of this process (we need uint16 as one process sometimes needs more than 255 %)
uint16 cpu

# mem is the amount of MB this process needs.
uint32 mem

# flag whether logs are published
uint8 publishing

# constants for publishing
uint8 PUBLISHING_OFF = 0
uint8 PUBLISHING_ON = 1
