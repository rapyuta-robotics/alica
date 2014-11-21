uint8 START = 0
uint8 STOP = 1

# RobotId determines the ROBOT-Environment Variable. 
uint8 robotId

# Cmd has to be set to one of the values above. 
uint8 cmd

# Array of IDs from the Processes.conf file.
uint8[] processKeys

# Example: Stop the processes with the Id 1, 2 and 3, if they run with ROBOT=nase 
# robotId = 9, cmd = 1, processKeys = [1,2,3]
 
