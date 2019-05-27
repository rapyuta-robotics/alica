# Example: Hairy's ProcessManager should stop the processes with the Id 1, 2 and 3, if they run with ROBOT=nase 
# receiverId = 8, robotId = [9], cmd = 1, processKeys = [1,2,3]

#constants for cmd
uint8 START = 0
uint8 STOP = 1
uint8 START_LOG_PUBLISHING = 2
uint8 STOP_LOG_PUBLISHING = 3

# Cmd has to be set to one of the values above. 
uint8 cmd

# ReceiverId determines the ProcessManager-Instanz, which is addressed by this message. 0 means every ProcessManager
id_ros/ID receiver_id

# RobotId determines the ROBOT-Environment Variable. 
id_ros/ID[] robot_ids

# Array of process IDs from the Processes.conf file.
int32[] process_keys

# Array of paramSet IDs from the Processes.conf file.
int32[] param_sets

