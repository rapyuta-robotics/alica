#ifndef CANCONNECTION_CAN_H
#define CANCONNECTION_CAN_H 1
	/// CanDevice is used as reciever or sender id
	enum CanDevice {
		Eth2Can         = 0x00u,
		MotorController = 0x20u,
		Compass         = 0x40u,
		ReKick          = 0x60u,
		TomServo        = 0x80u,
		BallHandler		= 0xA0u,
		KickerSwitch	= 0xC0u,
		MonitoringBoard = 0xE0u,
	};

	enum CanPriority {
		CanPriHigh = 0x20u,
		CanPriNorm = 0x40u,
		CanPriLow  = 0x80u,
	};
#endif //CANCONNECTION_CAN_H
