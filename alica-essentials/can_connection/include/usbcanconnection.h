#ifndef USBCANCONNECTION_H
#define USBCANCONNECTION_H 1

#include <errno.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <pthread.h>
#include <poll.h>
//#include <fcntl.h>
#include "canlistener.h"
#include "canconnection.h"

#ifndef CAN_TIMEOUT
#define CAN_TIMEOUT 800
#endif

class UsbCanConnection : public CanConnection {
	public:
		UsbCanConnection(const char* strCanIf);
		int Start();
		int Stop();
		void SetReceiver(CanListener* ul);
		int SendCanMsg(unsigned short canid,unsigned char* data,int len);
		int SendExCanMsg(unsigned int canid,unsigned char* data,int len);
		int SendCanRtr(unsigned short canid);
		int SendExCanRtr(unsigned int canid);
		int WaitForCanMsg(unsigned int msgid, unsigned char* buffer, int* len, int timeout);


	protected:
		void* readLoop();
		int initSocket();

		CanListener* listener;

		int socketfd;
		struct sockaddr_can addr;
		struct can_frame rx_frame;
		struct can_frame tx_frame;
		struct ifreq ifr;
		struct iovec iov;
		struct msghdr msg;

		int writeOk;

		char ctrlmsg[CMSG_SPACE(sizeof(struct timeval)) + CMSG_SPACE(sizeof(__u32))];

		pthread_t listenerThread;

		unsigned short waitOnId;
		unsigned char explicitAnswerBuf[64];
		volatile int stopThread;

		static void* _thread(void* This);
};


#endif
