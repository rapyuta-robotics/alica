#include "usbcanconnection.h"



#define DEBUG_SEND 0
#define DEBUG_RECV 0

#define USBCANRCVTIMEOUT 300000

UsbCanConnection::UsbCanConnection(const char* strCanIf)  {
			strcpy(ifr.ifr_name, strCanIf);
			socketfd = -1;
			listener = NULL;
			waitOnId = 0;
			stopThread = 0;
}

int UsbCanConnection::Start() {
	if (socketfd > 0) return -1;

	/* open socket */
	while(initSocket()<0) {
		usleep(100000);
	}
	//if (initSocket()<0) return -1;

	int ret = pthread_create(&listenerThread,NULL,(void* (*)(void*))&UsbCanConnection::_thread,this);
	if (ret!=0) {
		printf("Thread creation failed\n");
		return -1;
	}
	printf("Started Listening\n");
	return 1;
}

int UsbCanConnection::Stop() {
	if (socketfd < 0) return -1;
    stopThread = 1;
	return shutdown(socketfd, 2);
};

void UsbCanConnection::SetReceiver(CanListener* cl) {
	listener = cl;
};
int UsbCanConnection::initSocket() {
	if (socketfd > 0) {
		close(socketfd);
	}
	if ((socketfd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("socket");
		return -1;
	}

	addr.can_family = AF_CAN;


	if (ioctl(socketfd, SIOCGIFINDEX, &ifr) < 0) {
		perror("SIOCGIFINDEX");
		return -1;
	}
	addr.can_ifindex = ifr.ifr_ifindex;

	struct timeval t;
	t.tv_sec = 0;
	t.tv_usec = USBCANRCVTIMEOUT;
		
	if (bind(socketfd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("bind");
		return -1;
	}
	setsockopt(socketfd, SOL_SOCKET, SO_RCVTIMEO, &t,sizeof(timeval));

	writeOk = 1;
	return 1;

}
void* UsbCanConnection::readLoop() {
	int result;
	unsigned int canid;
	int dlen;

	iov.iov_base = &rx_frame;
	msg.msg_name = &addr;
	msg.msg_iov = &iov;
	msg.msg_iovlen = 1;
	msg.msg_control = &ctrlmsg;

	//pollsocket.fd = socketfd;
	//pollsocket.events = POLLIN | POLLPRI;
	//pollsocket.revents = POLLERR | POLLHUP | POLLNVAL;
	
	
	while(!stopThread) {
		if (writeOk < 0) {
			if(initSocket()<0) {
				usleep(20000);
			}
			//printf("Socket reinitialised!\n");
		}
		else if (socketfd > 0) {

				
				
				//int sres= poll(&pollsocket, 1,1);
				//printf("SRES %d\n",sres);
				/* these settings may be modified by recvmsg() */
				iov.iov_len = sizeof(rx_frame);
				msg.msg_namelen = sizeof(addr);
				msg.msg_controllen = sizeof(ctrlmsg);  
				msg.msg_flags = 0;

				result = recvmsg(socketfd, &msg, 0);

				if (result < 0) {
					//perror("read");
				}
				else {
					if (result < (int)sizeof(struct can_frame)) {
						fprintf(stderr, "read: incomplete CAN frame\n");
					}
				}
				if (result > 0) {
					if(result < 5) {
						fprintf(stderr, "read: incomplete CAN frame\n");
					} else {
						canid =  (unsigned int)rx_frame.can_id;						
						dlen  = rx_frame.can_dlc;					
#if DEBUG_RECV

	printf("<< %#x l: %d [",canid,dlen);

	for(int i=0; i<dlen; i++) {
		printf("%#x ",rx_frame.data[i]);
	}
	printf("]\n");

#endif
					if(waitOnId && canid == waitOnId) {
						explicitAnswerBuf[0] = dlen;
						memcpy(explicitAnswerBuf+1,rx_frame.data,dlen);
						waitOnId = 0;
					}
					else {
						if (listener != NULL) listener->Receive(canid,rx_frame.data,dlen);
					}
				}
			}
			
		} else {
			fprintf(stderr,"Unexpected socket failure!\n");
			usleep(500000);
		}
	}
	return NULL;
}
int UsbCanConnection::SendExCanMsg(unsigned int canid,unsigned char* data,int len) {
	if (len<0 || len>8) {
		printf("Data length is out of range in SendCanMsg: %d\n",len);
		return -1;
	}
	/* set frame */
	tx_frame.can_id = canid;
	tx_frame.can_dlc = len;
	tx_frame.can_id |=  CAN_EFF_FLAG;
	memcpy(tx_frame.data,data,len);
	/* send frame */

#if DEBUG_SEND

	printf(">> %#x l: %d [",canid,len);

	for(int i=0; i<len; i++) {
		printf("%#x ",data[i]);
	}
	printf("]\n");

#endif
	if (write(socketfd, &tx_frame, sizeof(tx_frame)) != sizeof(tx_frame)) {
		perror("write");
		writeOk=-1;
		usleep(1000);
		return -1;
	}
	return 1;
};
int UsbCanConnection::SendCanMsg(unsigned short canid,unsigned char* data,int len) {
	if (len<0 || len>8) {
		printf("Data length is out of range in SendCanMsg: %d\n",len);
		return -1;
	}
	/* set frame */
	tx_frame.can_id = canid;
	tx_frame.can_dlc = len;
	memcpy(tx_frame.data,data,len);
	/* send frame */

#if DEBUG_SEND

	printf(">> %#x l: %d [",canid,len);

	for(int i=0; i<len; i++) {
		printf("%#x ",data[i]);
	}
	printf("]\n");

#endif
	if (write(socketfd, &tx_frame, sizeof(tx_frame)) != sizeof(tx_frame)) {
		perror("write");
		writeOk=-1;
		usleep(1000);
//		close(socketfd);
		return -1;
	}
	return 1;
};
int UsbCanConnection::SendCanRtr(unsigned short canid) {
	tx_frame.can_id = canid;
	tx_frame.can_id |= CAN_RTR_FLAG;
	tx_frame.can_dlc = 0;
	#if DEBUG_SEND

	printf(">> %#x RTR\n",canid);

	#endif

	if (write(socketfd, &tx_frame, sizeof(tx_frame)) != sizeof(tx_frame)) {
		perror("write");
		writeOk=-1;
		usleep(1000);
		return -1;
	}
	return 1;
};
int UsbCanConnection::SendExCanRtr(unsigned int canid) {
	tx_frame.can_id = canid;
	tx_frame.can_id |= CAN_RTR_FLAG;
	tx_frame.can_id |= CAN_EFF_FLAG;
	tx_frame.can_dlc = 0;
	#if DEBUG_SEND

	printf(">> %#x RTR\n",canid);

	#endif

	if (write(socketfd, &tx_frame, sizeof(tx_frame)) != sizeof(tx_frame)) {
		perror("write");
		writeOk=-1;
		usleep(1000);
		return -1;
	}
	return 1;
};

int UsbCanConnection::WaitForCanMsg(unsigned int msgid, unsigned char* buffer, int* len, int timeout) {
	waitOnId = msgid;
    int i=0;
    while(waitOnId && i++ < timeout) {
        usleep(10);
    }
    if (waitOnId) {
        waitOnId = 0;
        printf("timeout during wait!\n");
        return 0;
    }
	memcpy(buffer,explicitAnswerBuf+1,explicitAnswerBuf[0]);
    (*len)=explicitAnswerBuf[0];
    return 1;
};


void* UsbCanConnection::_thread(void* This) { return ((UsbCanConnection*)This)->readLoop(); }

