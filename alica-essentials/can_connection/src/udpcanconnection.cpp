#include "udpcanconnection.h"

#define DEBUG_SEND 0
//#define DEBUG_RECV 1


UdpCanConnection::UdpCanConnection(const char* IP,const int port)  {
			serverAddrCast  = (struct sockaddr *) &serverAddr;
			clientAddrCast  = (struct sockaddr *) &clientAddr;
			SetTargetIP(IP);
			SetTargetPort(port);
			hp=NULL;
			socketfd = -1;
			listener = NULL;
			waitOnId = 0;
			stopThread = 0;
}
void UdpCanConnection::SetLocalIP(const char* IP) {
	hp = gethostbyname(IP);
	memcpy((char*)&serverAddr.sin_addr, (char*)hp->h_addr, hp->h_length);
}

void UdpCanConnection::SetTargetIP(const char* IP) {
	/*if (hp!=NULL) {
		free(hp);
	}*/
	//printf("Here %s\n",IP);
	hp = gethostbyname(IP);
	memcpy((char*)&clientAddr.sin_addr, (char*)hp->h_addr, hp->h_length);
}
void UdpCanConnection::SetTargetPort(const int port) {
	this->port = port;
	serverAddr.sin_port = htons(port);
	clientAddr.sin_port = htons(port);
}
void UdpCanConnection::SetReceiver(CanListener* cl) {
	listener = cl;
}
int UdpCanConnection::Start() {
	if (socketfd > 0) return -1;

	socketfd = socket(AF_INET, SOCK_DGRAM, 0);
	if (socketfd < 0) return -1;

	serverAddr.sin_family = AF_INET;

	//serverAddr.sin_addr.s_addr = INADDR_ANY;

	clientAddr.sin_family = AF_INET;

	bind(socketfd, serverAddrCast, sizeof(serverAddr));


	int ret = pthread_create(&listenerThread,NULL,(void* (*)(void*))&UdpCanConnection::_thread,this);
	if (ret!=0) {
		printf("Thread creation failed\n");
		return -1;
	}
	printf("Started Listening\n");


	return 1;
}
int UdpCanConnection::SendCanRtr(unsigned short canid) {
		unsigned char* ptr = txbuf;
		*ptr++ = 0x0;
		*ptr++ = 0x0;
		*ptr++ = (unsigned char)(canid >> 8) & 0x7;
		*ptr++ = (unsigned char)canid & 0xFF;
		*ptr++ = 0x01;
#if DEBUG_SEND

	printf(">> %#x RTR\n",canid);

#endif

		return sendto(socketfd,txbuf,5,0, clientAddrCast, sizeof(clientAddr));

}
int UdpCanConnection::SendCanMsg(unsigned short canid,unsigned char* data,int len) {
		if (len<0 || len>8) {
			printf("Data length is out of range in SendCanMsg: %d\n",len);
			return -1;
		}
		unsigned char* ptr = txbuf;
		*ptr++ = 0x0;
		*ptr++ = 0x0;
		*ptr++ = (unsigned char)(canid >> 8) & 0x7;
		*ptr++ = (unsigned char)canid & 0xFF;
		*ptr++ = (len << 1) & 0xFE;
		memcpy(ptr,data,len);
#if DEBUG_SEND

	printf(">> %#x l: %d [",canid,len);

	for(int i=0; i<len; i++) {
		printf("%#x ",data[i]);
	}
	printf("]\n");

#endif

		return sendto(socketfd, txbuf, len+5, 0, clientAddrCast, sizeof(clientAddr));
}

void* UdpCanConnection::readLoop() {
	int result;
	unsigned short canid;
	int dlen;
	while(!stopThread) {
		if (socketfd > 0) {
			result = recv(socketfd,rxbuf,sizeof(rxbuf),0);
			if (result > 0) {
				if(result < 5) {
				printf("Received partial or incomplete packet!\n");

				} else {
					canid =  (((unsigned short)rxbuf[2])<<8) | rxbuf[3];
					dlen  = rxbuf[4] >> 1;
#if DEBUG_RECV

	printf("<< %#x l: %d [",canid,dlen);

	for(int i=0; i<dlen; i++) {
		printf("%#x ",rxbuf[i+5]);
	}
	printf("]\n");

#endif
					if(waitOnId && canid == waitOnId) {
						explicitAnswerBuf[0] = dlen;
						memcpy(explicitAnswerBuf+1,rxbuf+5,dlen);
						waitOnId = 0;
					}
					else {
						listener->Receive(canid,(rxbuf+5),dlen);
					}
				}
			}
		} else {
			usleep(500000);
		}
	}
	return NULL;
}
int UdpCanConnection::WaitForCanMsg(unsigned int msgid, unsigned char* buffer, int* len, int timeout) {
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
}

int UdpCanConnection::Stop() {
	if (socketfd < 0) return -1;
    stopThread = 1;
	return shutdown(socketfd, 2);

}
void* UdpCanConnection::_thread(void* This) { return ((UdpCanConnection*)This)->readLoop(); }
