#ifndef UDPCANCONNECTION_H
#define UDPCANCONNECTION_H 1
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <unistd.h>

#include <sys/types.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <pthread.h>

#include <errno.h>

#include <fcntl.h>
#include "canlistener.h"
#include "canconnection.h"

#ifndef CAN_TIMEOUT
#define CAN_TIMEOUT 800
#endif

class UdpCanConnection : public CanConnection {
	public:
		UdpCanConnection(const char* IP,const int port);
		int Start();
		void SetTargetIP(const char* IP);
		void SetTargetPort(const int port);
		void SetLocalIP(const char* IP);
		int Stop();
		void SetReceiver(CanListener* ul);
		//void Send(unsigned char* buf, int len);
		int SendCanMsg(unsigned short canid,unsigned char* data,int len);
		int SendCanRtr(unsigned short canid);
		int WaitForCanMsg(unsigned int msgid, unsigned char* buffer, int* len, int timeout);
	protected:
		void* readLoop();

		CanListener* listener;
		int port;
		int socketfd;
		struct hostent *hp;
		unsigned char rxbuf[128];
		unsigned char txbuf[64];
		struct sockaddr_in serverAddr;
		struct sockaddr_in clientAddr;
		struct sockaddr *serverAddrCast;
		struct sockaddr *clientAddrCast;
		pthread_t listenerThread;
		unsigned short waitOnId;
		unsigned char explicitAnswerBuf[64];
		volatile int stopThread;

		static void* _thread(void* This);
};


#endif
