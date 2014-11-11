#ifndef CANCONNECTION_H
#define CANCONNECTION_H 1

class CanConnection {


public:	
	virtual int Start() =0;
	virtual int Stop() =0;

	virtual void SetReceiver(CanListener* cl) =0;

	virtual int SendCanMsg(unsigned short canid,unsigned char* data,int len)=0;
	virtual int SendCanRtr(unsigned short canid)=0;
	virtual int WaitForCanMsg(unsigned int msgid, unsigned char* buffer, int* len, int timeout)=0;

	
};


#endif
