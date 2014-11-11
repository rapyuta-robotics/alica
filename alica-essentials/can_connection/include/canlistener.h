#ifndef CANLISTENER_H
#define CANLISTENER_H 1

class CanListener {
	public:
	virtual void Receive(unsigned int canid,unsigned char* data, int len) =0;
};

#endif
