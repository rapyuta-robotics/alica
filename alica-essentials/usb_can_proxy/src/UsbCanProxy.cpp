
#include "ros/ros.h"
#include "usb_can_proxy/CanMsg.h"
#include "msl_actuator_msgs/RawOdometryInfo.h"
#include <stdio.h>
#include <signal.h>
#include <sys/time.h>
#include <vector>

#include <Can.h>
#include <usbcanconnection.h>

#define TIMEDIFFMS(n,o) (((n).tv_sec-(o).tv_sec)*1000+((n).tv_usec-(o).tv_usec)/1000)

#define 	CMD_BUNDLE			0x07

using namespace std;

class UsbCanProxy : public CanListener
{

public:
	void getCanMsg(const usb_can_proxy::CanMsg::ConstPtr& msg)
	{
		printf("write data to can!\n");
		
		lastCan = ros::Time::now();
		unsigned char* p = (unsigned char*)msg->data.data();
		//if( msg->data[0] == 9 ) {
		//printf("cmd to usb:\n");
		//for(unsigned int i = 0; i<msg->data.size(); i++)
		//{
		//	printf("%x\n",msg->data[i]);
		//}
		us->SendExCanMsg(msg->id,p,msg->data.size());
	}
	void onRawOdometry(const msl_actuator_msgs::RawOdometryInfo::ConstPtr& msg)
	{
		lastMotion = ros::Time::now();
	}
	UsbCanProxy()
	{
		us = new UsbCanConnection("can0");
		us->SetReceiver(this);
		
		//init ros stuff
		compass = n.advertise<CNCanProxyMsg::CanMsg>("usb_can_proxy/Compass", 30);
		rekick = n.advertise<CNCanProxyMsg::CanMsg>("usb_can_proxy/Rekick", 30);
		ballhandler = n.advertise<CNCanProxyMsg::CanMsg>("usb_can_proxy/BallHandler", 30);
		canSub = n.subscribe("usb_can_proxy/CanSub", 30, &UsbCanProxy::getCanMsg,this);//,ros::TransportHints().unreliable().tcpNoDelay());
		odomSub = n.subscribe("RawOdometry", 10, &UsbCanProxy::onRawOdometry, this);		
		
		//spinner = new ros::AsyncSpinner(3);
		//spinner->start();
		
		//add id's for can devices
		receivers.push_back(Compass);
		receivers.push_back(ReKick);
		receivers.push_back(BallHandler);
	}
	
	void Close()
	{
		us->Stop();
	}

	void Start()
	{
		us->Start();
		ROS_INFO("Start Proxy...");
		//ros::spin();
		ros::Rate loop_rate(100);
		while (ros::ok())
		{
// 			ROS_INFO("is running...");
			ros::spinOnce();
			loop_rate.sleep();
			
			ros::Time now = ros::Time::now();
			
			if( (now-lastMotion).toSec() > 1 )
			{
				printf("no motion!\n");
			}
			else if( (now-lastCan).toSec() > 2 )
			{
				//resetInterface();
			}
			
			//usleep(1000);
		}
	}
	
protected:
	//while reading from usb -> publisher
	//ros::AsyncSpinner *spinner;
	ros::NodeHandle n;
	ros::Publisher compass;
	ros::Publisher rekick;
	ros::Publisher ballhandler;
	ros::Subscriber canSub;
	ros::Subscriber odomSub;
	ros::Time lastCan;
	ros::Time lastMotion;
	
	// publisher for direct bundle restart trigger
	ros::Publisher brtPub;
	
	
	UsbCanConnection *us;
	vector<unsigned short> receivers;
	
	void Receive(unsigned int canid,unsigned char* data, int len)
	{
		int found = 0;
		unsigned int id = ((canid & 0xFF00)>>8);
		for(unsigned int i=0; i<receivers.size(); i++) {
			//printf("receiver %u\n",receivers[i]);
			//printf("id is : %u \n",id);
			if( id == receivers[i])
			{
				found=1;
				
				usb_can_proxy::CanMsg cm;
// 				cm.header = 0x0;
// 				cm.priority = (canid>>16) & 0xFF; //Priority
// 				cm.sender = (canid>>8) & 0xFF; //Sender
// 				cm.receiver = (canid)  & 0xFF;  //receiver
// 				cm.length = (len<<1);
				cm.id = id;
				for(int i=0; i<len; i++)
				{
					cm.data.push_back(data[i]);
				}
				
				if( id == Compass )
				{
					compass.publish(cm);
					//printf("get compass val from canbus!\n");
				}
				else if( id == ReKick )
				{
					rekick.publish(cm);
					/*printf("get rekick val from canbus!\n");
					for(unsigned int i=0; i<cm.data.size(); i++)
					{
						printf("%u\n",cm.data[i]);						
					}*/

				}
				else if( id == BallHandler )
				{					
					ballhandler.publish(cm);
					//printf("get actuator val from canbus!\n");
					
				}
				else
				{
					fprintf(stderr,"Unkown canid received: 0x%x\n",canid);
				}
				break;
			}
		}
		if(found <= 0) {
			fprintf(stderr,"Unkown canid received: 0x%x\n",canid);
		}
	}
	
	void resetInterface()
	{
		Close();
		int e = system("sudo ifdown can0");
		sleep(1);
		us = new UsbCanConnection("can0");
		us->SetReceiver(this);
		e = system("sudo ifup can0");
		sleep(1);
		if(!e) printf("error reseting!\n");
		
		lastCan = ros::Time::now();
		lastMotion = ros::Time::now();
		Start();
	}
};
	
UsbCanProxy *ucp;
int main(int argc, char** argv) {
	ros::init(argc, argv, "usb_can_proxy");
	ucp = new UsbCanProxy();
	ucp->Start();
}
