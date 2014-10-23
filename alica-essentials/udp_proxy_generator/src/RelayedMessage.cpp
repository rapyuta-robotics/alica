#include "RelayedMessage.h"

RelayedMessage::RelayedMessage(string topic, string message, string options)
{
	this->Ros2UdpQueueLength = 5;
	this->Udp2RosQueueLength = 5;
	this->Topic = topic;
	this->OptionsString = options;
	//this->Id = (uint)topic.GetHashCode();

	int lastSlash = message.find_last_of('/');
	if (lastSlash == string::npos)
	{
		//throw new Exception("unqualified Message Name: "+qualifiedName);
		cout << "No Namespace given, assuming std_msgs" << endl;
		this->BaseName = message;
		this->NameSpace = "std_msgs/";
		this->FullName = "std_msgs/" + message;
	}
	else
	{
		this->BaseName = message.substr(lastSlash + 1);
		this->NameSpace = message.substr(0, lastSlash + 1);
		this->FullName = message;
	}

	this->UseRosTcp = (options.find("tcpros") != string::npos);

	boost::regex Ros2UdpQueueRE("Ros2UdpQueueLength\\s*=\\s*([0-9]+)");
	boost::regex Udp2RosQueueRE("Ros2UdpQueueLength\\s*=\\s*([0-9]+)");

	if (boost::regex_match(options, Ros2UdpQueueRE))
	{
		boost::smatch m;
		boost::regex_search(options, m, Ros2UdpQueueRE);
		this->Ros2UdpQueueLength = std::stoi(m[1]);
	}
	if (boost::regex_match(options, Udp2RosQueueRE))
	{
		boost::smatch m;
		boost::regex_search(options, m, Udp2RosQueueRE);
		this->Udp2RosQueueLength = std::stoi(m[1]);
	}
}

RelayedMessage::~RelayedMessage()
{
}

string RelayedMessage::getRosMessageHandler()
{
}
//
//using System;
//using System.IO;
//using System.Collections.Generic;
//using System.Diagnostics;
//using System.Text.RegularExpressions;
//namespace UdpCodeGen
//{
//
//
//	public class RelayedMessage
//	{
//		//static ulong idcounter = 1;
//		public string Topic {get; private set;}
//		public string FullName {get; private set;}
//		public string BaseName {get; private set;}
//		public string NameSpace {get; private set;}
//		public string OptionsString {get; private set;}
//
//		public int Ros2UdpQueueLength {get; set;}
//		public int Udp2RosQueueLength {get; set;}
//
//		public uint Id {get; private set;}
//
//		public string RosCallBackName{get { return "onRos"+this.BaseName+this.Id;}}
//		public string RosClassName {
//			get {
//				return this.FullName.Replace("/","::");
//			}
//		}
//		public bool UseRosTcp {get; private set;}
//		private static Regex Ros2UdpQueueRE = new Regex(@"Ros2UdpQueueLength\s*=\s*([0-9]+)");
//		private static Regex Udp2RosQueueRE = new Regex(@"Ros2UdpQueueLength\s*=\s*([0-9]+)");
//
//		public RelayedMessage (string topic, string message, string options){
//			this.Ros2UdpQueueLength = 5;
//			this.Udp2RosQueueLength = 5;
//			this.Topic = topic;
//			this.OptionsString = options;
//			this.Id = (uint)topic.GetHashCode();
//
//			int lastSlash = message.LastIndexOf('/');
//			if (lastSlash < 0) {
//				//throw new Exception("unqualified Message Name: "+qualifiedName);
//				Console.WriteLine("No Namespace given, assuming std_msgs");
//				this.BaseName = message;
//				this.NameSpace = "std_msgs/";
//				this.FullName = "std_msgs/"+message;
//			} else {
//				this.BaseName = message.Substring(lastSlash+1);
//				this.NameSpace = message.Substring(0,lastSlash+1);
//				this.FullName = message;
//			}
//
//			this.UseRosTcp = (options.IndexOf("tcpros")>=0);
//
//			if(Ros2UdpQueueRE.IsMatch(options)) {
//				Match m = Udp2RosQueueRE.Match(options);
//				this.Ros2UdpQueueLength = Int32.Parse(m.Groups[1].Value);
//			}
//			if(Udp2RosQueueRE.IsMatch(options)) {
//				Match m = Udp2RosQueueRE.Match(options);
//				this.Udp2RosQueueLength = Int32.Parse(m.Groups[1].Value);
//			}
//
//			//Console.WriteLine("Message: {0} {1} {2}",topic,message,options);
//		}
//		public string PublisherName {get {return "pub"+this.Id;}}
//
//		public string GetRosMessageHandler() {
//
//			string ret = "void "+RosCallBackName+"(const ros::MessageEvent<"+this.RosClassName+">& event) {\n";
//			//ret += "std::cout << event.getPublisherName() << \" \" << ownRosName << std::endl;\n";
//			ret += "\tif(0 == event.getPublisherName().compare(ownRosName)) return;\n";
//			ret += "uint8_t* buffer = NULL;\n";
//			//ret += "\tstd::cout << \"Receiving ros msg: "+this.FullName+"\" << std::endl;\n";
//			ret += "\tconst "+this.RosClassName+"::ConstPtr& message = event.getMessage();\n";
//			ret +="\ttry{\n";
//			ret += "\t\tuint32_t serial_size = ros::serialization::serializationLength(*message);\n";
//
//			ret += "\t\tbuffer = new uint8_t[serial_size+4];\n";
//
//
//			ret += "\t\tros::serialization::OStream stream(buffer+4, serial_size);\n";
//
//			ret += "\t\t*((uint32_t*)buffer) = "+this.Id+"u;\n";
//
//			ret += "\t\tros::serialization::serialize(stream, *message);\n";
//
//
//
//			//ret += "\t\tros::serialization::OStream stream((uint8_t*)ddsmsg.data.get_buffer(), serial_size);\n";
//			//ret += "\t\tros::serialization::serialize(stream, *message);\n";
//			ret +="\t\t// write message to UDP\n";
//			ret +="\t\tinsocket->send_to(boost::asio::buffer((void*)buffer,serial_size+4),destEndPoint);\n";
//			ret +="\t} catch(std::exception& e) {\n";
//			ret +="\t\tROS_ERROR_STREAM_THROTTLE(2,\"Exception while sending UDP message:\"<<e.what()<< \" Discarding message!\");\n";
//
//			ret +="\t}\n";
//			ret +="\tif(buffer!=NULL) delete[] buffer;\n";
//			ret += "}\n";
//			return ret;
//		}
//
//	}
//}
