#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <vector>
#include <boost/regex.hpp>
#include <fstream>

#include "boost/filesystem.hpp"
using namespace boost::filesystem;

#include "RelayedMessage.h"

using namespace std;

std::string exec(const char* cmd)
{
	FILE* pipe = popen(cmd, "r");
	if (!pipe)
		return "ERROR";
	char buffer[128];
	std::string result = "";
	while (!feof(pipe))
	{
		if (fgets(buffer, 128, pipe) != NULL)
			result += buffer;
	}
	pclose(pipe);
	return result;
}

string getTemplateDir()
{
	string pwd = exec((string("rospack find udp_proxy_generator")).c_str());
	pwd.pop_back();
	return pwd + "/templates";
}

bool parseDefinitionFile(string msgDefFile, vector<RelayedMessage*>& msgList)
{
	//regex line("Topic:\\^ ");
	//regex line("Topic:\\.*(\\.+)\\.*Msg:\\.*(\\.+)\\.*Opt:\\.*\\[(.*)\\]");
	string regstr = "Topic:\\s*(\\S+)\\s*Msg:\\s*(\\S+)\\s*Opt:\\s*\\[(.*)\\]";
	boost::regex line(regstr);
	//regex line("Topic:\s*(^ )\s*Msg:\s*(^ )\s*Opt:\s*\[(.*)\]");
	ifstream ifs(msgDefFile);

	int i = 1;
	while (!ifs.eof())
	{
		string s;
		std::getline(ifs, s);
		if ((s.length() > 0 && s[0] == '#') || s.length() < 1 )
		{
			i++;
			continue;
		}
		if (boost::regex_match(s, line))
		{
			boost::smatch m;
			string topic, message, options;
			if (boost::regex_search(s, m, line))
			{
				topic = m[1];
				message = m[2];
				options = m[3];
				//for (int n=1; n<m.size(); n++)
				//	std::cout << m[n] << " " << n << " ";
				//std::cout << std::endl;
				//s = m.suffix().str();
			}

			//Match m = line.Match(s);
			RelayedMessage* msg = new RelayedMessage(topic, message, options);
			msgList.push_back(msg);
		}
		else
		{
			cout << "Parse Error in line " << i << " of " << msgDefFile << endl;
			cout << ">" << s << "<" << endl;
			return false;
		}
		i++;
	}
	return true;
}

int main(int argc, char *argv[])
{

	if (argc < 2)
	{
		cout << "Usage MakeUDPProxy.exe <packageName>" << endl;
		return -1;
	}

	string outputPath;
	outputPath = exec((string("rospack find ") + argv[1]).c_str());
	cout << outputPath << endl;
	outputPath.pop_back();
	string templateDir = getTemplateDir();
	if (!exists(templateDir))
	{
		cout << "Cannot find template directory: " << templateDir << endl;
	}

	/*string basePackageName=argv[1];
	 if(basePackageName.find('/')!=string::npos) {
	 int idx = basePackageName.find_last_of('/');
	 basePackageName = basePackageName.substr(idx+1);
	 }*/

	if (!exists(outputPath))
	{
		cout << "Cannot find package name!" << endl;
		return -1;
	}
	string msgDefFile = outputPath + "/relayMsgs.conf";
	outputPath += "/proxy_gen";
	//set namespace to packageName
	//namespaceS = args[0];

//	DateTime msgDefTime = File.GetLastWriteTime (msgDefFile);

	if (!exists(msgDefFile))
	{
		cout << "Cannot find definition file " << msgDefFile << endl;
		return -1;
	}
	vector<RelayedMessage*> msgList;
	if (!parseDefinitionFile(msgDefFile, msgList))
	{
		return -1;
	}
//	if (!CheckForCollisions())
//	{
//		return -1;
//	}
//
//	bool reGenerate = true;
//	if (Directory.Exists(outputPath))
//	{
//		//Console.WriteLine("dir exists: " + outputPath);
//		string[] tFiles = Directory.GetFiles(outputPath);
//
//		reGenerate = false;
//		foreach(string f in tFiles)
//		{
//			DateTime fTime = File.GetCreationTime(f);
//			//Console.WriteLine("fileToTest:" + f);
//			//Console.WriteLine("fTime: " + fTime);
//			if(fTime.CompareTo(msgDefTime) < 0)
//			{
//				reGenerate = true;
//				break;
//			}
//		}
//
//		if(reGenerate)
//		Directory.Delete(outputPath,true);
//	}
//	if(reGenerate)
//	{
//		Directory.CreateDirectory(outputPath);
//		ProcessTemplates(templateDir,outputPath);
//	}
//	return 0;
	for(auto d : msgList) {
		delete d;
	}
	cout << "Done" << endl;
	return 0;
}

/*using System;
 using System.IO;
 using System.Collections.Generic;
 using System.Diagnostics;
 using System.Text.RegularExpressions;
 namespace UdpCodeGen
 {



 public class UDPGen
 {
 static string outputPath;
 static string msgDefFile;
 static string basePackageName;
 static List<RelayedMessage> msgList;

 public static void ProcessTemplates(string tmplDir,string outDir) {
 string[] tmplarr = Directory.GetFiles(tmplDir,"*.*");
 foreach(string tmpl in tmplarr) {
 Console.WriteLine("Template: {0}",tmpl);
 int idx = tmpl.LastIndexOf('/');
 string basename = tmpl.Substring(idx+1);
 StreamReader read = new StreamReader(tmpl);
 string content = read.ReadToEnd();
 string parsedContent = ProcessTemplate(content);
 StreamWriter file = new StreamWriter(outDir + "/"+basename);
 file.WriteLine(parsedContent);
 file.Close();
 }
 }


 public static bool CheckForCollisions() {
 for(int i=0; i<msgList.Count; i++) {
 for(int j=i+1; j<msgList.Count;j++) {
 if (msgList[i].Id == msgList[j].Id) {
 Console.Error.WriteLine("Hashcollision between topic {0} and topic {1}!",msgList[i].Topic,msgList[j].Topic);
 return false;
 }
 }
 }
 return true;
 }
 public static string ProcessTemplate(string t) {

 Regex markers = new Regex(@"<\?(.*)\?>");
 return markers.Replace(t,new  MatchEvaluator(MatchReplacer));
 }
 public static string MatchReplacer(Match ma) {
 string s = ma.ToString();
 s = s.Substring(2,s.Length-4).Trim();
 string ret = "";
 switch(s) {
 case "messageIncludes":
 foreach(RelayedMessage m in msgList) {
 ret+= "#include \""+m.FullName+".h\"\n";
 }
 return ret;
 case "subscriptions":
 int i=0;
 foreach(RelayedMessage m in msgList) {
 if(m.UseRosTcp) {
 ret +="ros::Subscriber sub"+i+" = n.subscribe(\""+m.Topic+"\","+m.Ros2UdpQueueLength+", "+m.RosCallBackName+");\n";
 } else {
 ret +="ros::Subscriber sub"+i+" = n.subscribe(\""+m.Topic+"\","+m.Ros2UdpQueueLength+", "+m.RosCallBackName+",ros::TransportHints().unreliable().tcpNoDelay().reliable());\n";
 }
 i++;
 }
 return ret;
 case "rosMessageHandler":
 foreach(RelayedMessage m in msgList) {
 ret += m.GetRosMessageHandler();

 }
 return ret;


 case "advertisement":
 foreach(RelayedMessage m in msgList) {
 ret+=m.PublisherName+" = n.advertise<"+m.RosClassName+">(\""+m.Topic+"\","+m.Udp2RosQueueLength+",false);\n";
 }
 return ret;
 case "rosPublisherDecl":
 foreach(RelayedMessage m in msgList) {
 ret+="ros::Publisher "+m.PublisherName+";\n";
 }
 return ret;
 case "packageName":
 return basePackageName;
 case "udpReception":
 foreach(RelayedMessage m in msgList) {
 ret+="case "+m.Id+"ul: {\n";
 ret+=m.RosClassName +" m"+m.Id+";\n";
 ret+="ros::serialization::Serializer<"+m.RosClassName+">::read(stream, m"+m.Id+");\n";
 ret+=m.PublisherName+".publish<"+m.RosClassName+">(m"+m.Id+");\n";
 //ret+="std::cout << m"+m.Id+" << std::endl;\n";
 ret+="break; }\n";
 }
 return ret;

 default: Console.Error.WriteLine("Unknown Marker: {0}",s);
 System.Environment.Exit(-1);
 break;
 }
 return ret;
 }



 }
 }*/
