/*
 * TestBase.cpp
 *
 *  Created on: Mar 27, 2014
 *      Author: Stephan Opfer
 */

#include "TestBase.h"

using namespace tinyxml2;
using namespace std;

namespace alica {

TestBase::TestBase(int argc, char** argv) {

	cout << "+++ Consuming arguments with get opt:" << endl;
	ConsumeCmdLineArguments(argc, argv);
	cout << endl;

	this->ae = AlicaEngine::getInstance();
	this->ae->init(this->roleset, this->masterPlan, this->rolesetdir, this->stepEngine);
	this->ae->start();
}

void TestBase::ConsumeCmdLineArguments(int argc, char** argv) {
	// consume command line arguments with "getopt" from linux kernel
	int option_index = 0;
	static struct option long_options[] = { { "masterplan", required_argument,
			NULL, 'm' }, { "m", required_argument, NULL, 'm' }, { "rolesetdir",
			required_argument, NULL, 'd' }, { "rd", required_argument,
	NULL, 'd' }, { "roleset", required_argument, NULL, 'r' }, { "r",
			required_argument, NULL, 'r' }, { "teamColour", required_argument,
			NULL, 'c' }, { "tc", required_argument,
	NULL, 'c' }, { "delayStart", required_argument, NULL, 't' }, { "ds",
			required_argument,
			NULL, 't' }, { "simulator", no_argument, NULL, 's' }, { "s",
			no_argument, NULL, 's' }, { "stepEngine", no_argument, NULL, 'e' },
			{ "se", no_argument, NULL, 'e' }, { 0, 0, NULL, 0 } };

	// set all known and given cmd-line params
	while (1) {
		int opt = getopt_long(argc, argv, "", long_options, &option_index);
		if (opt == -1) {
			cout << "All known arguments parsed!" << endl;
			break;
		}
		switch (opt) {
		case 'm': // masterplan
			printf("Option masterplan/m is set to '%s'.\n", optarg);
			this->masterPlan = optarg;
			break;
		case 's': // simulator
			printf("Option simulator/s is set.\n");
			this->simulator = true;
			break;
		case 'd': // rolesetdir
			printf("Option rolesetdir/rd is set to '%s'.\n", optarg);
			this->rolesetdir = optarg;
			break;
		case 'r': // roleset
			printf("Option roleset/r is set to '%s'.\n", optarg);
			this->roleset = optarg;
			break;
		case 'c': // teamColour
			printf("Option teamColour/tc is set to '%s'.\n", optarg);
			this->teamColour = optarg;
			break;
		case 't': // delayStart
			printf("Option delayStart/ds is set to '%s'.\n", optarg);
			this->delayStart = optarg;
			break;
		case 'e': // stepEngine
			printf("Option stepEngine/se is set.\n");
			this->stepEngine = true;
			break;
		case '?':
			printf("Unknown option is set to '%s'.\n", optarg);
			break;
		default:
			printf(
					"getopt returned character code '0%o', which is the char '%c'.\n",
					opt, opt);
		}
	}

	// check whether all required cnd-line params were set
	if (this->masterPlan.empty()) {
		cerr
				<< "ERROR: The required command line parameter 'masterplan' is missing."
				<< endl;
		PrintUsage();
	}

	if (optind < argc) {
		printf("Unknown or invalid formatted options: ");
		while (optind < argc) {
			printf("'%s' ", argv[optind++]);
		}
		printf("\n");
	}

}

void TestBase::PrintUsage() {
	cout << "Usage:" << endl;
	cout << endl;
	cout << "Required parameters (ALICA):" << endl;
	cout
			<< "\t--masterplan=<plan-name>\t Defines the ALICA top-level plan, which should be executed."
			<< endl;
	cout << endl;
	cout << "Optional parameters (ALICA):" << endl;
	cout
			<< "\t--rolesetdir=<rolesetdir>\t Defines the directory, in which the roleset file should be located."
			<< endl;
	cout
			<< "\t--roleset=<roleset>\t\t Defines the file, which contains the roleset."
			<< endl;
	cout
			<< "\t--delayStart=<milliseconds>\t Defines the time span between initialising everything and starting the ALICA engine."
			<< endl;
	cout
			<< "\t--stepEngine\t\t\t Makes the control-loop of the ALICA engine triggered by the user instead of a timer."
			<< endl;
	cout << endl;
	cout << "Optional parameters (MSL):" << endl;
	cout
			<< "\t--teamColour=[cyan|magenta]\t Defines the team colour of your team."
			<< endl;
	cout
			<< "\t--simulator\t\t\t Makes the control-loop of the ALICA engine triggered by the user instead of a timer."
			<< endl;
}

TestBase::~TestBase() {
	// TODO Auto-generated destructor stub
}

void TestBase::Run() {

	this->ae->start();

	while (ros::ok()) {
		ros::Duration(0.5).sleep();
	}
}

} /* namespace Alica */

int main(int argc, char** argv) {

	ros::init(argc, argv, "test_base");
	ros::NodeHandle n;
	alica::TestBase *tb = new alica::TestBase(argc, argv);
	tb->Run();

	/*
	 XMLDocument doc;
	 doc.LoadFile("WM09.pml");
	 printf("doc.ErrorCode: %d\n", doc.ErrorID());
	 // Structure of the XML file:
	 // - Element "PLAY"      the root Element, which is the
	 //                       FirstChildElement of the Document
	 // - - Element "TITLE"   child of the root PLAY Element
	 // - - - Text            child of the TITLE Element

	 // Navigate to the title, using the convenience function,
	 // with a dangerous lack of error checking.
	 const char* title = doc.FirstChildElement("alica:Plan")->FirstChildElement(
	 "states")->FirstChildElement("plans")->GetText();
	 printf("Text (1): %s\n", title);

	 // Text is just another Node to TinyXML-2. The more
	 // general way to get to the XMLText:
	 XMLText* textNode = doc.FirstChildElement("alica:Plan")->FirstChildElement(
	 "states")->FirstChildElement("plans")->FirstChild()->ToText();
	 title = textNode->Value();
	 printf("Text (2): %s\n", title);

	 return 0;
	 */
}
