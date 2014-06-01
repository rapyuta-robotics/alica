// Bring in my package's API, which is what I'm testing
#include "engine/AlicaEngine.h"
#include "SystemConfig.h"

// Bring in gtest
#include <gtest/gtest.h>

/**
 * Helpfull method to get the location of the currently executed executable.
 * @return The path to the running executable.
 */
std::string getSelfpath()
{
	char buff[1024];
	ssize_t len = ::readlink("/proc/self/exe", buff, sizeof(buff) - 1);
	if (len != -1)
	{
		buff[len] = '\0';
		return std::string(buff);
	}
	else
	{
		/* handle error condition */
		cerr << "Could not determine my own path!" << endl;
		throw new exception();
	}
}

/**
 * Tests the plan parser with some nice plans
 */
TEST(Alica, planParser)
{
	// determine the path to the test config
	std::string path = getSelfpath();
	int place = path.rfind("devel");
	path = path.substr(0, place);
	path = path + "src/alica/test";

	// bring up the SystemConfig with the corresponding path
	supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
	sc->setRootPath(path);
	sc->setConfigPath(path + "/etc");

	// setup the engine
	alica::AlicaEngine* ae = alica::AlicaEngine::getInstance();
	ae->init("WM09", "WM09", "WM09", false);
	ae->start();
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
