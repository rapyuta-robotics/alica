#include "TestBehaviour.h"

#include <map>

TestBehaviour::TestBehaviour() {}

extern "C" void Start(char*, char*){};
extern "C" void Stop(char*, char*){};
extern "C" void Configure(const std::map<std::string, std::string>& conf){};
