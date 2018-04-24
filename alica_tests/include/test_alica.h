#pragma once

#include <csignal>
#include <csetjmp>

namespace alica {
class AlicaEngine;
}

extern std::jmp_buf restore_point;
void signalHandler(int signal);
void step(alica::AlicaEngine* ae);