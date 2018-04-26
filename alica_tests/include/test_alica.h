#pragma once

#include <csignal>
#include <csetjmp>

#define ASSERT_NO_SIGNAL ASSERT_EQ(setjmp(restore_point), 0);

namespace alica {
class AlicaEngine;
}

extern std::jmp_buf restore_point;
void signalHandler(int signal);
void step(alica::AlicaEngine* ae);