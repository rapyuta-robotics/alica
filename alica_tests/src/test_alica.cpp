#include "test_alica.h"

std::jmp_buf restore_point;

void signalHandler(int signal)
{
    std::longjmp(restore_point, signal);
}