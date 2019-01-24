#include <test_alica.h>
#include <engine/AlicaClock.h>
#include <engine/AlicaEngine.h>
#include <engine/PlanBase.h>

std::jmp_buf restore_point;

void signalHandler(int signal)
{
    std::longjmp(restore_point, signal);
}

void step(alica::AlicaEngine* ae)
{
    ae->stepNotify();
    do {
        ae->getAlicaClock()->sleep(alica::AlicaTime::milliseconds(33));
    } while (!ae->getPlanBase()->isWaiting());
}