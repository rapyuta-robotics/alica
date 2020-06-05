#include "CounterClass.h"
namespace alica
{
std::atomic<int> CounterClass::called(0);
CounterClass::CounterClass() {}
CounterClass::~CounterClass() {}
} // namespace alica
