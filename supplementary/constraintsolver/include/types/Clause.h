#pragma once

#include "types/Lit.h"

#include <memory>
#include <vector>

namespace alica
{
namespace reasoner
{
namespace cnsat
{

class Var;
class Watcher;

class Clause
{
  public:
    Clause();
    ~Clause();

    void addChecked(std::shared_ptr<Lit> l);
    std::shared_ptr<Clause> clone();
    void add(std::shared_ptr<Lit> l);
    int avgActivity();
    bool checkSatisfied();

    static bool compareTo(std::shared_ptr<Clause> ep1, std::shared_ptr<Clause> ep2);

    void print();

    bool isTautologic;
    bool isFinished;

    bool satisfied;
    std::shared_ptr<std::vector<Watcher*>> watcher;
    std::shared_ptr<Var> lastModVar;
    int activity;
    std::shared_ptr<std::vector<std::shared_ptr<Lit>>> literals;
};

} // namespace cnsat
/* namespace cnsat */
} /* namespace reasoner */
} /* namespace alica */
