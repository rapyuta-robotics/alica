
#include "types/Var.h"

#include "types/Clause.h"
#include "types/DecisionLevel.h"
#include "types/Lit.h"
#include "types/Watcher.h"

#include <iostream>

namespace alica
{
namespace reasoner
{
namespace cnsat
{

Var::Var(int index, bool prefSign)
    : _curTerm(nullptr)
    , seen(false)
    , negativeAppearance(0)
    , positiveAppearance(0)
    , negativeRangeSize(0.0)
    , positiveRangeSize(0.0)
    , index(index)
    , assignment(Assignment::UNASSIGNED)
    , locked(false)
    , preferedSign(prefSign)
    , activity(0)
    , positiveRanges(nullptr)
    , negativeRanges(nullptr)
    , watchList(make_shared<vector<Watcher*>>())
{
}

Var::~Var() {}

std::shared_ptr<Clause> Var::getReason() const
{
    return reason;
}

void Var::setReason(std::shared_ptr<Clause> reason)
{
    if (reason && reason != this->reason) {
        for (const std::shared_ptr<Lit>& l : *reason->literals) {
            if (l->var->assignment == Assignment::UNASSIGNED) {
                cout << this->toString() << " " << l->var->toString();
                reason->print();
                cerr << "!!!!!";
                throw "!!!!!";
            }
        }
    }
    this->reason = reason;
}

void Var::reset()
{
    if (locked) {
        return;
    }
    this->reason = nullptr;
    this->seen = false;

    std::cout << "Var::reset() unass" << std::endl;
    this->assignment = Assignment::UNASSIGNED;

    this->activity = 0;
}

void Var::print() const
{
    if (this->assignment == Assignment::FALSE) {
        std::cout << "-";
    } else if (this->assignment == Assignment::UNASSIGNED) {
        std::cout << "o";
    } else {
        std::cout << "+";
    }
    std::cout << this->index;
}

std::string Var::toString() const
{
    std::string str;
    if (this->assignment == Assignment::FALSE)
        str.append("-");
    else if (this->assignment == Assignment::TRUE)
        str.append("+");
    else
        str.append("o");
    str.append(to_string(this->index));
    return str;
}

} /* namespace cnsat */
} /* namespace reasoner */
} /* namespace alica */
