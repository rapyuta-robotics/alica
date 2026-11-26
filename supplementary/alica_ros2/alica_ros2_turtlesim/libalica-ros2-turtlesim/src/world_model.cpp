#include "world_model.hpp"

namespace turtlesim
{
using std::placeholders::_1;
ALICATurtleWorldModel* ALICATurtleWorldModel::instance = nullptr;

ALICATurtleWorldModel* ALICATurtleWorldModel::get()
{
    return instance;
}

void ALICATurtleWorldModel::init()
{
    if (!instance) {
        instance = new ALICATurtleWorldModel();
    }
}

void ALICATurtleWorldModel::del()
{
    delete instance;
    instance = nullptr;
}

ALICATurtleWorldModel::ALICATurtleWorldModel()
{
    _turtle = std::make_unique<ALICATurtle>(this);
}

// Must not call del(): del() deletes the instance, so calling it here recursed until the stack overflowed
ALICATurtleWorldModel::~ALICATurtleWorldModel() = default;

} // namespace turtlesim
