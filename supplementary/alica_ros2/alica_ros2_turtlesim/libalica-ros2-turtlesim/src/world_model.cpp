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
}

ALICATurtleWorldModel::ALICATurtleWorldModel()
{
    turtle = std::make_unique<ALICATurtle>(this);

    // initialize attribute.
    _initTrigger = false;
}

ALICATurtleWorldModel::~ALICATurtleWorldModel() {}

} // namespace turtlesim
