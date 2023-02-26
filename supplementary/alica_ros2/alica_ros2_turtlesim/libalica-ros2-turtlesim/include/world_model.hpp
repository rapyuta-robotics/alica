#pragma once

#include "turtle.hpp"
#include <engine/AlicaEngine.h>
#include <memory>

namespace turtlesim
{
/*
        ALICATurtleWorldModel
        - WorldModel for ALICA ros turtlesim which interface between ALICA and ROS.
        - A few class is static since one robot has one world model
        - ROS:
                - Subscribe: t/init
*/
class ALICATurtleWorldModel
{
public:
    static ALICATurtleWorldModel* get(); // return instance
    static void init();                  // create instance
    static void del();
    bool getInit() const { return _initTrigger; };
    void setInit(const bool input) { _initTrigger = input; };

    static ALICATurtleWorldModel* instance;
    std::unique_ptr<ALICATurtle> turtle;

    bool _initTrigger; // become true when /init topic published

private:
    ALICATurtleWorldModel();
    ~ALICATurtleWorldModel();
};

} // namespace turtlesim
