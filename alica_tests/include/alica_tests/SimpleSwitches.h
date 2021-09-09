#pragma once
#include <vector>

class SimpleSwitches
{
public:
    static void reset() { _switches.clear(); }
    static bool isSet(int i) { return i < static_cast<int>(_switches.size()) ? _switches[i] : false; }
    static void set(int i, bool value)
    {
        if (i >= static_cast<int>(_switches.size())) {
            _switches.resize(i + 1, false);
        }
        _switches[i] = value;
    }

private:
    static std::vector<bool> _switches;
};