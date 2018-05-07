#pragma once

namespace alica {
typedef signed long long AlicaTime;

class IAlicaClock {
public:
    virtual ~IAlicaClock() {}
    virtual AlicaTime now() = 0;
    virtual void sleep(long us) = 0;
};

}  // namespace alica
