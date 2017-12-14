#pragma once

#include "supplementary/IAgentID.h"

#include <iostream>

namespace supplementary
{

class BroadcastID : public supplementary::IAgentID
{
  public:
    BroadcastID(const uint8_t *idBytes, int idSize);
    virtual ~BroadcastID();

    uint8_t *getRaw() const;
    int getSize() const;
    uint8_t getType() const;

    std::string toString() const;
    std::size_t hash() const;
    bool operator==(const supplementary::IAgentID &obj) const;
    bool operator!=(const supplementary::IAgentID &obj) const;
    bool operator<(const supplementary::IAgentID &other) const;
    bool operator>(const supplementary::IAgentID &other) const;

    std::vector<uint8_t> toByteVector() const;

    static const uint8_t TYPE = 1;

  private:
    uint8_t id;
};
} /* namespace supplementary */
