#include "engine/model/Quantifier.h"
#include "engine/model/EntryPoint.h"
#include "engine/model/Plan.h"
#include "engine/model/State.h"
#include <assert.h>
namespace alica
{

Quantifier::Quantifier()
        : _scope(nullptr)
        , _scopeType(PLANSCOPE)
{
}

Quantifier::~Quantifier() {}

/**
 * Set the scope of this quantifier
 * @param ae An AlicaElement
 */
void Quantifier::setScope(const AlicaElement* element)
{
    _scope = element;
    if (dynamic_cast<const EntryPoint*>(element) != nullptr) {
        _scopeType = ENTRYPOINTSCOPE;
    } else if (dynamic_cast<const Plan*>(element) != nullptr) {
        _scopeType = PLANSCOPE;
    } else if (dynamic_cast<const State*>(element) != nullptr) {
        _scopeType = STATESCOPE;
    } else {
        assert(false);
    }
}

void Quantifier::setDomainIdentifiers(const std::vector<std::string>& domainIdentifiers)
{
    _domainIdentifiers = domainIdentifiers;
}

} // namespace alica
