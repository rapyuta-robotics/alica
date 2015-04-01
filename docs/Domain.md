#Autogen code

**--> explanation of code**

``` cpp
#include "DomainBehaviour.h"

namespace alica
{
    DomainBehaviour::DomainBehaviour(string name) :
            BasicBehaviour(name)
    {
	**---> start your world model here**
	**---> you can use it then in all behaviours**
	**---> or implement your motion command in this class, then you use it in all behaviours**
    }

    DomainBehaviour::~DomainBehaviour()
    {
    }
} /* namespace alica */

```
