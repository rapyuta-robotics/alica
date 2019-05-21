@0xf1c8e694d7d7fbbe;
using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("essentials");

struct ID {
    value @0 :Data;
}