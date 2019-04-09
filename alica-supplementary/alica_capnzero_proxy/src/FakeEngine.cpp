#include <communication/AlicaCapnzeroCommunication.h>

#include <capnp/common.h>
#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include <kj/array.h>


int main(int argc, char* argv[])
{
    alicaCapnzeroProxy::AlicaCapnzeroCommunication com();
}