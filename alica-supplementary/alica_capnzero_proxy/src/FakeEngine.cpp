#include <communication/AlicaCapnzeroCommunication.h>
#include <engine/AlicaEngine.h>

#include <capnp/common.h>
#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include <kj/array.h>


int main(int argc, char* argv[])
{
    alica::AlicaEngine *ae = NULL;
    alicaCapnzeroProxy::AlicaCapnzeroCommunication *com = new alicaCapnzeroProxy::AlicaCapnzeroCommunication(ae);
}