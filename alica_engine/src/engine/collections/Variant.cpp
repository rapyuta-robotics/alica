#include "engine/collections/Variant.h"

namespace alica
{

int variant::serializeTo(uint8_t* arr, const Variant& var)
{
    arr[0] = static_cast<uint8_t>(var.index());
    std::visit( [arr](auto&& arg) {
        using T = std::decay_t<decltype(arg)>;
        if constexpr (!std::is_same_v<std::monostate, T>) {
            memcpy(arr + 1, &arg, kUnionSize);
        }
    }, var );
    return static_cast<int>(kVariantSize);
}

int variant::loadFrom(const uint8_t* arr, Variant& var)
{
    auto type = static_cast<Type>(arr[0]);
    if (type == 1) {
        double data;
        memcpy(&data, arr + 1, kUnionSize);
        var = data;
    }
    else if (type == 2) {
        float data;
        memcpy(&data, arr + 1, kUnionSize);
        var = data;
    }
    else if (type == 3) {
        bool data;
        memcpy(&data, arr + 1, kUnionSize);
        var = data;
    }
    else if (type == 4) {
        void* data;
        memcpy(data, arr + 1, kUnionSize);
        var = data;
    }
    else if (type == 5) {
        int64_t data;
        memcpy(&data, arr + 1, kUnionSize);
        var = data;
    }
    else {
        var.emplace<std::monostate>();
    }
    return static_cast<int>(kVariantSize);
}

} // namespace alica