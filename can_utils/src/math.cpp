#include "math.hpp"

using namespace net::wavem::can;

Math::Math() = default;
Math::~Math() = default;

bool Math::is_big_endian() const
{
    char buf[2] = {0, 1};
    unsigned short *val = reinterpret_cast<unsigned short *>(buf);
    return *val == 1;
}

uint16_t Math::to_big_endian(const uint16_t &value) const
{
    return (value >> 8) | (value << 8);
}

double Math::convert_linear_to_kmh(const double &linear_x, double correction) const
{
    return linear_x * 3.6 * correction;
}

double Math::convert_kmh_to_linear(const double &kmh, double correction) const
{
    return kmh / 3.6 / correction;
}