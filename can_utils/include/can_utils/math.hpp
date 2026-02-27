#ifndef CAN_UTILS_MATH_HPP
#define CAN_UTILS_MATH_HPP

#include <cmath>
#include <iostream>
#include <functional>
#include <memory>

namespace net::wavem::can
{
    class Math final
    {
    public:
        explicit Math();
        virtual ~Math();
        using SharedPtr = std::shared_ptr<Math>;

    public:
        [[nodiscard]] bool is_big_endian() const;
        [[nodiscard]] uint16_t to_big_endian(const uint16_t &value) const;
        double convert_linear_to_kmh(const double &linear_x, double correction = 1) const;
        double convert_kmh_to_linear(const double &kmh, double correction = 1) const;

        template <typename T>
        T apply_dump_factor_offset(const auto &raw_value, const double &factor, const double &offset) const
        {
            return static_cast<T>(raw_value == 0 ? 0 : raw_value * factor + offset);
        }

        template <typename T>
        T apply_send_factor_offset(const T &physical_value, const double &factor, const double &offset) const
        {
            return static_cast<T>(physical_value == 0 ? 0 : (physical_value - offset) / factor);
        }
    };
}

#endif