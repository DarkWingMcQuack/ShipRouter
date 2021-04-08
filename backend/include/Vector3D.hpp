#pragma once

#include <Constants.hpp>
#include <LatLng.hpp>
#include <Utils.hpp>
#include <cmath>
#include <fmt/core.h>
#include <tuple>

class Vector3D
{
public:
    Vector3D(Latitude<Radian> lat, Longitude<Radian> lng) noexcept
        : x_(std::cos(lat.getValue()) * std::cos(lng.getValue())),
          y_(std::cos(lat.getValue()) * std::sin(lng.getValue())),
          z_(std::sin(lat.getValue())) {}

    Vector3D() = default;
    Vector3D(const Vector3D&) = default;
    Vector3D(Vector3D&&) = default;
    auto operator=(const Vector3D&) -> Vector3D& = default;
    auto operator=(Vector3D&&) -> Vector3D& = default;

    constexpr Vector3D(double x, double y, double z) noexcept
        : x_(x), y_(y), z_(z) {}

    auto normalize() const noexcept
        -> Vector3D
    {
        const auto l = length();

        const auto x = x_ / l;
        const auto y = y_ / l;
        const auto z = z_ / l;

        return Vector3D{x, y, z};
    }

    auto toLatLng() const noexcept
        -> std::pair<Latitude<Radian>, Longitude<Radian>>
    {
        auto lat = std::atan2(z_, std::sqrt(x_ * x_ + y_ * y_));
        auto lng = std::atan2(y_, x_);

        return std::pair{Latitude<Radian>{lat},
                         Longitude<Radian>{lng}};
    }

    constexpr auto dotProduct(const Vector3D& other) const noexcept
        -> double
    {
        return x_ * other.x_ + y_ * other.y_ + z_ * other.z_;
    }

    constexpr auto crossProduct(const Vector3D& other) const noexcept
        -> Vector3D
    {
        return Vector3D{y_ * other.z_ - z_ * other.y_,
                        z_ * other.x_ - x_ * other.z_,
                        x_ * other.y_ - y_ * other.x_};
    }

    auto length() const noexcept
        -> double
    {
        return std::sqrt(x_ * x_ + y_ * y_ + z_ * z_);
    }

    constexpr auto operator-(const Vector3D& other) const noexcept
        -> Vector3D
    {
        return Vector3D{
            x_ - other.x_,
            y_ - other.y_,
            z_ - other.z_};
    }

    auto distanceTo(const Vector3D& other) const noexcept
        -> double
    {
        const auto a = std::atan2(crossProduct(other).length(), dotProduct(other));
        return EARTH_RADIUS_IN_METERS * a;
    }

    auto angleBetween(const Vector3D& other,
                      const Vector3D& plain_normal) const noexcept
        -> double
    {
        const auto sign = crossProduct(other).dotProduct(plain_normal) >= 0 ? 1.0 : -1.0;
        const auto sin_theta = crossProduct(other).length() * sign;
        const auto cos_theta = dotProduct(other);

        return std::atan2(sin_theta, cos_theta);
    }


    auto toString() const noexcept
        -> std::string
    {
        return fmt::format("x:{}, y:{}, z:{}", x_, y_, z_);
    }

private:
    double x_;
    double y_;
    double z_;
};


template<class Tag>
auto distanceBetween(Latitude<Tag> lat_start,
                     Longitude<Tag> lng_start,
                     Latitude<Tag> lat_dest,
                     Longitude<Tag> lng_dest) noexcept
    -> std::enable_if_t<is_tag<Tag>, double>
{
    const auto [first_vec, second_vec] = [&]() constexpr
    {
        if constexpr(std::is_same_v<Tag, Degree>) {
            return std::pair{Vector3D{lat_start.toRadian(),
                                      lng_start.toRadian()},
                             Vector3D{lat_dest.toRadian(),
                                      lng_dest.toRadian()}};
        } else {
            return std::pair{Vector3D{lat_start,
                                      lng_start},
                             Vector3D{lat_dest,
                                      lng_dest}};
        }
    }
    ();

    return first_vec.distanceTo(second_vec);
}
