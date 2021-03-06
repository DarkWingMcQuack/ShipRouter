
#pragma once

#include <LatLng.hpp>
#include <Utils.hpp>
#include <unordered_map>

class OSMNode
{
public:
    OSMNode(std::uint64_t id,
            double lon,
            double lat);

    auto getLon() const noexcept
        -> Longitude<Degree>;

    auto getLat() const noexcept
        -> Latitude<Degree>;

    auto getId() const noexcept
        -> std::uint64_t;

    auto operator==(const OSMNode& other) const noexcept
        -> bool;

    auto operator!=(const OSMNode& other) const noexcept
        -> bool;

private:
    std::uint64_t id_;
    Longitude<Degree> lon_;
    Latitude<Degree> lat_;
};

namespace std {

template<>
struct hash<OSMNode>
{
    auto operator()(const OSMNode& node) const noexcept
        -> std::size_t
    {
        auto h1 = std::hash<double>()(node.getLat().getValue());
        auto h2 = std::hash<double>()(node.getLon().getValue());
        return (h1 ^ (h2 << 1));
    }
};

} // namespace std
