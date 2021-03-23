#pragma once

#include <OSMNode.hpp>
#include <vector>

class NodeLookup;
class CoastlineLookup;

class Polygon
{
public:
    Polygon(const std::vector<OSMNode>& nodes);

    auto pointInPolygon(Latitude<Degree> lat, Longitude<Degree> lng) const
        -> bool;

    auto numberOfPoints() const
        -> std::size_t;

    auto getLatAndLng() const
        -> std::vector<std::pair<double, double>>;

private:
    std::vector<double> x_;
    std::vector<double> y_;
    std::vector<double> z_;
    Latitude<Degree> top_;
    Longitude<Degree> left_;
    Latitude<Degree> bottom_;
    Longitude<Degree> right_;

    auto pointInRectangle(Latitude<Degree> lat, Longitude<Degree> lng) const -> bool;
};


auto calculatePolygons(CoastlineLookup&& coastline_lookup,
                       NodeLookup&& node_lookup) noexcept
    -> std::vector<Polygon>;
