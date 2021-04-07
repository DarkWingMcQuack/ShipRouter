#pragma once

#include <OSMNode.hpp>
#include <Vector3D.hpp>
#include <vector>

class NodeLookup;
class CoastlineLookup;

class Polygon
{
public:
    Polygon(const std::vector<OSMNode>& nodes) noexcept;

    auto pointInPolygon(Latitude<Degree> lat,
                        Longitude<Degree> lng,
                        const Vector3D& point) const noexcept
        -> bool;

    auto numberOfPoints() const noexcept
        -> std::size_t;

    auto getLatAndLng() const noexcept
        -> std::vector<std::pair<double, double>>;

private:
    auto pointInRectangle(Latitude<Degree> lat,
                          Longitude<Degree> lng) const noexcept
        -> bool;

private:
    std::vector<double> x_;
    std::vector<double> y_;
    std::vector<double> z_;
    Latitude<Degree> top_;
    Longitude<Degree> left_;
    Latitude<Degree> bottom_;
    Longitude<Degree> right_;
};


auto calculatePolygons(CoastlineLookup&& coastline_lookup,
                       NodeLookup&& node_lookup) noexcept
    -> std::vector<Polygon>;
