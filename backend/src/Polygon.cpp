#include <CoastlineLookup.hpp>
#include <Constants.hpp>
#include <LatLng.hpp>
#include <NodeLookup.hpp>
#include <OSMNode.hpp>
#include <Polygon.hpp>
#include <Range.hpp>
#include <SphericalPoint.hpp>
#include <Vector3D.hpp>
#include <execution>
#include <fmt/core.h>
#include <numeric>


Polygon::Polygon(const std::vector<OSMNode>& nodes) noexcept
{
    const auto first = nodes.front();
    bottom_ = first.getLat();
    top_ = first.getLat();
    left_ = first.getLon();
    right_ = first.getLon();
    for(const auto& n : nodes) {
        if(n.getLat() < bottom_) {
            bottom_ = n.getLat();
        }
        if(n.getLat() > top_) {
            top_ = n.getLat();
        }
        if(n.getLon() < left_) {
            left_ = n.getLon();
        }
        if(n.getLon() > right_) {
            right_ = n.getLon();
        }
        auto lat = n.getLat().toRadian();
        auto lng = n.getLon().toRadian();

        points_.emplace_back(lat, lng);
    }
}

auto Polygon::pointInRectangle(const Latitude<Degree>& lat,
                               const Longitude<Degree>& lng) const noexcept
    -> bool
{
    return bottom_ <= lat
        and lat <= top_
        and left_ <= lng
        and lng <= right_;
}

auto Polygon::pointInPolygon(const Latitude<Degree>& lat,
                             const Longitude<Degree>& lng,
                             const Vector3D& p) const noexcept
    -> bool
{
    if(!pointInRectangle(lat, lng)) {
        return false;
    }

    const auto size = numberOfPoints();
    const auto range = utils::range(size);

    const auto sum = std::transform_reduce(
        std::execution::unseq,
        std::begin(range),
        std::end(range),
        0.0,
        [&](auto current, auto next) {
            return current + next;
        },
        [&](auto idx) {
            const auto first = p - points_[idx % size];
            const auto second = p - points_[(idx + 1) % size];
            return first.angleBetween(second, p);
        });


    // external points should sum up to something close to 0
    // internal points should sum up to something smaller than pi
    return std::abs(sum) > PI;
}


auto Polygon::getLatAndLng() const noexcept
    -> std::vector<std::pair<double, double>>
{
    std::vector<std::pair<double, double>> ret_vec;
    std::transform(std::begin(points_),
                   std::end(points_),
                   std::back_inserter(ret_vec),
                   [&](const auto& point) {
                       auto [lat, lng] = point.toLatLng();
                       return std::pair{lat.toDegree().getValue(),
                                        lng.toDegree().getValue()};
                   });

    return ret_vec;
}

auto Polygon::numberOfPoints() const noexcept
    -> std::size_t
{
    return points_.size();
}

auto calculatePolygons(CoastlineLookup&& coastline_lookup,
                       NodeLookup&& node_lookup) noexcept
    -> std::vector<Polygon>
{
    auto coastlines = std::move(coastline_lookup.coastlines_);

    std::vector<Polygon> polygons;
    while(!coastlines.empty()) {
        auto [first, current_line] = std::move(*coastlines.begin());

        while(current_line.getRefs().front() != current_line.getRefs().back()) {
            auto current_last = current_line.getRefs().back();

            auto iter = coastlines.find(current_last);
            if(iter == std::end(coastlines)) {
                current_line.getRefs().emplace_back(current_line.getRefs().front());
                continue;
            }

            auto [inner_first, append_line] = std::move(*iter);
            coastlines.erase(current_last);

            auto new_nodes = std::move(current_line.getRefs());
            std::copy(std::begin(append_line.getRefs()) + 1,
                      std::end(append_line.getRefs()),
                      std::back_inserter(new_nodes));

            current_line = std::move(new_nodes);
        }

        coastlines.erase(first);

        std::vector<OSMNode> polygon_nodes;
        std::transform(std::begin(current_line.getRefs()),
                       std::end(current_line.getRefs()),
                       std::back_inserter(polygon_nodes),
                       [&](auto id) {
                           return std::move(node_lookup.nodes_.find(id)->second);
                       });

        polygons.emplace_back(std::move(polygon_nodes));
    }

    return polygons;
}
