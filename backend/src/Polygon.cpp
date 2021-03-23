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


namespace {

auto latLngTo3D(Latitude<Radian> lat, Longitude<Radian> lng) noexcept
    -> std::tuple<double, double, double>
{
    return std::tuple{std::cos(lat.getValue()) * std::cos(lng.getValue()),
                      std::cos(lat.getValue()) * std::sin(lng.getValue()),
                      std::sin(lat.getValue())};
}

} // namespace

Polygon::Polygon(const std::vector<OSMNode>& nodes)
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
        auto [x, y, z] = latLngTo3D(lat, lng);
        auto lenght = std::sqrt(x * x + y * y + z * z);
        x_.emplace_back(x / lenght);
        y_.emplace_back(y / lenght);
        z_.emplace_back(z / lenght);
    }
}

auto Polygon::pointInRectangle(Latitude<Degree> lat, Longitude<Degree> lng) const -> bool
{
    return bottom_ <= lat and lat <= top_ and left_ <= lng and lng <= right_;
}

auto Polygon::pointInPolygon(Latitude<Degree> lat, Longitude<Degree> lng) const
    -> bool
{
    if(!pointInRectangle(lat, lng)) {
        return false;
    }
    const auto size = numberOfPoints();
    const auto range = utils::range(size);
    const auto p = Vector3D{lat.toRadian(), lng.toRadian()}
                       .normalize();

    // get vectors from p to each vertex
    std::vector<Vector3D> vec_to_vertex;
    std::transform(std::begin(range),
                   std::end(range),
                   std::back_inserter(vec_to_vertex),
                   [&](auto idx) {
                       return p - Vector3D{x_[idx], y_[idx], z_[idx]};
                   });


    auto sum = std::transform_reduce(
        std::execution::par_unseq,
        std::begin(range),
        std::end(range),
        0.0,
        [&](auto current, auto next) {
            return current + next;
        },
        [&](auto idx) {
            const auto& first = vec_to_vertex[idx % size];
            const auto& second = vec_to_vertex[(idx + 1) % size];
            return first.angleBetween(second, p);
        });


    // external points should sum up to something close to 0
    // internal points should sum up to something smaller than pi

    return std::abs(sum) > PI;
}


namespace {

auto vec3DtoLatLong(double x, double y, double z) noexcept
{
    auto lat = std::atan2(z, std::sqrt(x * x + y * y));
    auto lng = std::atan2(y, x);

    return std::pair{Latitude<Radian>{lat},
                     Longitude<Radian>{lng}};
}

} // namespace

auto Polygon::getLatAndLng() const
    -> std::vector<std::pair<double, double>>
{
    std::vector<std::pair<double, double>> ret_vec;
    auto range = utils::range(numberOfPoints());

    std::transform(std::begin(range),
                   std::end(range),
                   std::back_inserter(ret_vec),
                   [&](auto idx) {
                       auto [lat, lng] = vec3DtoLatLong(x_[idx], y_[idx], z_[idx]);
                       return std::pair{lat.toDegree().getValue(),
                                        lng.toDegree().getValue()};
                   });

    return ret_vec;
}

auto Polygon::numberOfPoints() const
    -> std::size_t
{
    return x_.size();
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
