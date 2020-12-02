#include <Coastline.hpp>
#include <CoastlineLookup.hpp>
#include <NodeLookup.hpp>
#include <Utils.hpp>
#include <fmt/core.h>
#include <string>
#include <unordered_map>
#include <vector>

auto CoastlineLookup::addCoastline(std::uint64_t osmid,
                                   std::vector<std::uint64_t> refs) noexcept
    -> void
{
    coastlines_.try_emplace(osmid, std::move(refs));
}

auto CoastlineLookup::getCoastline(std::uint64_t osmid) const noexcept
    -> std::optional<CRef<Coastline>>
{
    auto iter = coastlines_.find(osmid);
    if(iter == std::end(coastlines_)) {
        return std::nullopt;
    }

    return std::cref(iter->second);
}

auto CoastlineLookup::deleteCoastline(std::uint64_t osmid) noexcept
    -> void
{
    coastlines_.erase(osmid);
}


auto CoastlineLookup::calculatePolygons(const NodeLookup& node_lookup) const noexcept
    -> std::vector<Polygon>
{
    std::unordered_map<std::uint64_t, Coastline> coastlines;

    for(const auto& [id, coastline] : coastlines_) {
        if(coastline.getRefs().empty()) {
            continue;
        }

        auto last = coastline.getRefs().back();
        auto first = coastline.getRefs().front();

        auto iter = coastlines.find(first);

        if(iter == std::end(coastlines)) {
            coastlines.emplace(last, coastline);
            continue;
        }

        auto old_way = std::move(iter->second.getRefs());
        const auto& new_refs = coastline.getRefs();

        std::copy(std::begin(new_refs) + 1,
                  std::end(new_refs),
                  std::back_inserter(old_way));

        coastlines.erase(first);
        coastlines.try_emplace(last, std::move(old_way));
    }

    std::vector<Polygon> polygons;
    std::transform(std::make_move_iterator(std::begin(coastlines)),
                   std::make_move_iterator(std::end(coastlines)),
                   std::back_inserter(polygons),
                   [&](auto elem) {
                       std::vector<OSMNode> nodes;
                       const auto& refs = elem.second.getRefs();

                       std::transform(std::begin(refs),
                                      std::end(refs),
                                      std::back_inserter(nodes),
                                      [&](auto id) {
                                          return node_lookup.getNode(id).value().get();
                                      });

                       return Polygon{std::move(nodes)};
                   });

    return polygons;
}


auto calculatePolygons(CoastlineLookup&& coastline_lookup,
                       NodeLookup&& node_lookup) noexcept
    -> std::vector<Polygon>
{
    std::unordered_map<std::uint64_t, Coastline> coastlines;
    for(auto [_, coastline] : std::move(coastline_lookup.coastlines_)) {
        auto first = coastline.getRefs().front();
        coastlines.emplace(first, std::move(coastline));
    }

    std::vector<Polygon> polygons;
    while(!coastlines.empty()) {
        auto [first, current_line] = std::move(*coastlines.begin());

        while(current_line.getRefs().front() != current_line.getRefs().back()) {
            auto current_last = current_line.getRefs().back();

            auto [inner_first, append_line] = std::move(*coastlines.find(current_last));
            coastlines.erase(current_last);

            auto new_nodes = std::move(current_line.getRefs());
            std::copy(std::begin(append_line.getRefs()) + 1,
                      std::end(append_line.getRefs()),
                      std::back_inserter(new_nodes));

            current_line = std::move(new_nodes);
        }

        coastlines.erase(first);

        auto polygon_nodes = node_lookup.idsToNodes(current_line.getRefs());
    }


    return polygons;
}
