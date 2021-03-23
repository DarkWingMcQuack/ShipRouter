#include <Graph.hpp>
#include <Range.hpp>
#include <SphericalGrid.hpp>
#include <Vector3D.hpp>
#include <fmt/ranges.h>
#include <iostream>
#include <nonstd/span.hpp>
#include <queue>


Graph::Graph(SphericalGrid&& g)
    : offset_(g.size() + 1, 0),
      snap_selled_(g.size(), false),
      grid_(std::move(g)),
	  level_(g.size(), 0)
{
    for(auto id : utils::range(grid_.size())) {
        if(!grid_.indexIsLand(id)) {
            auto neigs = grid_.getNeighbours(id);

            std::sort(std::begin(neigs),
                      std::end(neigs));
            auto remove_iter =
                std::unique(std::begin(neigs),
                            std::end(neigs));

            neigs.erase(remove_iter, std::end(neigs));

            std::vector<std::pair<NodeId, Distance>> neig_dist;
            std::transform(std::begin(neigs),
                           std::end(neigs),
                           std::back_inserter(neig_dist),
                           [&](auto neig) {
                               auto [start_lat, start_lng] = grid_.idToLatLng(id);
                               auto [dest_lat, dest_lng] = grid_.idToLatLng(neig);
                               auto distance = ::distanceBetween(start_lat, start_lng, dest_lat, dest_lng);

                               return std::pair{neig, distance};
                           });

            neigbours_ = concat(std::move(neigbours_),
                                std::move(neig_dist));
        }

        offset_[id + 1] = neigbours_.size();

        auto [m, n] = grid_.idToGrid(id);
        ns_.emplace_back(n);
        ms_.emplace_back(m);
    }

    //insert dummy at the end
    neigbours_.emplace_back(std::numeric_limits<NodeId>::max(), UNREACHABLE);
}

auto Graph::idToLat(NodeId id) const noexcept
    -> Latitude<Degree>
{
    return grid_.lats_[id];
}

auto Graph::idToLng(NodeId id) const noexcept
    -> Longitude<Degree>
{
    return grid_.lngs_[id];
}

auto Graph::idToM(NodeId id) const noexcept
    -> std::size_t
{
    return ms_[id];
}

auto Graph::idToN(NodeId id) const noexcept
    -> std::size_t
{
    return ns_[id];
}

auto Graph::isValidId(NodeId id) const noexcept
    -> bool
{
    return id < size();
}

auto Graph::size() const noexcept
    -> std::size_t
{
    return grid_.lats_.size();
}

auto Graph::getNeigboursOf(NodeId node) const noexcept
    -> nonstd::span<const std::pair<NodeId, Distance>>
{
    const auto start_offset = offset_[node];
    const auto end_offset = offset_[node + 1];
    const auto* start = &neigbours_[start_offset];
    const auto* end = &neigbours_[end_offset];

    return nonstd::span{start, end};
}

auto Graph::gridToId(std::size_t m, std::size_t n) const noexcept
    -> NodeId
{
    return grid_.gridToID(m, n);
}

auto Graph::isLandNode(NodeId node) const noexcept
    -> bool
{
    return !grid_.is_water_[node];
}


auto Graph::getRowGridNeigboursOf(std::size_t m, std::size_t n) const noexcept
    -> std::vector<NodeId>
{
    auto on_grid = grid_.getRowNeighbours(m, n);
    std::vector<NodeId> ids;
    std::transform(std::begin(on_grid),
                   std::end(on_grid),
                   std::back_inserter(ids),
                   [&](auto pair) {
                       auto [m, n] = pair;
                       return gridToId(m, n);
                   });

    return ids;
}

auto Graph::getLowerGridNeigboursOf(std::size_t m, std::size_t n) const noexcept
    -> std::vector<NodeId>
{
    auto on_grid = grid_.getLowerNeighbours(m, n);
    std::vector<NodeId> ids;
    std::transform(std::begin(on_grid),
                   std::end(on_grid),
                   std::back_inserter(ids),
                   [&](auto pair) {
                       auto [m, n] = pair;
                       return gridToId(m, n);
                   });

    return ids;
}

auto Graph::getUpperGridNeigboursOf(std::size_t m, std::size_t n) const noexcept
    -> std::vector<NodeId>
{
    auto on_grid = grid_.getUpperNeighbours(m, n);
    std::vector<NodeId> ids;
    std::transform(std::begin(on_grid),
                   std::end(on_grid),
                   std::back_inserter(ids),
                   [&](auto pair) {
                       auto [m, n] = pair;
                       return gridToId(m, n);
                   });

    return ids;
}

auto Graph::getGridNeigboursOf(std::size_t m, std::size_t n) const noexcept
    -> std::vector<NodeId>
{
    return concat(getUpperGridNeigboursOf(m, n),
                  getLowerGridNeigboursOf(m, n),
                  getRowGridNeigboursOf(m, n));
}

auto Graph::getSnapNodeCandidate(Latitude<Degree> lat,
                                 Longitude<Degree> lng) const noexcept
    -> NodeId
{
    const auto [m, n] = grid_.sphericalToGrid(lat.toRadian(), lng.toRadian());

    const auto id = gridToId(m, n);

    fmt::print("index: {}\n", id);

    std::vector<NodeId> candidates;
    std::vector<NodeId> touched_nodes;

    if(!isLandNode(id)) {
        candidates.emplace_back(id);
    }

    auto workstack = getGridNeigboursOf(m, n);

    while(!workstack.empty()) {
        fmt::print("workstack: {}", workstack);
        std::cout << std::endl;
        fmt::print("candidates: {}", candidates);
        std::cout << std::endl;
        const auto candidate = workstack.back();
        workstack.pop_back();
        snap_selled_[candidate] = true;
        touched_nodes.emplace_back(candidate);

        if(!isLandNode(candidate)) {
            candidates.emplace_back(candidate);
            continue;
        }

        if(snap_selled_[candidate]) {
            continue;
        }

        workstack = concat(std::move(workstack),
                           getGridNeigboursOf(ms_[candidate],
                                              ns_[candidate]));
    }

    for(auto touched : touched_nodes) {
        snap_selled_[touched] = false;
    }

    return *std::min_element(std::cbegin(candidates),
                             std::cend(candidates),
                             [&](auto lhs, auto rhs) {
                                 auto lhs_lat = grid_.lats_[lhs];
                                 auto lhs_lng = grid_.lngs_[lhs];

                                 auto rhs_lat = grid_.lats_[rhs];
                                 auto rhs_lng = grid_.lngs_[rhs];

                                 return ::distanceBetween(lat, lng, lhs_lat, lhs_lng)
                                     < ::distanceBetween(lat, lng, rhs_lat, rhs_lng);
                             });
}

auto Graph::snapToGridNode(Latitude<Degree> lat,
                           Longitude<Degree> lng) const noexcept
    -> NodeId
{
    auto candidate = getSnapNodeCandidate(lat, lng);

    std::priority_queue candidates(
        [&](auto id1, auto id2) {
            return ::distanceBetween(lat, lng,
                                     grid_.lats_[id1],
                                     grid_.lngs_[id1])
                > ::distanceBetween(lat, lng,
                                    grid_.lats_[id2],
                                    grid_.lngs_[id2]);
        },
        std::vector{candidate});

    while(true) {
        const auto best_before_insert = candidates.top();
        for(auto [neig, _] : getNeigboursOf(best_before_insert)) {
            candidates.emplace(neig);
        }
        const auto best_after_insert = candidates.top();

        if(best_before_insert == best_after_insert) {
            break;
        }
    }

    return candidates.top();
}
