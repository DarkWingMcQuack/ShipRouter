#include <Graph.hpp>
#include <Range.hpp>
#include <SphericalGrid.hpp>
#include <Vector3D.hpp>
#include <fmt/core.h>
#include <fmt/ranges.h>
#include <iostream>
#include <nonstd/span.hpp>
#include <queue>


Graph::Graph(SphericalGrid&& g)
    : offset_(g.size() + 1, 0),
      snap_settled_(g.size(), false),
      grid_(std::move(g)),
      level_(grid_.size(), LEVEL_NOT_SET),
      max_edge_id_(0)
{
    for(auto id : utils::range(grid_.size())) {
        if(grid_.indexIsWater(id)) {
            auto neigs = grid_.getNeighbours(id);

            std::sort(std::begin(neigs),
                      std::end(neigs));

            neigs.erase(std::unique(std::begin(neigs),
                                    std::end(neigs)),
                        std::end(neigs));

            std::vector<Edge> new_edges;
            std::transform(std::begin(neigs),
                           std::end(neigs),
                           std::back_inserter(new_edges),
                           [&](auto neig) {
                               auto [start_lat, start_lng] = grid_.idToLatLng(id);
                               auto [dest_lat, dest_lng] = grid_.idToLatLng(neig);
                               auto distance = ::distanceBetween(start_lat,
                                                                 start_lng,
                                                                 dest_lat,
                                                                 dest_lng);

                               return Edge{id, neig, static_cast<Distance>(distance), std::nullopt};
                           });

            for(auto i = 0ul; i < new_edges.size(); i++) {
                neigbours_.emplace_back(max_edge_id_++);
            }

            edges_ = concat(std::move(edges_),
                            std::move(new_edges));
        }

        offset_[id + 1] = neigbours_.size();

        auto [m, n] = grid_.idToGrid(id);
        ns_.emplace_back(n);
        ms_.emplace_back(m);
    }

    // for(auto n : utils::range(grid_.size())) {
    //     auto edge_ids = getEdgeIdsOf(n);

    //     for(auto id : edge_ids) {

    //         const auto& edge = getEdge(id);
    //         auto source = edge.source;
    //         auto target = edge.target;

    //         if(source != n) {
    //             fmt::print("WTF IS GOING ON SOURCE WAS NOT N\n");
    //         }

    //         if(!doesEdgeExist(target, source)) {
    //             fmt::print("WTF IS GOING ON {}->{} EXISTS BUT BACKEDGE DOESNT\n", source, target);
    //         }
    //     }
    // }
}


auto Graph::getLevelOf(NodeId node) const noexcept
    -> Level
{
    return level_[node];
}


auto Graph::doesEdgeExist(NodeId source, NodeId target) const noexcept
    -> bool
{
    auto edge_ids = getEdgeIdsOf(source);
    return std::any_of(std::begin(edge_ids),
                       std::end(edge_ids),
                       [&](auto id) {
                           const auto& edge = getEdge(id);
                           return edge.source == source
                               and edge.target == target;
                       });
}

auto Graph::rebuildWith(std::unordered_map<NodeId, std::vector<Edge>> new_edges,
                        const std::vector<NodeId>& contracted_nodes,
                        Level current_level) noexcept
    -> void
{
    std::vector<EdgeId> neigbours;
    std::vector<size_t> offset(grid_.size() + new_edges.size() + 1, 0);

    for(auto id : utils::range(grid_.size())) {
        if(!grid_.indexIsLand(id)) {
            auto edge_ids = getEdgeIdsOf(id);

            neigbours.insert(std::end(neigbours),
                             std::begin(edge_ids),
                             std::end(edge_ids));

            auto shortcut_iter = new_edges.find(id);
            if(shortcut_iter != std::end(new_edges)) {

                for([[maybe_unused]] auto&& _ : shortcut_iter->second) {
                    neigbours.emplace_back(max_edge_id_++);
                }

                edges_ = concat(std::move(edges_),
                                std::move(shortcut_iter->second));
            }
        }

        offset[id + 1] = neigbours.size();
    }

    offset.shrink_to_fit();

    for(auto n : contracted_nodes) {
        level_[n] = current_level;
    }

    neigbours_ = std::move(neigbours);
    offset_ = std::move(offset);

    for(auto id : utils::range(grid_.size())) {
        auto edge_ids = getEdgeIdsOf(id);

        std::sort(std::begin(edge_ids),
                  std::end(edge_ids),
                  [&](auto lhs, auto rhs) {
                      auto lhs_target = getEdge(lhs).target;
                      auto rhs_target = getEdge(rhs).target;
                      return level_[lhs_target] > level_[rhs_target];
                  });
    }
}


auto Graph::getDegree(NodeId node) const noexcept
    -> std::size_t
{
    return getEdgeIdsOf(node).size();
}

auto Graph::numberOfEdges() const noexcept
    -> std::size_t
{
    return edges_.size();
}


auto Graph::getInverserEdgeId(EdgeId id) const noexcept
    -> EdgeId
{
    const auto& edge = getEdge(id);
    auto source = edge.source;
    auto target = edge.target;

    auto target_edge_ids = getEdgeIdsOf(target);

    for(auto inv_id : target_edge_ids) {
        const auto& inv_edge = getEdge(inv_id);

        if(inv_edge.target == source) {
            return inv_id;
        }
    }

    return EDGE_NOT_SET;
}

auto Graph::isAlreadyContracted(NodeId node) const noexcept
    -> bool
{
    return level_[node] != LEVEL_NOT_SET;
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

auto Graph::getEdgeIdsOf(NodeId node) const noexcept
    -> nonstd::span<const EdgeId>
{
    const auto start_offset = offset_[node];
    const auto end_offset = offset_[node + 1];
    const auto* start = &neigbours_[start_offset];
    const auto* end = &neigbours_[end_offset];

    return nonstd::span{start, end};
}


auto Graph::getEdgeIdsOf(NodeId node) noexcept
    -> nonstd::span<EdgeId>
{
    const auto start_offset = offset_[node];
    const auto end_offset = offset_[node + 1];
    auto* start = &neigbours_[start_offset];
    auto* end = &neigbours_[end_offset];

    return nonstd::span{start, end};
}

auto Graph::getEdge(EdgeId id) const noexcept
    -> const Edge&
{
    return edges_[id];
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


auto Graph::areOneSameRow(NodeId first, NodeId second) const noexcept
    -> bool
{
    auto [m1, _1] = grid_.idToGrid(first);
    auto [m2, _2] = grid_.idToGrid(second);

    return m1 == m2;
}

auto Graph::getSnapNodeCandidate(Latitude<Degree> lat,
                                 Longitude<Degree> lng) const noexcept
    -> NodeId
{
    const auto [m, n] = grid_.sphericalToGrid(lat.toRadian(), lng.toRadian());

    const auto id = gridToId(m, n);

    std::vector<NodeId> candidates;
    std::vector<NodeId> touched_nodes;

    if(!isLandNode(id)) {
        candidates.emplace_back(id);
    }

    auto workstack = getGridNeigboursOf(m, n);

    while(!workstack.empty()) {
        const auto candidate = workstack.back();
        workstack.pop_back();
        touched_nodes.emplace_back(candidate);

        if(!isLandNode(candidate)) {
            candidates.emplace_back(candidate);
            continue;
        }

        if(snap_settled_[candidate]) {
            continue;
        }

        workstack = concat(std::move(workstack),
                           getGridNeigboursOf(ms_[candidate],
                                              ns_[candidate]));
        snap_settled_[candidate] = true;
    }

    for(auto touched : touched_nodes) {
        snap_settled_[touched] = false;
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
        for(auto edge_id : getEdgeIdsOf(best_before_insert)) {
            const auto& edge = getEdge(edge_id);
            candidates.emplace(edge.target);
        }
        const auto best_after_insert = candidates.top();

        if(best_before_insert == best_after_insert) {
            break;
        }
    }

    return candidates.top();
}
