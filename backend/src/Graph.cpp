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
      snap_settled_(g.size(), false),
      grid_(std::move(g)),
      level_(grid_.size(), LEVEL_NOT_SET),
      max_edge_id_(0)
{
    for(auto id : utils::range(grid_.size())) {
        if(!grid_.indexIsLand(id)) {
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
                               auto distance = ::distanceBetween(start_lat, start_lng, dest_lat, dest_lng);

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

    //insert dummy at the end
    edges_.emplace_back(NON_EXISTENT, NON_EXISTENT, UNREACHABLE, std::nullopt);
    neigbours_.emplace_back(max_edge_id_);
}


auto Graph::rebuildWith(std::unordered_map<NodeId, std::vector<Edge>> new_edges,
                        const std::vector<NodeId>& contracted_nodes,
                        Level current_level) noexcept
    -> void
{
    std::vector<EdgeId> neigbours;
    std::vector<size_t> offset(grid_.size() + new_edges.size() + 1, 0);

    //remove the dummy entry
    edges_.pop_back();

    for(auto id : utils::range(grid_.size())) {
        if(grid_.indexIsLand(id)) {
            continue;
        }
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

        offset[id + 1] = neigbours.size();
    }

    for(auto n : contracted_nodes) {
        level_[n] = current_level;
    }

    //insert dummy at the end
    edges_.emplace_back(NON_EXISTENT, NON_EXISTENT, UNREACHABLE, std::nullopt);
    neigbours.emplace_back(max_edge_id_);

    neigbours_ = std::move(neigbours);
    offset_ = std::move(offset);
}


auto Graph::getDegree(NodeId node) const noexcept
    -> std::size_t
{
    return getEdgeIdsOf(node).size();
}


auto Graph::getInverserEdgeId(EdgeId id) const noexcept
    -> std::optional<EdgeId>
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

    return std::nullopt;
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
