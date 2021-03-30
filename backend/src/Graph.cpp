#include <Dijkstra.hpp>
#include <Graph.hpp>
#include <Range.hpp>
#include <SphericalGrid.hpp>
#include <Vector3D.hpp>
#include <fmt/ranges.h>
#include <iostream>
#include <nonstd/span.hpp>
#include <queue>
#include <unordered_set>


Graph::Graph(SphericalGrid&& g)
    : offset_(g.size() + 1, 0),
      snap_settled_(g.size(), false),
      levels(g.size(), 0),
      grid_(std::move(g))
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

            std::vector<Edge> neig_dist;
            std::transform(std::begin(neigs),
                           std::end(neigs),
                           std::back_inserter(neig_dist),
                           [&](auto neig) {
                               auto [start_lat, start_lng] = grid_.idToLatLng(id);
                               auto [dest_lat, dest_lng] = grid_.idToLatLng(neig);
                               auto distance = ::distanceBetween(start_lat, start_lng, dest_lat, dest_lng);

                               return Edge{neig, static_cast<Distance>(distance), std::nullopt};
                           });

            edges_ = concat(std::move(edges_),
                            std::move(neig_dist));
        }

        offset_[id + 1] = edges_.size();

        auto [m, n] = grid_.idToGrid(id);
        ns_.emplace_back(n);
        ms_.emplace_back(m);
    }

    //insert dummy at the end
    edges_.emplace_back(std::numeric_limits<NodeId>::max(), UNREACHABLE, std::nullopt);
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

auto Graph::relaxEdges(NodeId node) const noexcept
    -> nonstd::span<const Edge>
{
    const auto start_offset = offset_[node];
    const auto end_offset = offset_[node + 1];
    const auto* start = &edges_[start_offset];
    const auto* end = &edges_[end_offset];

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
        for(auto e : relaxEdges(best_before_insert)) {
            candidates.emplace(e.target);
        }
        const auto best_after_insert = candidates.top();

        if(best_before_insert == best_after_insert) {
            break;
        }
    }

    return candidates.top();
}

// === stuff for ch and contraction === //

void Graph::contract() noexcept
{
    fmt::print("Starting graph contraction...");
    while(!fully_contracted) {
        contractionStep();
    }
    fmt::print("Done contracting.");
}

void Graph::contractionStep() noexcept
{
    fmt::print("Starting contraction step {}", current_level);
    /*
    * 1. create independent set of nodes
    * 2. for each node: 
    *      calculate distances from and to all neighbors and node edge diff
    * 3. sort by edge diff ascending
    * 4. Create shortcuts for the first n nodes (with lowest edge diff)
    * 5. Insert new edges into existing graph
      */

    // 1.
    auto indep_nodes = independentSet();
    fmt::print("Independent set contains {} nodes", indep_nodes.size());
    if(indep_nodes.empty()) {
        fully_contracted = true;
        return;
    }

    // 2.
    Dijkstra dijkstra{*this};
    std::vector<std::tuple<int32_t, NodeId, std::vector<Edge>>> newEdgeCandidates{};
    for(auto node : indep_nodes) {
        std::unordered_set<EdgeId> obsolete_edges{};
        std::vector<Edge> new_edges{};
        auto neighbors = relaxEdges(node);
        for(auto neigh1 : neighbors) {
            auto source = neigh1.target;
            // shortest path from neigh to all other neighbors
            for(auto neigh2 : neighbors) {
                auto target = neigh2.target;
                if(source == target) {
                    continue;
                }
                auto res = dijkstra.findRoute(source, target);
                if(!res) {
                    continue;
                }
                // check if shortest path contains node
                auto [path, cost] = res.value();
                if(path.size() == 3 and path[0] == source and path[1] == node and path[2] == target) {
                    new_edges.emplace_back(target, cost, std::pair{420, 420});
                }
            }
        }
        // TODO: consider alternative to just use amount of neighbors as removed edges
        auto edge_diff = new_edges.size() - obsolete_edges.size();
        newEdgeCandidates.emplace_back(edge_diff, node, new_edges);
    }

    // 3.
    std::sort(newEdgeCandidates.begin(), newEdgeCandidates.end(), [](auto first, auto second) {
        return std::get<0>(first) < std::get<0>(second);
    });

    // 4.
    for(auto i = 0; i < newEdgeCandidates.size() / 2; i++) {
        auto entry = newEdgeCandidates[i];
        NodeId source = std::get<1>(entry);
        auto edges = std::get<2>(entry);
        insertEdges(source, edges);
    }

    // increment level and assign to nodes
    current_level++;
    for(auto node : indep_nodes) {
        levels[node] = current_level;
    }
}

std::vector<NodeId> Graph::independentSet() const
{
    std::vector<bool> visited(size(), false);
    std::vector<NodeId> indepNodes;

    for(auto i = 0; i < size(); i++) {
        if(levels[i] == 0 && !visited[i]) {
            auto edges = relaxEdges(i);
            for(auto e : edges) {
                visited[e.target] = true;
            }
            indepNodes.emplace_back(i);
        }
    }
    return indepNodes;
}

void Graph::insertEdges(NodeId source, std::vector<Edge> edges)
{
    // TODO: Insert new edges into graph
}