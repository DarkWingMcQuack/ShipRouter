#include <Dijkstra.hpp>
#include <Graph.hpp>
#include <SphericalGrid.hpp>
#include <functional>
#include <numeric>
#include <optional>
#include <queue>
#include <string_view>
#include <vector>

Dijkstra::Dijkstra(const Graph& graph) noexcept
    : graph_(graph),
      distances_(graph_.size(), UNREACHABLE),
      settled_(graph_.size(), false),
      previous_nodes_(graph_.size(), NON_EXISTENT),
      pq_(DijkstraQueueComparer{}) {}


auto Dijkstra::findRoute(NodeId source, NodeId target) noexcept
    -> std::optional<std::pair<Path, Distance>>
{
    if(source == last_source_
       and isSettled(target)
       and previous_nodes_[target] != NON_EXISTENT) {
        return extractShortestPath(source, target);
    }

    if(source != last_source_ or previous_nodes_[target] == NON_EXISTENT) {
        last_source_ = source;
        reset();
        pq_.emplace(source, 0l);
        setDistanceTo(source, 0);
        touched_.emplace_back(source);
    }

    while(!pq_.empty()) {
        const auto [current_node, current_dist] = pq_.top();

        settle(current_node);

        if(current_node == target) {
            return extractShortestPath(source, target);
        }

        //pop after the return, otherwise we loose a value
        //when reusing the pq
        pq_.pop();

        const auto edge_ids = graph_.getEdgeIdsOf(current_node);

        for(auto id : edge_ids) {
            const auto& edge = graph_.getEdge(id);
            const auto neig = edge.target_;
            const auto dist = edge.distance_;

            auto neig_dist = getDistanceTo(neig);
            const auto new_dist = current_dist + dist;

            if(UNREACHABLE != current_dist and neig_dist > new_dist) {
                touched_.emplace_back(neig);
                setDistanceTo(neig, new_dist);
                pq_.emplace(neig, new_dist);
                previous_nodes_[neig] = current_node;
            }
        }
    }

    return extractShortestPath(source, target);
}

auto Dijkstra::findDistance(NodeId source, NodeId target) noexcept
    -> Distance
{
    return computeDistance(source, target);
}

auto Dijkstra::getDistanceTo(NodeId n) const noexcept
    -> Distance
{
    return distances_[n];
}

auto Dijkstra::setDistanceTo(NodeId n, Distance distance) noexcept
    -> void
{
    distances_[n] = distance;
}

auto Dijkstra::extractShortestPath(NodeId source, NodeId target) const noexcept
    -> std::optional<std::pair<Path, Distance>>
{
    //check if a path exists
    if(UNREACHABLE == getDistanceTo(target)) {
        return std::nullopt;
    }

    Path path{target};
    while(path[0] != source) {
        path.insert(std::begin(path),
                    previous_nodes_[path[0]]);
    }

    return std::pair{path, getDistanceTo(target)};
}

auto Dijkstra::reset() noexcept
    -> void
{
    for(auto n : touched_) {
        unSettle(n);
        setDistanceTo(n, UNREACHABLE);
        previous_nodes_[n] = NON_EXISTENT;
    }
    touched_.clear();
    pq_ = DijkstraQueue{DijkstraQueueComparer{}};
}

auto Dijkstra::unSettle(NodeId n)
    -> void
{
    settled_[n] = false;
}

auto Dijkstra::settle(NodeId n) noexcept
    -> void
{
    settled_[n] = true;
}

auto Dijkstra::isSettled(NodeId n)
    -> bool
{
    return settled_[n];
}

auto Dijkstra::computeDistance(NodeId source, NodeId target) noexcept
    -> Distance
{
    if(source == last_source_
       && isSettled(target)) {
        return getDistanceTo(target);
    }

    if(source != last_source_) {
        last_source_ = source;
        reset();
        pq_.emplace(source, 0l);
        setDistanceTo(source, 0);
        touched_.emplace_back(source);
    }

    while(!pq_.empty()) {
        const auto [current_node, current_dist] = pq_.top();

        settle(current_node);

        if(current_node == target) {
            return current_dist;
        }

        //pop after the return, otherwise we loose a value
        //when reusing the pq
        pq_.pop();

        const auto edge_ids = graph_.getEdgeIdsOf(current_node);

        for(auto id : edge_ids) {
            const auto& edge = graph_.getEdge(id);
            const auto neig = edge.target_;
            const auto dist = edge.distance_;

            auto neig_dist = getDistanceTo(neig);
            const auto new_dist = current_dist + dist;

            if(UNREACHABLE != current_dist and neig_dist > new_dist) {
                touched_.emplace_back(neig);
                setDistanceTo(neig, new_dist);
                pq_.emplace(neig, new_dist);
            }
        }
    }

    return getDistanceTo(target);
}
