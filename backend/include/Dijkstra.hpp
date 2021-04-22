#pragma once

#include <Graph.hpp>
#include <SphericalGrid.hpp>
#include <functional>
#include <optional>
#include <queue>
#include <string_view>
#include <vector>

struct DijkstraQueueComparer
{
    auto operator()(const std::pair<NodeId, Distance>& lhs,
                    const std::pair<NodeId, Distance>& rhs) const noexcept
        -> bool
    {
        return lhs.second > rhs.second;
    }
};

using DijkstraQueue = std::priority_queue<std::pair<NodeId, Distance>,
                                          std::vector<std::pair<NodeId, Distance>>,
                                          DijkstraQueueComparer>;



class Dijkstra
{
public:
    Dijkstra(const Graph& graph) noexcept;
    Dijkstra() = delete;
    Dijkstra(Dijkstra&&) = default;
    Dijkstra(const Dijkstra&) = default;
    auto operator=(const Dijkstra&) -> Dijkstra& = delete;
    auto operator=(Dijkstra&&) -> Dijkstra& = delete;

    auto findRoute(NodeId source, NodeId target) noexcept
        -> std::optional<std::pair<Path, Distance>>;

    auto findDistance(NodeId source, NodeId target) noexcept
        -> Distance;

    auto shortestPathSTGoesOverU(NodeId s,
                                 NodeId t,
                                 NodeId u,
                                 Distance distance_over_u) noexcept
        -> bool;

    auto getAverageQPopsPerQuery() const noexcept
        -> double;

private:
    auto getDistanceTo(NodeId n) const noexcept
        -> Distance;

    auto setDistanceTo(NodeId n, Distance distance) noexcept
        -> void;

    auto computeDistance(NodeId source, NodeId target) noexcept
        -> Distance;

    auto extractShortestPath(NodeId source, NodeId target) const noexcept
        -> std::optional<std::pair<Path, Distance>>;

    auto unSettle(NodeId n)
        -> void;

    auto settle(NodeId n) noexcept
        -> void;

    auto isSettled(NodeId n)
        -> bool;

    auto reset() noexcept
        -> void;

private:
    const Graph& graph_;
    std::vector<Distance> distances_;
    std::vector<bool> settled_;
    std::vector<NodeId> touched_;
    std::vector<NodeId> previous_nodes_;
    DijkstraQueue pq_;
    std::optional<NodeId> last_source_;
    std::optional<NodeId> last_middle_node_;

    std::size_t q_pops_ = 0;
    std::size_t queries_ = 0;
};
