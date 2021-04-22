#pragma once

#include <Graph.hpp>
#include <queue>


class CHDijkstra
{
public:
    CHDijkstra(const Graph& graph) noexcept;
    CHDijkstra() = delete;
    CHDijkstra(CHDijkstra&&) = default;
    CHDijkstra(const CHDijkstra&) = default;
    auto operator=(const CHDijkstra&) -> CHDijkstra& = delete;
    auto operator=(CHDijkstra&&) -> CHDijkstra& = delete;

    auto findRoute(NodeId source, NodeId target) noexcept
        -> std::optional<std::pair<Path, Distance>>;

    auto findDistance(NodeId source, NodeId target) noexcept
        -> Distance;

    [[nodiscard]] auto getAverageQPopsPerQuery() const noexcept
        -> double;

private:
    auto resetForward() noexcept
        -> void;

    auto resetBackward() noexcept
        -> void;

    auto fillForwardInfo(NodeId source) noexcept
        -> void;

    auto fillBackwardInfo(NodeId target) noexcept
        -> void;

    auto findShortestPathCommonNode() noexcept
        -> std::optional<NodeId>;

    auto extractPath(NodeId source,
                     NodeId top_node,
                     NodeId target) const noexcept
        -> Path;

    auto extractSourcePathWrapped(NodeId source,
                                  NodeId top_node) const noexcept
        -> std::vector<EdgeId>;

    auto extractTargetPathWrapped(NodeId source,
                                  NodeId top_node) const noexcept
        -> std::vector<EdgeId>;

    auto unwrap(std::vector<EdgeId> ids,
                NodeId source,
                NodeId top_node,
                NodeId target) const noexcept
        -> Path;

private:
    const Graph& graph_;
    std::vector<Distance> forward_distances_;
    std::vector<NodeId> forward_settled_;
    std::vector<NodeId> forward_touched_;
    std::optional<NodeId> last_source_;
    std::vector<EdgeId> forward_best_ingoing_;
    std::vector<bool> forward_already_settled_;

    std::vector<Distance> backward_distances_;
    std::vector<NodeId> backward_settled_;
    std::vector<NodeId> backward_touched_;
    std::optional<NodeId> last_target_;
    std::vector<EdgeId> backward_best_ingoing_;
    std::vector<bool> backward_already_settled_;

    std::size_t q_pops_ = 0;
    std::size_t queries_ = 0;
};
