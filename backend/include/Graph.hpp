#pragma once

#include <Range.hpp>
#include <SphericalGrid.hpp>
#include <nonstd/span.hpp>
#include <utility>

struct Edge
{
    Edge(NodeId source,
         NodeId target,
         Distance distance,
         std::optional<std::pair<EdgeId, EdgeId>> shortcut_for)
        : source(source),
          target(target),
          distance(distance),
          shortcut_for(std::move(shortcut_for)) {}

    NodeId source;
    NodeId target;
    Distance distance;
    std::optional<std::pair<EdgeId, EdgeId>> shortcut_for;
};

class Graph
{
public:
    Graph(SphericalGrid&& grid);

    auto idToLat(NodeId id) const noexcept
        -> Latitude<Degree>;

    auto idToLng(NodeId id) const noexcept
        -> Longitude<Degree>;

    auto idToM(NodeId id) const noexcept
        -> std::size_t;

    auto idToN(NodeId id) const noexcept
        -> std::size_t;

    auto isValidId(NodeId id) const noexcept
        -> bool;

    auto getEdgeIdsOf(NodeId node) const noexcept
        -> nonstd::span<const EdgeId>;

    auto getEdge(EdgeId) const noexcept
        -> const Edge&;

    auto size() const noexcept
        -> std::size_t;

    auto doesEdgeExist(NodeId source, NodeId target) const noexcept
        -> bool;

    auto rebuildWith(std::unordered_map<NodeId, std::vector<Edge>> new_edges,
                     const std::vector<NodeId>& contracted_nodes,
                     Level current_level) noexcept
        -> void;

    auto snapToGridNode(Latitude<Degree> lat, Longitude<Degree> lng) const noexcept
        -> NodeId;

    auto gridToId(std::size_t m, std::size_t n) const noexcept
        -> NodeId;

    auto isLandNode(NodeId node) const noexcept
        -> bool;

    auto getLevelOf(NodeId node) const noexcept
        -> Level;

    auto isAlreadyContracted(NodeId node) const noexcept
        -> bool;

    auto getDegree(NodeId node) const noexcept
        -> std::size_t;

    auto getInverserEdgeId(EdgeId id) const noexcept
        -> std::optional<EdgeId>;

private:
    auto getSnapNodeCandidate(Latitude<Degree> lat,
                              Longitude<Degree> lng) const noexcept
        -> NodeId;

    auto getUpperGridNeigboursOf(std::size_t m, std::size_t n) const noexcept
        -> std::vector<NodeId>;

    auto getLowerGridNeigboursOf(std::size_t m, std::size_t n) const noexcept
        -> std::vector<NodeId>;

    auto getRowGridNeigboursOf(std::size_t m, std::size_t n) const noexcept
        -> std::vector<NodeId>;

    auto getGridNeigboursOf(std::size_t m, std::size_t n) const noexcept
        -> std::vector<NodeId>;

    auto isFullyContracted() const noexcept
        -> bool;



private:
    friend class GraphContractor;
    std::vector<std::size_t> ns_;
    std::vector<std::size_t> ms_;

    std::vector<EdgeId> neigbours_;
    std::vector<size_t> offset_;
    std::vector<Edge> edges_;

    mutable std::vector<bool> snap_settled_;
    const SphericalGrid grid_;

    std::vector<Level> level_;

    std::size_t max_edge_id_;
};
