#pragma once

#include <Range.hpp>
#include <SphericalGrid.hpp>
#include <nonstd/span.hpp>

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

    auto getNeigboursOf(NodeId node) const noexcept
        -> nonstd::span<const std::pair<NodeId, Distance>>;

    auto size() const noexcept
        -> std::size_t;

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
    std::vector<std::size_t> ns_;
    std::vector<std::size_t> ms_;

    std::vector<std::pair<NodeId, Distance>> neigbours_;
    std::vector<size_t> offset_;

    mutable std::vector<bool> snap_selled_;
    const SphericalGrid grid_;

    std::vector<Level> level_;
};
