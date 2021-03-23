#pragma once

#include <Graph.hpp>
#include <OSMNode.hpp>
#include <vector>


class IndependentSetCalculator
{
public:
    IndependentSetCalculator(const Graph& graph) noexcept;

    auto hasAnotherSet() const noexcept
        -> bool;

    auto calculateNextSet() noexcept
        -> std::vector<NodeId>;

private:
    auto sortNodesByDegree() noexcept
        -> void;

    auto cleanup() noexcept
        -> void;

private:
    const Graph& graph_;
    std::vector<NodeId> nodes_;
    std::vector<bool> visited_;
    std::vector<NodeId> touched_;
};
