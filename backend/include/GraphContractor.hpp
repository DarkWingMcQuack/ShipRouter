#pragma once

#include <Dijkstra.hpp>
#include <Graph.hpp>

struct NodeContractionResult
{
    std::unordered_map<NodeId, std::vector<Edge>> shortcuts;
    std::int64_t edge_diff;
};

struct IndependentSetContractionResult
{
    std::unordered_map<NodeId, std::vector<Edge>> shortcuts;
    std::vector<NodeId> contracted_nodes;
};

class GraphContractor
{
public:
    GraphContractor(Graph&& graph) noexcept;

    auto fullyContractGraph() noexcept
        -> void;

    auto getGraph() noexcept
        -> Graph&;

private:
    auto contract(NodeId node) noexcept
        -> NodeContractionResult;

    auto contractSet(const std::vector<NodeId>& independent_set) noexcept
        -> IndependentSetContractionResult;


private:
    Graph graph_;
    Dijkstra dijkstra_;
    std::vector<bool> edge_deleted_;
};
