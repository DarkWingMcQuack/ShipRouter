#include <Dijkstra.hpp>
#include <Graph.hpp>
#include <GraphContractor.hpp>
#include <IndependentSetCalculator.hpp>
#include <fmt/core.h>
#include <iostream>
#include <numeric>


GraphContractor::GraphContractor(Graph&& graph) noexcept
    : graph_(std::move(graph)),
      dijkstra_(graph_) {}


auto GraphContractor::fullyContractGraph() noexcept
    -> void
{
    IndependentSetCalculator is_calculator{graph_};
    Level current_level = 0;

    while(is_calculator.hasAnotherSet()) {

        auto independent_set = is_calculator.calculateNextSet();
        auto contraction_result = contractSet(independent_set);

        graph_.rebuildWith(std::move(contraction_result.shortcuts),
                           contraction_result.contracted_nodes,
                           current_level++);

        fmt::print("contracted {} nodes with level: {}\n",
                   independent_set.size(),
                   current_level - 1);
    }
}


auto GraphContractor::getGraph() noexcept
    -> Graph&
{
    return graph_;
}

auto GraphContractor::contract(NodeId node) noexcept
    -> NodeContractionResult
{
    std::unordered_map<NodeId, std::vector<Edge>> shortcuts;
    std::int64_t counter = 0;

    const auto edge_ids = graph_.getEdgeIdsOf(node);

    for(auto outer_id : edge_ids) {
        const auto& outer_edge = graph_.getEdge(outer_id);
        const auto source = outer_edge.target;

        if(graph_.isAlreadyContracted(source)) {
            continue;
        }

        for(auto inner_id : edge_ids) {
            const auto& inner_edge = graph_.getEdge(inner_id);
            const auto target = inner_edge.target;
            const auto distance_over_u = outer_edge.distance + inner_edge.distance;


            if(graph_.isAlreadyContracted(target)) {
                continue;
            }

            if(dijkstra_.shortestPathSTGoesOverU(source, target, node, distance_over_u)) {
                auto distance = inner_edge.distance + outer_edge.distance;
                auto back_edge = graph_.getInverserEdgeId(outer_id).value();
                auto recurse_pair = std::pair{back_edge, inner_id};
                shortcuts[source].emplace_back(source, target, distance, recurse_pair);

                counter++;
            }
        }
    }

    auto edge_diff = counter - countObsoleteEdges(node);

    return NodeContractionResult{std::move(shortcuts),
                                 edge_diff};
}


auto GraphContractor::countObsoleteEdges(NodeId node) const noexcept
    -> std::int64_t
{
    auto edge_ids = graph_.getEdgeIdsOf(node);

    return std::count_if(std::begin(edge_ids),
                         std::end(edge_ids),
                         [this](auto id) {
                             const auto& edge = graph_.getEdge(id);
                             auto destination = edge.target;

                             return !graph_.isAlreadyContracted(destination);
                         })
        * 2;
}

namespace {

auto removeHalfOfTheResults(std::vector<NodeContractionResult>& results) noexcept
{
    if(results.size() < 10) {
        return;
    }

    std::sort(std::begin(results),
              std::end(results),
              [](const auto& lhs, const auto& rhs) {
                  return lhs.edge_diff > rhs.edge_diff;
              });

    results.resize(results.size() / 2);
}

} // namespace


auto GraphContractor::contractSet(const std::vector<NodeId>& independent_set) noexcept
    -> IndependentSetContractionResult
{
    std::vector<NodeContractionResult> contraction_results;
    std::transform(std::begin(independent_set),
                   std::end(independent_set),
                   std::back_inserter(contraction_results),
                   [this](auto n) {
                       return contract(n);
                   });

    removeHalfOfTheResults(contraction_results);

    std::unordered_map<NodeId, std::vector<Edge>> shortcuts;
    std::vector<NodeId> contracted_nodes;

    auto counter = 0;
    auto shortcuts_added = 0;

    for(auto&& [sc, edgediff] : contraction_results) {
        auto current_node = independent_set[counter++];

        for(auto&& [from, edges] : sc) {
            shortcuts[from].insert(std::end(shortcuts[from]),
                                   std::begin(edges),
                                   std::end(edges));
        }
        contracted_nodes.emplace_back(current_node);
        shortcuts_added += shortcuts[current_node].size();
    }

    return IndependentSetContractionResult{std::move(shortcuts),
                                           std::move(contracted_nodes)};
}
