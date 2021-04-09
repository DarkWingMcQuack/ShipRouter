#include <Dijkstra.hpp>
#include <Graph.hpp>
#include <GraphContractor.hpp>
#include <IndependentSetCalculator.hpp>
#include <algorithm>
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

        fmt::print("contracted {} nodes in level: {}\n",
                   independent_set.size(),
                   current_level - 1);

        fmt::print("new graph has {} edges\n", graph_.numberOfEdges());
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

        if(graph_.isAlreadyContracted(source) or source == node) {
            continue;
        }

        for(auto inner_id : edge_ids) {

            const auto& inner_edge = graph_.getEdge(inner_id);
            const auto target = inner_edge.target;
            const auto distance_over_u = outer_edge.distance + inner_edge.distance;


            if(graph_.isAlreadyContracted(target) or target == node) {
                continue;
            }

            if(dijkstra_.shortestPathSTGoesOverU(source, target, node, distance_over_u)) {

                auto first_back_edge = graph_.getInverserEdgeId(outer_id);
                // auto second_back_edge = graph_.getInverserEdgeId(inner_id);

                auto first_recurse_pair = std::pair{first_back_edge, inner_id};
                // auto second_recurse_pair = std::pair{second_back_edge, outer_id};

                shortcuts[source].emplace_back(source, target, distance_over_u, first_recurse_pair);
                // shortcuts[target].emplace_back(target, source, distance_over_u, second_recurse_pair);

                // counter += 2;
                counter++;
            }
        }
    }

    auto edge_diff = counter - countObsoleteEdges(node);

    return NodeContractionResult{node,
                                 std::move(shortcuts),
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

auto removeBiggerThanMedian(std::vector<NodeContractionResult>& results) noexcept
{
    auto n = results.size() / 2;
    std::nth_element(std::begin(results),
                     std::begin(results) + n,
                     std::end(results),
                     [](const auto& lhs, const auto& rhs) {
                         return lhs.edge_diff < rhs.edge_diff;
                     });

    auto median = results[n].edge_diff;

    results.erase(
        std::remove_if(std::begin(results),
                       std::end(results),
                       [&](const auto& result) {
                           return result.edge_diff > median;
                       }),
        std::end(results));
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

    removeBiggerThanMedian(contraction_results);

    std::unordered_map<NodeId, std::vector<Edge>> shortcuts;
    std::vector<NodeId> contracted_nodes;

    for(auto&& [current_node, sc, edgediff] : contraction_results) {
        for(auto&& [from, edges] : sc) {
            shortcuts[from].insert(std::end(shortcuts[from]),
                                   std::begin(edges),
                                   std::end(edges));
        }
        contracted_nodes.emplace_back(current_node);
    }

    return IndependentSetContractionResult{std::move(shortcuts),
                                           std::move(contracted_nodes)};
}
