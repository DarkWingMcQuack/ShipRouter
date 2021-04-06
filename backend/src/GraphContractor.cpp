#include <Dijkstra.hpp>
#include <Graph.hpp>
#include <GraphContractor.hpp>
#include <IndependentSetCalculator.hpp>
#include <fmt/core.h>
#include <iostream>
#include <numeric>


GraphContractor::GraphContractor(Graph&& graph) noexcept
    : graph_(std::move(graph)),
      dijkstra_(graph_)
{
    fmt::print("constructed graph contractor\n");
    std::cout << std::flush;
}


auto GraphContractor::fullyContractGraph() noexcept
    -> void
{
    fmt::print("initilaizing IS calculator...\n");
    std::cout << std::flush;
    IndependentSetCalculator is_calculator{graph_};

    fmt::print("done initilaizing IS calculator...\n");
    std::cout << std::flush;

    Level current_level = 0;
    while(is_calculator.hasAnotherSet()) {
        fmt::print("calculate independed set\n");
        std::cout << std::flush;
        auto independent_set = is_calculator.calculateNextSet();

        fmt::print("contract independed set\n");
        std::cout << std::flush;
        auto contraction_result = contractSet(independent_set);

        fmt::print("rebuild graph\n");
        std::cout << std::flush;
        graph_.rebuildWith(std::move(contraction_result.shortcuts),
                           contraction_result.contracted_nodes,
                           current_level++);

        fmt::print("contracted {} new nodes with level: {}\n",
                   independent_set.size(),
                   current_level - 1);
        std::cout << std::flush;

        fmt::print("graph has: {} nodes and {} edges\n", graph_.size(), graph_.edges_.size());
        std::cout << std::flush;
    }

    fmt::print("done contracting graph\n");
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

            if(source == target or graph_.isAlreadyContracted(target)) {
                continue;
            }

            if(dijkstra_.shortestPathSTGoesOverU(source, target, node)) {
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

auto calculateAverageEdgeDiff(const std::vector<NodeContractionResult>& results) noexcept
    -> double
{
    auto sum = std::accumulate(std::cbegin(results),
                               std::cend(results),
                               0.,
                               [](auto current, const auto& next) {
                                   return current + next.edge_diff;
                               });

    return sum / results.size();
}


auto removeResultsWithEdgeDiffSmallerThan(std::vector<NodeContractionResult>& results,
                                          double min_edge_diff) noexcept
{
    results.erase(
        std::remove_if(std::begin(results),
                       std::end(results),
                       [&](auto r) {
                           return r.edge_diff < min_edge_diff;
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

    auto average_edge_diff = calculateAverageEdgeDiff(contraction_results);

    removeResultsWithEdgeDiffSmallerThan(contraction_results,
                                         average_edge_diff);

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
