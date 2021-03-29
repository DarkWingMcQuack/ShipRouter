#include <Dijkstra.hpp>
#include <Graph.hpp>
#include <GraphContractor.hpp>
#include <IndependentSetCalculator.hpp>
#include <numeric>


GraphContractor::GraphContractor(Graph&& graph) noexcept
    : graph_(std::move(graph)),
      dijkstra_(graph_) {}


auto GraphContractor::fullyContractGraph() noexcept
    -> void
{
    IndependentSetCalculator is_calculator{graph_};

    Level current_level = 1;
    while(is_calculator.hasAnotherSet()) {
        auto independent_set = is_calculator.calculateNextSet();
        auto contraction_result = contractSet(independent_set);

        graph_.rebuildWith(std::move(contraction_result.shortcuts),
                           contraction_result.contracted_nodes,
                           current_level++);
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
        const auto source = outer_edge.target_;

        for(auto inner_id : edge_ids) {
            const auto& inner_edge = graph_.getEdge(inner_id);
            const auto target = inner_edge.target_;

            if(source == target) {
                continue;
            }

            if(dijkstra_.shortestPathSTGoesOverU(source, target, node)) {
                auto distance = inner_edge.distance_ + outer_edge.distance_;
                auto recurse_pair = std::pair{outer_id, inner_id};
                shortcuts[source].emplace_back(target, distance, recurse_pair);
                counter++;
            }
        }
    }

    std::int64_t edge_diff = counter - 2 * edge_ids.size();

    return NodeContractionResult{std::move(shortcuts),
                                 edge_diff};
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
