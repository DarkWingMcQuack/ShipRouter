#include <Graph.hpp>
#include <IndependentSetCalculator.hpp>
#include <OSMNode.hpp>
#include <numeric>
#include <vector>


IndependentSetCalculator::IndependentSetCalculator(const Graph& graph) noexcept
    : graph_(graph),
      nodes_(graph_.size()),
      visited_(graph_.size(), false)
{
    std::iota(std::begin(nodes_),
              std::end(nodes_),
              0);
}

auto IndependentSetCalculator::hasAnotherSet() const noexcept
    -> bool
{
    return std::any_of(std::begin(nodes_),
                       std::end(nodes_),
                       [&](auto node) {
                           return !graph_.isAlreadyContracted(node);
                       });
}

auto IndependentSetCalculator::calculateNextSet() noexcept
    -> std::vector<NodeId>
{
    std::vector<NodeId> independent_set;

    sortNodesByDegree();

    for(auto n : nodes_) {
        if(visited_[n] or graph_.isAlreadyContracted(n)) {
            continue;
        }

        visited_[n] = true;
        independent_set.emplace_back(n);
        touched_.emplace_back(n);

        for(auto edge_id : graph_.getEdgeIdsOf(n)) {
            const auto& edge = graph_.getEdge(edge_id);
            const auto& neig = edge.target;

            if(!visited_[neig]) {
                touched_.emplace_back(neig);
				visited_[neig] = true;
            }
        }
    }

    return independent_set;
}

auto IndependentSetCalculator::sortNodesByDegree() noexcept
    -> void
{
    std::sort(std::begin(nodes_),
              std::end(nodes_),
              [this](auto lhs, auto rhs) {
                  return graph_.getDegree(lhs) < graph_.getDegree(rhs);
              });
}

auto IndependentSetCalculator::cleanup() noexcept
    -> void
{
    for(auto n : touched_) {
        visited_[n] = false;
    }

    touched_.clear();
}
