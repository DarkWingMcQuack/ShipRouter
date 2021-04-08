#include <CHDijkstra.hpp>
#include <Dijkstra.hpp>
#include <Graph.hpp>
#include <iostream>
#include <queue>


CHDijkstra::CHDijkstra(const Graph& graph) noexcept
    : graph_(graph),
      forward_distances_(graph_.size(), UNREACHABLE),
      forward_best_ingoing_(graph_.size(), EDGE_NOT_SET),
      forward_already_settled_(graph_.size(), false),
      backward_distances_(graph_.size(), UNREACHABLE),
      backward_best_ingoing_(graph_.size(), EDGE_NOT_SET),
      backward_already_settled_(graph_.size(), false) {}


auto CHDijkstra::findDistance(NodeId source, NodeId target) noexcept
    -> Distance
{
    fillForwardInfo(source);

    fillBackwardInfo(target);

    auto top_node_opt = findShortestPathCommonNode();

    if(!top_node_opt) {
        return UNREACHABLE;
    }

    auto top_node = top_node_opt.value();

    return forward_distances_[top_node] + backward_distances_[top_node];
}

auto CHDijkstra::findRoute(NodeId source, NodeId target) noexcept
    -> std::optional<std::pair<Path, Distance>>
{
    fillForwardInfo(source);
    fillBackwardInfo(target);

    auto top_node_opt = findShortestPathCommonNode();

    if(!top_node_opt) {
        return std::nullopt;
    }


    auto top_node = top_node_opt.value();

    auto distance = forward_distances_[top_node]
        + backward_distances_[top_node];

    auto path = extractPath(source, top_node, target);

    return std::pair{std::move(path),
                     distance};
}

auto CHDijkstra::extractPath(NodeId source,
                             NodeId top_node,
                             NodeId target) const noexcept
    -> Path
{
    if(source == target) {
        return Path{source};
    }

    auto source_to_top_wrapped = extractSourcePathWrapped(source, top_node);
    auto top_to_target_wrapped = extractTargetPathWrapped(target, top_node);

    auto source_to_top = unwrap(std::move(source_to_top_wrapped),
                                source,
                                top_node,
                                target);
    auto top_to_target = unwrap(std::move(top_to_target_wrapped),
                                source,
                                top_node,
                                target);

    std::reverse(std::begin(top_to_target),
                 std::end(top_to_target));


    if(source == top_node or target == top_node) {
        return concat(Path{source},
                      std::move(source_to_top),
                      std::move(top_to_target),
                      Path{target});
    }


    return concat(Path{source},
                  std::move(source_to_top),
                  Path{top_node},
                  std::move(top_to_target),
                  Path{target});
}

auto CHDijkstra::extractSourcePathWrapped(NodeId source,
                                          NodeId top_node) const noexcept
    -> std::vector<EdgeId>
{
    std::vector<EdgeId> ids;
    while(top_node != source) {
        auto id = forward_best_ingoing_[top_node];
        ids.emplace_back(id);

        const auto& edge = graph_.getEdge(id);
        top_node = edge.source;
    }

    return ids;
}

auto CHDijkstra::extractTargetPathWrapped(NodeId target,
                                          NodeId top_node) const noexcept
    -> std::vector<EdgeId>
{
    std::vector<EdgeId> ids;
    while(top_node != target) {
        auto id = backward_best_ingoing_[top_node];
        ids.emplace_back(id);

        const auto& edge = graph_.getEdge(id);

        top_node = edge.source;
    }

    return ids;
}

auto CHDijkstra::unwrap(std::vector<EdgeId> ids,
                        NodeId source,
                        NodeId top_node,
                        NodeId target) const noexcept
    -> Path
{
    Path path;
    while(!ids.empty()) {
        auto next = ids.back();
        ids.pop_back();

        const auto& edge = graph_.getEdge(next);
        auto node = edge.target;

        if(edge.shortcut_for) {
            auto [left, right] = edge.shortcut_for.value();

            ids.emplace_back(right);
            ids.emplace_back(left);
        } else if(node != source
                  and node != top_node
                  and node != target) {
            path.emplace_back(node);
        }
    }

    return path;
}

auto CHDijkstra::resetForward() noexcept
    -> void
{
    for(auto n : forward_touched_) {
        forward_distances_[n] = UNREACHABLE;
        forward_best_ingoing_[n] = EDGE_NOT_SET;
        forward_already_settled_[n] = false;
    }
    forward_settled_.clear();
    forward_touched_.clear();
}

auto CHDijkstra::resetBackward() noexcept
    -> void
{
    for(auto n : backward_touched_) {
        backward_distances_[n] = UNREACHABLE;
        backward_best_ingoing_[n] = EDGE_NOT_SET;
        backward_already_settled_[n] = false;
    }
    backward_settled_.clear();
    backward_touched_.clear();
}

auto CHDijkstra::fillForwardInfo(NodeId source) noexcept
    -> void
{
    if(last_source_ == source) {
        return;
    }

    resetForward();
    last_source_ = source;
    forward_touched_.emplace_back(source);
    forward_distances_[source] = 0;

    DijkstraQueue heap;
    heap.emplace(source, 0);

    while(!heap.empty()) {
        auto [current_node, cost_to_current] = heap.top();
        heap.pop();

        if(forward_already_settled_[current_node]) {
            continue;
        }

        forward_settled_.emplace_back(current_node);
        forward_already_settled_[current_node] = true;

        auto edge_ids = graph_.getEdgeIdsOf(current_node);


        for(auto id : edge_ids) {
            const auto& edge = graph_.getEdge(id);
            auto neig = edge.target;

            if(graph_.getLevelOf(current_node) >= graph_.getLevelOf(neig)) {
                continue;
            }

            auto new_dist = edge.distance + cost_to_current;

            if(new_dist < forward_distances_[neig]) {
                heap.emplace(neig, new_dist);
                forward_distances_[neig] = new_dist;
                forward_touched_.emplace_back(neig);
                forward_best_ingoing_[neig] = id;
            }
        }
    }

    std::sort(std::begin(forward_settled_),
              std::end(forward_settled_));
}

auto CHDijkstra::fillBackwardInfo(NodeId target) noexcept
    -> void
{
    if(last_target_ == target) {
        return;
    }

    resetBackward();
    last_target_ = target;
    backward_touched_.emplace_back(target);
    backward_distances_[target] = 0;

    DijkstraQueue heap;
    heap.emplace(target, 0);

    while(!heap.empty()) {
        auto [current_node, cost_to_current] = heap.top();
        heap.pop();

        if(backward_already_settled_[current_node]) {
            continue;
        }

        backward_settled_.emplace_back(current_node);
        backward_already_settled_[current_node] = true;

        auto edge_ids = graph_.getEdgeIdsOf(current_node);

        for(auto id : edge_ids) {
            const auto& edge = graph_.getEdge(id);
            auto neig = edge.target;

            if(graph_.getLevelOf(current_node) >= graph_.getLevelOf(neig)) {
                continue;
            }

            auto new_dist = edge.distance + cost_to_current;

            if(new_dist < backward_distances_[neig]) {
                heap.emplace(neig, new_dist);
                backward_distances_[neig] = new_dist;
                backward_touched_.emplace_back(neig);
                backward_best_ingoing_[neig] = id;
            }
        }
    }

    std::sort(std::begin(backward_settled_),
              std::end(backward_settled_));
}

auto CHDijkstra::findShortestPathCommonNode() noexcept
    -> std::optional<NodeId>
{
    if(forward_settled_.empty() or backward_settled_.empty()) {
        return std::nullopt;
    }

    auto best_node = forward_settled_[0];
    auto best_dist = UNREACHABLE;

    auto forward_idx = 0ul;
    auto backward_idx = 0ul;

    while(forward_idx < forward_settled_.size()
          and backward_idx < backward_settled_.size()) {

        if(forward_settled_[forward_idx] < backward_settled_[backward_idx]) {
            forward_idx++;
            continue;
        }

        if(forward_settled_[forward_idx] > backward_settled_[backward_idx]) {
            backward_idx++;
            continue;
        }

        const auto common = forward_settled_[forward_idx];
        const auto new_dist = forward_distances_[common] + backward_distances_[common];

        if(new_dist < best_dist) {
            best_dist = new_dist;
            best_node = common;
        }

        forward_idx++;
        backward_idx++;
    }

    if(best_dist == UNREACHABLE) {
        return std::nullopt;
    }

    return best_node;
}
