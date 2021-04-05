#pragma once

#include <Constants.hpp>
#include <functional>

template<class T>
using Ref = std::reference_wrapper<T>;

template<class T>
using CRef = std::reference_wrapper<const T>;

using Tags = std::unordered_map<std::string, std::string>;


template<class Head0, class Head1, class... Tail>
constexpr auto concat(Head0&& head0, Head1&& head1, Tail&&... tail) noexcept
{
    if constexpr(sizeof...(tail) == 0) {
        head0.insert(std::end(head0),
                     std::begin(head1),
                     std::end(head1));
        return std::forward<Head0>(head0);
    } else {
        return concat(
            concat(std::forward<Head0>(head0),
                   std::forward<Head1>(head1)),
            std::forward<Tail>(tail)...);
    }
}


using Distance = std::uint64_t;
using NodeId = std::uint64_t;
using EdgeId = std::uint64_t;
using Path = std::vector<NodeId>;
using Level = std::size_t;

constexpr static inline auto UNREACHABLE = std::numeric_limits<Distance>::max();
constexpr static inline auto NON_EXISTENT = std::numeric_limits<NodeId>::max();
constexpr static inline auto NODE_NOT_SET = std::numeric_limits<NodeId>::max() - 1;
constexpr static inline auto LEVEL_NOT_SET = std::numeric_limits<Level>::max();
constexpr static inline auto EDGE_NOT_SET = std::numeric_limits<EdgeId>::max();
