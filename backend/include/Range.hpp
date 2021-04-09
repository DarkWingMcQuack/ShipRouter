#pragma once
#include <iterator>
#include <type_traits>

namespace utils::impl {

template<typename Counter>
struct RangeIterator
{
    // iterator traits
    using difference_type = Counter;
    using value_type = Counter;
    using pointer = Counter;
    using reference = Counter;
    using iterator_category = std::random_access_iterator_tag;

    explicit constexpr RangeIterator(Counter current) noexcept
        : current_(current),
          start_(current)
    {
    }

    constexpr auto operator++() noexcept -> RangeIterator&
    {
        ++current_;
        return *this;
    }

    constexpr auto operator--() noexcept -> RangeIterator&
    {
        --current_;
        return *this;
    }

    constexpr auto operator++(int) noexcept -> RangeIterator
    {

        constexpr auto ret = *this;
        (*this)++;
        return ret;
    }

    constexpr auto operator--(int) noexcept -> RangeIterator
    {
        constexpr auto ret = *this;
        (*this)--;
        return ret;
    }

    constexpr auto operator+(RangeIterator const& other) const noexcept
        -> difference_type
    {
        return current_ + other.current_;
    }

    constexpr auto operator+(difference_type other) const noexcept
        -> RangeIterator
    {
        return RangeIterator{current_ + other};
    }

    constexpr auto operator-(RangeIterator const& other) const noexcept
        -> difference_type
    {
        return current_ - other.current_;
    }

    constexpr auto operator-(difference_type other) const noexcept
        -> RangeIterator
    {
        return RangeIterator{current_ - other};
    }

    constexpr auto operator+=(RangeIterator const& other) noexcept
        -> RangeIterator&
    {
        current_ += other.current_;
        return *this;
    }

    constexpr auto operator-=(RangeIterator const& other) noexcept
        -> RangeIterator&
    {
        current_ -= other.current_;
        return *this;
    }

    constexpr auto operator+=(difference_type other) noexcept
        -> RangeIterator&
    {
        current_ += other;
        return *this;
    }

    constexpr auto operator-=(difference_type other) noexcept
        -> RangeIterator&
    {
        current_ -= other;
        return *this;
    }


    constexpr auto operator==(RangeIterator const& other) const noexcept
        -> bool
    {
        return current_ == other.current_;
    }

    constexpr auto operator!=(RangeIterator const& other) const noexcept
        -> bool
    {
        return !(*this == other);
    }

    constexpr auto operator<=(RangeIterator const& other) const noexcept
        -> bool
    {
        return current_ <= other.current_;
    }

    constexpr auto operator>=(RangeIterator const& other) const noexcept
        -> bool
    {
        return current_ >= other.current_;
    }

    constexpr auto operator<(RangeIterator const& other) const noexcept
        -> bool
    {
        return current_ < other.current_;
    }

    constexpr auto operator>(RangeIterator const& other) const noexcept
        -> bool
    {
        return current_ > other.current_;
    }

    constexpr auto operator*() const noexcept
        -> Counter
    {
        return current_;
    }

    constexpr auto operator->() const noexcept
        -> Counter
    {
        return current_;
    }

    constexpr auto operator[](int idx) const noexcept
        -> Counter
    {
        return start_ + idx;
    }

private:
    Counter current_;
    Counter start_;
};

template<typename Counter, bool forward>
struct RangeWrapper
{
    using ValueType = typename std::remove_cv<Counter>::type;
    using IteratorBase = RangeIterator<ValueType>;
    using Iterator = typename std::conditional<forward,
                                               IteratorBase,
                                               std::reverse_iterator<IteratorBase>>::type;

    constexpr RangeWrapper(Counter from, Counter to) noexcept
        : begin_(from),
          end_(to)
    {
    }

    constexpr auto begin() const noexcept
        -> Iterator
    {
        return Iterator(IteratorBase(begin_));
    }
    constexpr auto end() const noexcept
        -> Iterator
    {
        return Iterator(IteratorBase(end_));
    }

    constexpr auto operator[](int idx) const noexcept
        -> Counter
    {
        return begin_ + idx;
    }

    constexpr auto size() const noexcept
        -> std::size_t
    {
        return end_ - begin_;
    }

private:
    ValueType const begin_;
    ValueType const end_;
};

} // namespace utils::impl


namespace utils {

template<typename Counter>
constexpr auto range(Counter to) noexcept
    -> impl::RangeWrapper<Counter, true>
{
    return {0, to};
}

template<typename Counter>
constexpr auto range(Counter from, Counter to) noexcept
    -> impl::RangeWrapper<Counter, true>
{
    return {from, to};
}

template<typename Counter>
constexpr auto reverseRange(Counter to) noexcept
    -> impl::RangeWrapper<Counter, false>
{
    return {to, 0};
}

template<typename Counter>
constexpr auto reverseRange(Counter from, Counter to) noexcept
    -> impl::RangeWrapper<Counter, false>
{
    return {to, from};
}

} // namespace utils
