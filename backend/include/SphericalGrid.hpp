#pragma once

#include <LatLng.hpp>
#include <Polygon.hpp>
#include <Utils.hpp>
#include <Vector3D.hpp>
#include <cmath>
#include <vector>


class SphericalGrid
{
public:
    SphericalGrid(std::size_t number_of_nodes) noexcept;

    auto getLats() const noexcept
        -> const std::vector<Latitude<Degree>>&;
    auto getLngs() const noexcept
        -> const std::vector<Longitude<Degree>>&;

    auto getLats() noexcept
        -> std::vector<Latitude<Degree>>&;
    auto getLngs() noexcept
        -> std::vector<Longitude<Degree>>&;

    auto sphericalToGrid(Latitude<Radian> theta, Longitude<Radian> phi) const noexcept
        -> std::pair<std::size_t, std::size_t>;

    auto gridToSpherical(size_t m, size_t n) const
        -> std::pair<Latitude<Degree>, Longitude<Degree>>;

    auto getRowNeighbours(size_t m, size_t n) const noexcept
        -> std::vector<std::pair<std::size_t, std::size_t>>;

    auto getUpperNeighbours(size_t m, size_t n) const noexcept
        -> std::vector<std::pair<std::size_t, std::size_t>>;

    auto getLowerNeighbours(size_t m, size_t n) const noexcept
        -> std::vector<std::pair<std::size_t, std::size_t>>;

    auto getNeighbours(size_t m, size_t n) const noexcept
        -> std::vector<size_t>;

    auto snapToNode(Latitude<Degree> lat, Longitude<Degree> lng) const
        -> size_t;

    auto getNeighbours(std::size_t id) const noexcept
        -> std::vector<size_t>;

    auto gridToID(size_t m, size_t n) const noexcept
        -> size_t;

    auto idToLatLng(NodeId n) const noexcept
        -> std::pair<Latitude<Degree>, Longitude<Degree>>;

    //only valid if from and to are neigbours
    auto distanceBetween(NodeId from, NodeId to) const noexcept
        -> Distance;

    auto isValidId(NodeId from) const noexcept
        -> bool;

    auto idToGrid(size_t id) const noexcept
        -> std::pair<size_t, size_t>;

    auto indexIsWater(std::size_t idx) const noexcept
        -> bool;

    auto indexIsLand(std::size_t idx) const noexcept
        -> bool;

    auto filter(const std::vector<Polygon>& polygons) noexcept
        -> void;

    auto size() const noexcept
        -> std::size_t;

private:
    friend class Graph;
    double a_;
    size_t n_rows_;
    double d_phi_;
    std::vector<size_t> first_index_of_;
    std::vector<Latitude<Degree>> lats_;
    std::vector<Longitude<Degree>> lngs_;
    std::vector<bool> is_water_;

    auto nCols(size_t m) const -> std::size_t;
    auto nCols(double theta) const -> std::size_t;
    auto calcTheta(size_t m) const -> double;
    auto calcPhi(size_t m, size_t n) const -> double;
};
