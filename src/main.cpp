#include <PBFExtractor.hpp>
#include <fmt/core.h>

auto main() -> int
{
    auto [nodes, coastlines] = parsePBFFile("../data/antarctica-latest.osm.pbf");
	fmt::print("starting calculation of polygons...\n");
    auto polygons = calculatePolygons(std::move(coastlines),
                                      std::move(nodes));

    fmt::print("{}\n", polygons.size());
}
