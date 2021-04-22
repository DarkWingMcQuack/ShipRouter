#include <Dijkstra.hpp>
#include <Environment.hpp>
#include <GraphContractor.hpp>
#include <PBFExtractor.hpp>
#include <ServiceManager.hpp>
#include <SphericalGrid.hpp>
#include <Timer.hpp>
#include <Vector3D.hpp>
#include <csignal>
#include <cstdint>
#include <execution>
#include <fmt/core.h>
#include <fmt/ranges.h>

static std::condition_variable condition;
static std::mutex mutex;

static auto handleUserInterrupt(int signal) noexcept
    -> void
{
    if(signal == SIGINT) {
        condition.notify_one();
    }
}

static auto waitForUserInterrupt() noexcept
    -> void
{
    std::unique_lock lock{mutex};
    condition.wait(lock);
    std::cout << "user has signaled to interrupt the program..." << std::endl;
    lock.unlock();
}

static auto getEnvironment() noexcept
    -> Environment
{
    auto environment_opt = loadEnv();
    if(!environment_opt) {
        std::cout << "the environment variables could not be understood" << std::endl;
        std::cout << "using default values" << std::endl;

        return Environment{9090,
                           "../data/antarctica-latest.osm.pbf",
                           10};
    }

    return environment_opt.value();
}

auto generateBenchmarkData(const Graph& graph, std::size_t size) noexcept
    -> std::vector<std::pair<NodeId, NodeId>>
{

    std::vector<std::pair<NodeId, NodeId>> benchmark_data;
    Dijkstra d{graph};
    while(benchmark_data.size() != size) {
        auto source = rand() % graph.size();
        auto target = rand() % graph.size();

        if(d.findDistance(source, target) != UNREACHABLE) {
            benchmark_data.emplace_back(source, target);
        }
    }

    return benchmark_data;
}


auto main() -> int
{
    auto environment = getEnvironment();

    utils::Timer t;

    std::cout << "Parsing pbf file..." << std::endl;
    auto [nodes, coastlines] = parsePBFFile(environment.getDataFile());
    std::cout << "parsed in " << t.elapsed() << "ms" << std::endl;


    std::cout << "calculating polygons..." << std::endl;

    t.reset();
    auto polygons = calculatePolygons(std::move(coastlines),
                                      std::move(nodes));
    std::cout << "calculated in " << t.elapsed() << "ms" << std::endl;

    std::cout << "building the grid..." << std::endl;
    SphericalGrid grid{environment.getNumberOfSphereNodes()};

    std::cout << "filtering land nodes..." << std::endl;
    t.reset();
    grid.filter(polygons);
    std::cout << "filtered in " << t.elapsed() << "ms" << std::endl;

    Graph pre_graph{std::move(grid)};

    const auto NUMBER_OF_BENCHMARKS = 10000ul;

    auto benchmark_data = generateBenchmarkData(pre_graph, NUMBER_OF_BENCHMARKS);

    std::vector<Distance> dijkstra_distance(NUMBER_OF_BENCHMARKS);

    Dijkstra d{pre_graph};

    t.reset();
    for(int i = 0; i < NUMBER_OF_BENCHMARKS; i++) {
        dijkstra_distance[i] = d.findDistance(benchmark_data[i].first,
                                              benchmark_data[i].second);
    }

    double dij_time = t.elapsed() / static_cast<double>(NUMBER_OF_BENCHMARKS);

    fmt::print("normal dijkstra took an average  of {}ms per call\n", dij_time);
    fmt::print("with an average of {} q_pops per call\n", d.getAverageQPopsPerQuery());

    std::cout << "contracting graph ..." << std::endl;

    t.reset();
    GraphContractor contractor{std::move(pre_graph)};
    contractor.fullyContractGraph();

    std::cout << "contracted in " << t.elapsed() << "ms" << std::endl;

    auto graph = std::move(contractor.getGraph());

    std::vector<Distance> ch_distance(NUMBER_OF_BENCHMARKS);
    t.reset();

    CHDijkstra ch{graph};
    for(int i = 0; i < NUMBER_OF_BENCHMARKS; i++) {
        ch_distance[i] = ch.findDistance(benchmark_data[i].first,
                                         benchmark_data[i].second);
    }

    double ch_time = t.elapsed() / static_cast<double>(NUMBER_OF_BENCHMARKS);

    fmt::print("ch dijkstra took an average of {}ms per call\n", ch_time);
    fmt::print("with an average of {} q_pops per call\n", ch.getAverageQPopsPerQuery());

    //handle sigint such that the user can stop the server
    std::signal(SIGINT, handleUserInterrupt);
    std::signal(SIGPIPE, [](int /**/) {});

    ServiceManager manager{Pistache::Address{Pistache::IP::any(),
                                             environment.getPort()},
                           graph};
    try {
        fmt::print("started server, listening at: {}",
                   environment.getPort());
        std::cout << std::endl;

        manager.serveThreaded();

        waitForUserInterrupt();

        manager.shutdown();

        std::cout << "shutting down server" << std::endl;

    } catch(const std::exception& e) {
        fmt::print("caught exception: {}\n", e.what());
    }
}
