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

    std::cout << "contracting graph ..." << std::endl;

    t.reset();
    GraphContractor contractor{std::move(pre_graph)};
    contractor.fullyContractGraph();

    std::cout << "contracted in " << t.elapsed() << "ms" << std::endl;

    auto graph = std::move(contractor.getGraph());

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
