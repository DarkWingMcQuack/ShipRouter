#pragma once

#include <optional>
#include <tuple>
#include <utility>

class Environment
{
public:
    Environment(std::uint16_t port,
                std::string data_file,
                std::uint64_t number_of_nodes)
        : port_(port),
          data_file_(std::move(data_file)),
          number_of_sphere_nodes_(number_of_nodes) {}

    auto getPort() const
        -> std::uint16_t
    {
        return port_;
    }

    auto getNumberOfSphereNodes() const
        -> std::uint64_t
    {
        return number_of_sphere_nodes_;
    }

    auto getDataFile() const
        -> std::string_view
    {
        return data_file_;
    }

private:
    std::uint16_t port_;
    std::string data_file_;
    std::uint64_t number_of_sphere_nodes_;
};


inline auto getEnv(std::string_view env_var) noexcept
    -> std::optional<std::string>
{
    const auto* raw = std::getenv(env_var.data());
    if(raw == nullptr) {
        return std::nullopt;
    }

    auto str = std::string(raw);
    if(str.empty()) {
        return std::nullopt;
    }
    return str;
}

inline auto loadEnv()
    -> std::optional<Environment>
{
    auto port_str_opt = getEnv("PORT");
    auto datafile_str_opt = getEnv("DATAFILE");
    auto nodes_on_sphere_str_opt = getEnv("NUMBER_OF_SPHERE_NODES");

    if(!port_str_opt or !datafile_str_opt or !nodes_on_sphere_str_opt) {
        return std::nullopt;
    }

    auto port_str = std::move(port_str_opt.value());
    auto datafile_str = std::move(datafile_str_opt.value());
    auto nodes_on_sphere_str = std::move(nodes_on_sphere_str_opt.value());

    try {
        auto port = std::stoi(port_str);
        auto nodes_on_sphere = std::stoul(nodes_on_sphere_str);

        return Environment{static_cast<std::uint16_t>(port),
                           datafile_str,
                           nodes_on_sphere};
    } catch(...) {
        return std::nullopt;
    }
}
