#include "config/config_loader.h"

#include <fstream>
#include <stdexcept>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

void from_json(const json& j, KinConfig& c) {
    j.at("control_cycle_us").get_to(c.control_cycle);

    j.at("speeds").at("max_speeds").get_to(c.speeds.max_vel);
    j.at("speeds").at("max_accels").get_to(c.speeds.max_acc);
    j.at("speeds").at("max_jerks").get_to(c.speeds.max_jerk);

    j.at("pulleyPoss").get_to(c.pulleyPoss);
}

KinConfig load_configs(const std::string& path) {
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("Failed to open config file: " + path);
    }

    json j;
    in >> j;

    return j.get<KinConfig>();
}