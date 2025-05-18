#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <iomanip>
#include <optional>
#include "json.hpp"

using json = nlohmann::json;

// ------------------ Utility Functions ------------------

double signed_volume(const std::vector<double>& a,
                     const std::vector<double>& b,
                     const std::vector<double>& c,
                     const std::vector<double>& d) {
    double ax = a[0] - d[0], ay = a[1] - d[1], az = a[2] - d[2];
    double bx = b[0] - d[0], by = b[1] - d[1], bz = b[2] - d[2];
    double cx = c[0] - d[0], cy = c[1] - d[1], cz = c[2] - d[2];
    return (ax * (by * cz - bz * cy) - ay * (bx * cz - bz * cx) + az * (bx * cy - by * cx)) / 6.0;
}

std::vector<double> get_point(const json& j, int idx) {
    std::vector<int> vi = j["vertices"][idx];
    double x = vi[0] * j["transform"]["scale"][0].get<double>() + j["transform"]["translate"][0].get<double>();
    double y = vi[1] * j["transform"]["scale"][1].get<double>() + j["transform"]["translate"][1].get<double>();
    double z = vi[2] * j["transform"]["scale"][2].get<double>() + j["transform"]["translate"][2].get<double>();
    return {x, y, z};
}

std::optional<std::vector<double>> compute_newell_normal(const std::vector<std::vector<double>>& pts) {
    if (pts.size() < 3) return std::nullopt;
    double nx = 0, ny = 0, nz = 0;
    for (size_t i = 0; i < pts.size(); ++i) {
        const auto& current = pts[i];
        const auto& next = pts[(i + 1) % pts.size()];
        nx += (current[1] - next[1]) * (current[2] + next[2]);
        ny += (current[2] - next[2]) * (current[0] + next[0]);
        nz += (current[0] - next[0]) * (current[1] + next[1]);
    }
    double norm = std::sqrt(nx * nx + ny * ny + nz * nz);
    if (norm == 0) return std::nullopt;
    return std::vector<double>{nx / norm, ny / norm, nz / norm};
}

double azimuth_from_normal(const std::vector<double>& normal) {
    double x = normal[0], y = normal[1];
    double angle_rad = std::atan2(x, y);  // measured from North
    double angle_deg = angle_rad * 180.0 / M_PI;
    return (angle_deg < 0) ? angle_deg + 360.0 : angle_deg;
}

// ------------------ Introspection Helpers ------------------

int get_no_roof_surfaces(json &j) {
    int total = 0;
    for (auto& co : j["CityObjects"].items()) {
        for (auto& g : co.value()["geometry"]) {
            if (g["type"] == "Solid") {
                for (auto& shell : g["semantics"]["values"]) {
                    for (auto& s : shell) {
                        if (g["semantics"]["surfaces"][s.get<int>()]["type"].get<std::string>() == "RoofSurface") {
                            total++;
                        }
                    }
                }
            }
        }
    }
    return total;
}

void visit_roofsurfaces(json &j) {
    for (auto& co : j["CityObjects"].items()) {
        for (auto& g : co.value()["geometry"]) {
            if (g["type"] == "Solid") {
                for (int i = 0; i < g["boundaries"].size(); i++) {
                    for (int j = 0; j < g["boundaries"][i].size(); j++) {
                        int sem_index = g["semantics"]["values"][i][j];
                        if (g["semantics"]["surfaces"][sem_index]["type"].get<std::string>() == "RoofSurface") {
                            std::cout << "RoofSurface: " << g["boundaries"][i][j] << std::endl;
                        }
                    }
                }
            }
        }
    }
}

void list_all_vertices(const json& j) {
    for (const auto& co : j["CityObjects"].items()) {
        std::cout << "= CityObject: " << co.key() << std::endl;
        for (const auto& g : co.value()["geometry"]) {
            if (g["type"] == "Solid") {
                for (const auto& shell : g["boundaries"]) {
                    for (const auto& surface : shell) {
                        for (const auto& ring : surface) {
                            std::cout << "---" << std::endl;
                            for (const auto& v : ring) {
                                std::vector<int> vi = j["vertices"][v.get<int>()];
                                double x = vi[0] * j["transform"]["scale"][0].get<double>() + j["transform"]["translate"][0].get<double>();
                                double y = vi[1] * j["transform"]["scale"][1].get<double>() + j["transform"]["translate"][1].get<double>();
                                double z = vi[2] * j["transform"]["scale"][2].get<double>() + j["transform"]["translate"][2].get<double>();
                                std::cout << std::setprecision(2) << std::fixed << v << " (" << x << ", " << y << ", " << z << ")" << std::endl;
                            }
                        }
                    }
                }
            }
        }
    }
}

// ------------------ Main Processing ------------------

int main(int argc, const char* argv[]) {
    const char* filename = (argc > 1) ? argv[1] : "nextbk_2b.city.json";
    //const char* filename = (argc > 1) ? argv[1] : "cube.city.json";
    std::cout << "Processing: " << filename << std::endl;
    std::ifstream input(filename);
    json j;
    input >> j;
    input.close();

    std::cout << "Total RoofSurface: " << get_no_roof_surfaces(j) << std::endl;
    list_all_vertices(j);
    visit_roofsurfaces(j);

    std::vector<double> fake_point = {0.0, 0.0, -1000.0};
    int nobuildings = 0;

    for (auto& [id, co] : j["CityObjects"].items()) {
        if (co["type"] != "Building") continue;

        double total_volume = 0.0;
        int total_tetrahedra = 0;
        std::vector<std::vector<double>> roof_normals;

        std::cout << "Decomposing building: " << id << std::endl;

        if (!co.contains("children")) continue;
        for (const auto& child_id : co["children"]) {
            const auto& child = j["CityObjects"][child_id.get<std::string>()];
            for (const auto& geom : child["geometry"]) {
                if (geom["type"] != "Solid" || geom["lod"] != "2.2") continue;

                for (size_t i = 0; i < geom["boundaries"].size(); ++i) {
                    const auto& shell = geom["boundaries"][i];
                    const auto& semantics = geom["semantics"]["values"][i];

                    for (size_t j_idx = 0; j_idx < shell.size(); ++j_idx) {
                        const auto& surface = shell[j_idx];
                        int sem_index = semantics[j_idx].get<int>();
                        std::string sem_type = geom["semantics"]["surfaces"][sem_index]["type"];

                        for (const auto& ring : surface) {
                            if (ring.size() < 3) continue;

                            std::vector<std::vector<double>> pts;
                            for (int idx : ring) {
                                pts.push_back(get_point(j, idx));
                            }

                            if (sem_type == "RoofSurface") {
                                auto maybe_normal = compute_newell_normal(pts);
                                if (maybe_normal.has_value()) {
                                    roof_normals.push_back(maybe_normal.value());
                                }
                            }

                            for (size_t i = 1; i + 1 < pts.size(); ++i) {
                                double vol = signed_volume(pts[0], pts[i], pts[i + 1], fake_point);
                                total_volume += vol;
                                total_tetrahedra++;
                            }
                        }
                    }
                }
            }
        }

        std::cout << " Total tetrahedra: " << total_tetrahedra << std::endl;
        std::cout << " Computed volume for building " << id << ": " << std::abs(total_volume) << std::endl;

        j["CityObjects"][id]["attributes"]["volume"] = std::abs(total_volume);

        if (!roof_normals.empty()) {
            double avg_nx = 0, avg_ny = 0, avg_nz = 0;
            for (const auto& n : roof_normals) {
                avg_nx += n[0];
                avg_ny += n[1];
                avg_nz += n[2];
            }
            double len = std::sqrt(avg_nx * avg_nx + avg_ny * avg_ny + avg_nz * avg_nz);
            if (len > 0) {
                std::vector<double> avg_normal = {avg_nx / len, avg_ny / len, avg_nz / len};
                double azimuth = azimuth_from_normal(avg_normal);
                std::cout << " Roof azimuth: " << azimuth << " degrees from North" << std::endl;
                j["CityObjects"][id]["attributes"]["roof_azimuth_deg"] = azimuth;
            }
        }

        nobuildings++;
    }

    std::cout << "Processed " << nobuildings << " buildings." << std::endl;
    std::cout << "Number of vertices " << j["vertices"].size() << std::endl;

    std::ofstream o("out.city.json");
    o << j.dump(2) << std::endl;
    o.close();

    return 0;
}
