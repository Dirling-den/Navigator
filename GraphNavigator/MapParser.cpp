#include "MapParser.hpp"
#include <tinyxml2.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <unordered_set>

namespace {
const std::unordered_set<std::string> kWalkableHighways = {
    "footway", "path", "pedestrian", "steps", "track", "cycleway",
    "living_street", "service", "residential", "unclassified"
};
std::string attribute(const tinyxml2::XMLElement* element, const char* name) {
    const char* value = element->Attribute(name);
    return value ? value : "";
}
}

MapParser::MapParser(const std::string& filepath)
    : min_lat(90.0), max_lat(-90.0), min_lon(180.0), max_lon(-180.0), path(filepath) {}

double MapParser::calculateDistance(double lat1, double lon1, double lat2, double lon2) {
    constexpr double pi = 3.14159265358979323846;
    double dLat = (lat2 - lat1) * pi / 180.0;
    double dLon = (lon2 - lon1) * pi / 180.0;
    double a = std::sin(dLat / 2) * std::sin(dLat / 2) +
        std::cos(lat1 * pi / 180) * std::cos(lat2 * pi / 180) *
        std::sin(dLon / 2) * std::sin(dLon / 2);
    a = std::clamp(a, 0.0, 1.0);
    return 6371000.0 * 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
}

void MapParser::addMapEdge(long long from, long long to, const std::string& type) {
    if (!nodes.count(from) || !nodes.count(to) || from == to) return;
    double distance = calculateDistance(nodes.at(from).lat, nodes.at(from).lon,
        nodes.at(to).lat, nodes.at(to).lon);
    graph[from].push_back({to, distance, type});
    graph[to].push_back({from, distance, type});
}

std::string MapParser::wayRejectReason(const WayTags& t) {
    if (!kWalkableHighways.count(t.highway)) return "not walkable";
    if (t.area == "yes") return "area polygon";
    if (t.foot == "no" || t.foot == "private") return "foot restricted";
    bool allowed = t.foot == "yes" || t.foot == "designated" || t.foot == "permissive";
    if (!allowed && (t.access == "no" || t.access == "private")) return "access restricted";
    return "";
}

bool MapParser::loadMap() {
    // Повторная загрузка не должна сохранять старые вершины, рёбра и границы.
    nodes.clear(); graph.clear();
    min_lat = 90; max_lat = -90; min_lon = 180; max_lon = -180;
    tinyxml2::XMLDocument document;
    if (document.LoadFile(path.c_str()) != tinyxml2::XML_SUCCESS) {
        std::cerr << "[MapParser] Cannot parse " << path << ": " << document.ErrorStr() << '\n';
        return false;
    }
    auto root = document.FirstChildElement("osm");
    if (!root) return false;
    // Сначала все вершины: XML не обязан перечислять их до way.
    for (auto element = root->FirstChildElement("node"); element;
         element = element->NextSiblingElement("node")) {
        int64_t id = 0;
        double lat = 0, lon = 0;
        if (element->QueryInt64Attribute("id", &id) != tinyxml2::XML_SUCCESS ||
            element->QueryDoubleAttribute("lat", &lat) != tinyxml2::XML_SUCCESS ||
            element->QueryDoubleAttribute("lon", &lon) != tinyxml2::XML_SUCCESS ||
            !std::isfinite(lat) || !std::isfinite(lon) || std::abs(lat) > 90 || std::abs(lon) > 180) {
            nodes.clear(); graph.clear();
            min_lat = 90; max_lat = -90; min_lon = 180; max_lon = -180;
            return false;
        }
        Node node{id, lat, lon, 0, 0, ""};
        for (auto tag = element->FirstChildElement("tag"); tag; tag = tag->NextSiblingElement("tag"))
            if (attribute(tag, "k") == "name") node.name = attribute(tag, "v");
        nodes[id] = node;
        min_lat = std::min(min_lat, lat); max_lat = std::max(max_lat, lat);
        min_lon = std::min(min_lon, lon); max_lon = std::max(max_lon, lon);
    }
    for (auto way = root->FirstChildElement("way"); way; way = way->NextSiblingElement("way")) {
        WayTags tags;
        for (auto tag = way->FirstChildElement("tag"); tag; tag = tag->NextSiblingElement("tag")) {
            auto key = attribute(tag, "k"); auto value = attribute(tag, "v");
            if (key == "highway") tags.highway = value;
            else if (key == "foot") tags.foot = value;
            else if (key == "access") tags.access = value;
            else if (key == "area") tags.area = value;
        }
        if (!wayRejectReason(tags).empty()) continue;
        long long previous = 0;
        bool hasPrevious = false;
        for (auto nd = way->FirstChildElement("nd"); nd; nd = nd->NextSiblingElement("nd")) {
            int64_t ref = 0;
            if (nd->QueryInt64Attribute("ref", &ref) != tinyxml2::XML_SUCCESS || !nodes.count(ref)) {
                hasPrevious = false; // Не соединяем дорожку через отсутствующую вершину.
                continue;
            }
            if (hasPrevious) addMapEdge(previous, ref, tags.highway);
            previous = ref; hasPrevious = true;
        }
    }
    return !nodes.empty() && !graph.empty();
}
