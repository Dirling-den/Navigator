#include "Navigator.hpp"
#include "MapParser.hpp"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

void check(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}
int main() {
    try {
        std::unordered_map<long long, Node> nodes;
        for (long long id = 1; id <= 4; ++id) nodes[id] = Node{id, 0, 0, 0, 0, ""};
        std::unordered_map<long long, std::vector<Edge>> graph = {
            {1, {{2, 10, "path"}, {2, 2, "path"}, {3, 9, "path"}}},
            {2, {{3, 3, "path"}}}
        };
        Navigator nav(nodes, graph);
        for (auto route : {nav.findPathAStar(1, 3), nav.findPathMultiSource({1}, 3)}) {
            check(route.success && route.node_ids == std::vector<long long>({1, 2, 3}), "shortest route");
            check(std::abs(route.total_length - 5) < 1e-8, "parallel edge length");
        }
        check(!nav.findPathAStar(1, 4).success, "unreachable goal");
        check(!nav.findPathAStar(99, 99).success, "unknown A* node");
        check(!nav.findPathMultiSource({99}, 99).success, "unknown Dijkstra node");
        check(nav.findPathMultiSource({1, 2}, 3).total_length == 3, "multi source");
        auto same = nav.findPathGreedyTSP({1, 1});
        check(same.success && same.node_ids == std::vector<long long>({1}) && same.total_length == 0, "duplicate TSP");
        check(nav.findPathGreedyTSP({1, 1, 2, 3}).success, "repeated TSP stop");
        check(!nav.findPathGreedyTSP({99}).success, "unknown TSP node");

        auto file = std::filesystem::current_path() / "graph_regression.osm";
        auto write = [&](const char* text) { std::ofstream out(file); out << text; };
        write("<osm><way id='9'><nd ref='1'/><nd ref='2'/><tag k='highway' v='footway'/></way>"
              "<node id = '1' lat='55' lon='37'><tag k='name' v='Gate &amp; Park'/></node>"
              "<node id='2'\n lat='55.001' lon='37'/></osm>");
        MapParser parser(file.string());
        check(parser.loadMap(), "compact XML and ways before nodes");
        check(parser.getNodes().at(1).name == "Gate & Park", "XML entity decoding");
        check(parser.getGraph().at(1).size() == 1, "parsed edge");
        check(parser.loadMap() && parser.getGraph().at(1).size() == 1, "reload does not duplicate");
        write("<osm><node id='3' lat='10' lon='20'/></osm>");
        check(!parser.loadMap() && parser.getGraph().empty() && parser.getNodes().size() == 1, "empty graph and old state cleared");
        write("<osm><node id='x' lat='a' lon='37'/></osm>");
        check(!parser.loadMap() && parser.getNodes().empty(), "bad coordinates fail safely");
        write("<osm><node></osm>");
        check(!parser.loadMap(), "malformed XML");
        std::filesystem::remove(file);
        std::cout << "All graph regression checks passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
