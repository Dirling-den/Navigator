// Проверяем реальные функции приложения, без открытия графического окна.
#define main graphNavigatorEntryPoint
#include "../main.cpp"
#undef main
#include <stdexcept>

void require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}
int main() {
    try {
        require(NormalizeName(" Main GATE ") == "maingate", "ASCII normalization");
        require(NormalizeName("ЁЛКА ВХОД") == NormalizeName("ёлка вход"), "Russian case folding");
        std::unordered_map<long long, Node> nodes = {
            {1, {1, 55, 37, 0, 0, "Main Gate"}},
            {2, {2, 55.001, 37, 0, 0, "Фонтан"}}
        };
        std::unordered_map<long long, std::vector<Edge>> graph = {
            {1, {{2, 111.2, "path"}}}, {2, {{1, 111.2, "path"}}}
        };
        require(FindValidRouterNode(nodes, graph, "ФОНТАН") == 2, "Russian search");
        require(FindValidRouterNode(nodes, graph, "Main Gate typo") == -1, "no reverse substring false match");
        ScaleGraphTo3DSpace(nodes, 55, 55.001, 37, 37, 8000, 6000);
        require(std::isfinite(nodes.at(1).x) && nodes.at(1).y > nodes.at(2).y, "single longitude map");
        require(std::abs(GeoDistanceMeters(55, 37, 55.001, 37) - 111.1949) < 0.1, "geographic demo weight");
        // Демо-сетка должна давать одинаковую оптимальную длину в A* и Дейкстре.
        nodes.clear(); graph.clear();
        for (int r = 0; r < 7; ++r) for (int c = 0; c < 7; ++c) {
            long long id = r * 7 + c + 1;
            nodes[id] = {id, 55.7035 + r * 0.001, 37.5150 + c * 0.003, 0, 0, ""};
        }
        auto connect = [&](long long a, long long b) {
            double d = GeoDistanceMeters(nodes.at(a).lat, nodes.at(a).lon, nodes.at(b).lat, nodes.at(b).lon);
            graph[a].push_back({b, d, "path"}); graph[b].push_back({a, d, "path"});
        };
        for (int r = 0; r < 7; ++r) for (int c = 0; c < 7; ++c) {
            long long id = r * 7 + c + 1;
            if (c < 6) connect(id, id + 1);
            if (r < 6) connect(id, id + 7);
        }
        Navigator nav(nodes, graph);
        for (long long start = 1; start <= 49; ++start) for (long long end = 1; end <= 49; ++end) {
            auto a = nav.findPathAStar(start, end); auto d = nav.findPathMultiSource({start}, end);
            require(a.success && d.success && std::abs(a.total_length - d.total_length) < 0.01, "A* vs Dijkstra demo grid");
        }
        std::cout << "All application logic checks passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n'; return 1;
    }
}
