// Router.hpp — A*, Dijkstra и жадный TSP на CSR-графе + рабочий поток.
// Логика перенесена из старого Navigator.cpp, но вершины теперь — индексы 0..N-1,
// а вместо unordered_map используются плоские массивы (быстро на больших графах).
#pragma once
#include "../data/Package.hpp"
#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>

namespace gn {

enum class Algorithm { AStar = 0, Dijkstra = 1, GreedyTsp = 2 };

struct Route {
    std::vector<uint32_t> nodes;   // вершины маршрута по порядку
    double lengthM = 0;            // длина в метрах
    double timeMs = 0;             // время работы алгоритма
    size_t visited = 0;            // сколько вершин раскрыто (для сравнения алгоритмов)
    bool success = false;
};

class Router {
public:
    explicit Router(const CsrGraph& g) : m_g(g) {}
    Route aStar(uint32_t start, uint32_t goal) const;
    Route dijkstra(const std::vector<uint32_t>& starts, uint32_t goal) const;
    // Открытый жадный маршрут: из текущей точки идём к ближайшей (по прямой) непосещённой.
    Route greedyTsp(const std::vector<uint32_t>& points) const;
    Route run(Algorithm a, const std::vector<uint32_t>& points) const;
    // Ближайшая вершина графа к точке (линейный поиск; для района достаточно).
    uint32_t nearestNode(Vec2f p) const;
private:
    Route search(const std::vector<uint32_t>& starts, uint32_t goal, bool heuristic) const;
    const CsrGraph& m_g;
};

// Выполняет запросы маршрута в фоне, чтобы окно не подвисало.
// Новый запрос заменяет ещё не начатый; результат забирается через poll() в главном потоке.
class RouteWorker {
public:
    explicit RouteWorker(const CsrGraph& g);
    ~RouteWorker();
    void request(Algorithm a, std::vector<uint32_t> points);
    void requestCompare(std::vector<uint32_t> points);       // все три алгоритма
    bool busy() const { return m_busy; }
    // true, если появился новый результат. Для сравнения заполняется все три элемента.
    bool poll(std::vector<Route>& results, bool& compare);
private:
    void loop();
    Router m_router;
    std::thread m_thread;
    std::mutex m_mx;
    std::condition_variable m_cv;
    bool m_stop = false, m_hasJob = false, m_jobCompare = false, m_hasResult = false, m_resCompare = false;
    Algorithm m_jobAlg = Algorithm::AStar;
    std::vector<uint32_t> m_jobPoints;
    std::vector<Route> m_results;
    std::atomic<bool> m_busy{ false };
};

} // namespace gn
