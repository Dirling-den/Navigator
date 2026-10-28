#include "Router.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>

namespace gn {
namespace {
using Clock = std::chrono::steady_clock;
constexpr uint32_t kNone = std::numeric_limits<uint32_t>::max();
double dist(Vec2f a, Vec2f b) { return std::hypot(double(a.x) - b.x, double(a.z) - b.z); }
}

// Общий поиск: heuristic=false — Dijkstra (много стартов), true — A*.
// Эвристика — прямое расстояние в локальных метрах. Она допустима, потому что
// вес ребра = геодезическая длина отрезка, а она не меньше прямой на плоскости
// (с небольшим запасом 0.995 на погрешность проекции).
Route Router::search(const std::vector<uint32_t>& starts, uint32_t goal, bool heuristic) const {
    Route r;
    const size_t n = m_g.nodeCount();
    if (goal >= n) return r;
    auto t0 = Clock::now();
    std::vector<double> g(n, std::numeric_limits<double>::infinity());
    std::vector<uint32_t> prev(n, kNone);
    using Item = std::pair<double, uint32_t>;   // (приоритет, вершина)
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open;
    auto h = [&](uint32_t v) { return heuristic ? 0.995 * dist(m_g.nodes[v], m_g.nodes[goal]) : 0.0; };
    for (uint32_t s : starts) if (s < n) { g[s] = 0; open.push({ h(s), s }); }

    while (!open.empty()) {
        auto [f, u] = open.top(); open.pop();
        if (f > g[u] + h(u) + 1e-9) continue;   // устаревшая запись очереди
        ++r.visited;
        if (u == goal) {
            r.success = true; r.lengthM = g[u];
            for (uint32_t v = goal; v != kNone; v = prev[v]) r.nodes.push_back(v);
            std::reverse(r.nodes.begin(), r.nodes.end());
            break;
        }
        for (uint32_t i = m_g.offsets[u]; i < m_g.offsets[u + 1]; ++i) {
            const CsrEdge& e = m_g.edges[i];
            double cand = g[u] + e.weight;
            if (cand < g[e.to]) { g[e.to] = cand; prev[e.to] = u; open.push({ cand + h(e.to), e.to }); }
        }
    }
    r.timeMs = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    return r;
}

Route Router::aStar(uint32_t s, uint32_t goal) const {
    if (s >= m_g.nodeCount()) return {};
    return search({ s }, goal, true);
}
Route Router::dijkstra(const std::vector<uint32_t>& starts, uint32_t goal) const { return search(starts, goal, false); }

Route Router::greedyTsp(const std::vector<uint32_t>& pts) const {
    Route r;
    for (uint32_t p : pts) if (p >= m_g.nodeCount()) return r;
    if (pts.empty()) return r;
    auto t0 = Clock::now();
    std::vector<uint32_t> left(pts.begin() + 1, pts.end());
    uint32_t cur = pts.front();
    r.nodes.push_back(cur); r.success = true;
    while (!left.empty()) {
        auto it = std::min_element(left.begin(), left.end(), [&](uint32_t a, uint32_t b) {
            return dist(m_g.nodes[cur], m_g.nodes[a]) < dist(m_g.nodes[cur], m_g.nodes[b]); });
        uint32_t next = *it; left.erase(it);
        Route leg = aStar(cur, next);
        if (!leg.success) { r.success = false; break; }
        r.nodes.insert(r.nodes.end(), leg.nodes.begin() + 1, leg.nodes.end());
        r.lengthM += leg.lengthM; r.visited += leg.visited;
        cur = next;
    }
    if (!r.success) r.nodes.clear();
    r.timeMs = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    return r;
}

Route Router::run(Algorithm a, const std::vector<uint32_t>& p) const {
    if (a == Algorithm::GreedyTsp) return greedyTsp(p);
    if (p.size() < 2) return {};
    // Для A*/Dijkstra с промежуточными точками маршрут собирается по отрезкам.
    Route total; total.success = true;
    for (size_t i = 0; i + 1 < p.size(); ++i) {
        Route leg = (a == Algorithm::AStar) ? aStar(p[i], p[i + 1]) : dijkstra({ p[i] }, p[i + 1]);
        if (!leg.success) return Route{ {}, 0, total.timeMs + leg.timeMs, total.visited + leg.visited, false };
        if (total.nodes.empty()) total.nodes = leg.nodes;
        else total.nodes.insert(total.nodes.end(), leg.nodes.begin() + 1, leg.nodes.end());
        total.lengthM += leg.lengthM; total.timeMs += leg.timeMs; total.visited += leg.visited;
    }
    return total;
}

uint32_t Router::nearestNode(Vec2f p) const {
    uint32_t best = kNone; double bd = std::numeric_limits<double>::infinity();
    for (uint32_t i = 0; i < m_g.nodeCount(); ++i) {
        if (m_g.offsets[i] == m_g.offsets[i + 1]) continue;   // изолированные вершины пропускаем
        double d = dist(p, m_g.nodes[i]);
        if (d < bd) { bd = d; best = i; }
    }
    return best;
}

RouteWorker::RouteWorker(const CsrGraph& g) : m_router(g), m_thread([this] { loop(); }) {}
RouteWorker::~RouteWorker() {
    { std::lock_guard<std::mutex> l(m_mx); m_stop = true; }
    m_cv.notify_all();
    m_thread.join();
}
void RouteWorker::request(Algorithm a, std::vector<uint32_t> pts) {
    { std::lock_guard<std::mutex> l(m_mx); m_jobAlg = a; m_jobPoints = std::move(pts); m_jobCompare = false; m_hasJob = true; m_busy = true; }
    m_cv.notify_one();
}
void RouteWorker::requestCompare(std::vector<uint32_t> pts) {
    { std::lock_guard<std::mutex> l(m_mx); m_jobPoints = std::move(pts); m_jobCompare = true; m_hasJob = true; m_busy = true; }
    m_cv.notify_one();
}
bool RouteWorker::poll(std::vector<Route>& results, bool& compare) {
    std::lock_guard<std::mutex> l(m_mx);
    if (!m_hasResult) return false;
    results = std::move(m_results); compare = m_resCompare; m_hasResult = false;
    return true;
}
void RouteWorker::loop() {
    for (;;) {
        Algorithm alg; std::vector<uint32_t> pts; bool cmp;
        {
            std::unique_lock<std::mutex> l(m_mx);
            m_cv.wait(l, [this] { return m_stop || m_hasJob; });
            if (m_stop) return;
            alg = m_jobAlg; pts = std::move(m_jobPoints); cmp = m_jobCompare; m_hasJob = false;
        }
        std::vector<Route> res;
        if (cmp) for (int a = 0; a < 3; ++a) res.push_back(m_router.run(Algorithm(a), pts));
        else res.push_back(m_router.run(alg, pts));
        std::lock_guard<std::mutex> l(m_mx);
        m_results = std::move(res); m_resCompare = cmp; m_hasResult = true;
        if (!m_hasJob) m_busy = false;
    }
}

} // namespace gn
