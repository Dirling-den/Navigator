// Регрессии нового клиента: чтение .gnpk с проверкой границ и CSR-роутинг.
#include "data/Package.hpp"
#include "routing/Router.hpp"
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <thread>

using namespace gn;

static void require(bool v, const char* msg) { if (!v) throw std::runtime_error(msg); }

// Собирает минимальный корректный пакет вручную (тот же формат, что пишет mapbake.py).
struct Writer {
    std::vector<uint8_t> b;
    template <class T> void put(T v) { uint8_t t[sizeof(T)]; std::memcpy(t, &v, sizeof(T)); b.insert(b.end(), t, t + sizeof(T)); }
};

// Граф-квадрат 0-1-2-3-0 со стороной 100 м + диагональ 0-2 длиной 500 (невыгодная).
static std::vector<uint8_t> squarePackage() {
    Writer w;
    for (char c : std::string("GNPK")) w.put<char>(c);
    w.put<uint32_t>(1); w.put<double>(55.75); w.put<double>(37.6);
    struct E { uint32_t a, b; float wt; };
    std::vector<E> es{ {0,1,100}, {1,2,100}, {2,3,100}, {3,0,100}, {0,2,500} };
    std::vector<std::vector<std::pair<uint32_t, float>>> adj(4);
    for (auto& e : es) { adj[e.a].push_back({ e.b, e.wt }); adj[e.b].push_back({ e.a, e.wt }); }
    uint32_t ne = 0; for (auto& a : adj) ne += uint32_t(a.size());
    for (uint32_t v : { 4u, ne, 1u, 1u, 1u, 1u, 1u }) w.put<uint32_t>(v);
    float xs[] = { 0, 100, 100, 0 }, zs[] = { 0, 0, 100, 100 };
    for (int i = 0; i < 4; ++i) { w.put(xs[i]); w.put(zs[i]); }
    uint32_t off = 0; for (auto& a : adj) { w.put(off); off += uint32_t(a.size()); } w.put(off);
    for (auto& a : adj) for (auto& [to, wt] : a) { w.put(to); w.put(wt); w.put<uint8_t>(1); }
    w.put<uint32_t>(3); w.put<float>(12); for (float v : { 0.f, 0.f, 10.f, 0.f, 10.f, 10.f }) w.put(v);   // здание
    w.put<float>(5); w.put<float>(5);                                                                  // дерево
    w.put<uint8_t>(4); w.put<float>(7); w.put<uint32_t>(2); for (float v : { 0.f, 0.f, 100.f, 0.f }) w.put(v); // дорога
    w.put<uint32_t>(2);                                                                                // переход
    std::string nm = "Фонтан"; w.put<uint32_t>(2); w.put<uint16_t>(uint16_t(nm.size()));
    w.b.insert(w.b.end(), nm.begin(), nm.end());
    return w.b;
}

int main(int argc, char** argv) {
    try {
        auto bytes = squarePackage();
        CityPackage pkg; std::string err;
        require(parsePackage(bytes, pkg, err), "valid package rejected");
        require(pkg.graph.nodeCount() == 4 && pkg.buildings.size() == 1 && pkg.names[0].name == "Фонтан", "fields");

        // Любое усечение файла — ошибка, а не падение.
        for (size_t n = 0; n < bytes.size(); ++n) {
            std::vector<uint8_t> cut(bytes.begin(), bytes.begin() + n);
            CityPackage p; require(!parsePackage(cut, p, err), "truncated package accepted");
        }
        // Порча каждого байта не должна приводить к падению (результат любой).
        for (size_t i = 0; i < bytes.size(); ++i) {
            auto bad = bytes; bad[i] ^= 0xFF; CityPackage p; parsePackage(bad, p, err);
        }
        { auto bad = bytes; bad.push_back(0); CityPackage p; require(!parsePackage(bad, p, err), "trailing data accepted"); }
        { auto bad = bytes; bad[24] = 0xFF; bad[25] = 0xFF; bad[26] = 0xFF; bad[27] = 0x7F;   // огромный edge_count
          CityPackage p; require(!parsePackage(bad, p, err), "huge count accepted"); }

        Router r(pkg.graph);
        auto a = r.aStar(0, 2), d = r.dijkstra({ 0 }, 2);
        require(a.success && std::abs(a.lengthM - 200) < 1e-3 && a.nodes.size() == 3, "A* shortest");
        require(d.success && std::abs(d.lengthM - 200) < 1e-3, "Dijkstra shortest");
        require(r.dijkstra({ 1, 3 }, 2).lengthM == 100, "multi-source");
        auto t = r.greedyTsp({ 0, 2, 1 });
        require(t.success && std::abs(t.lengthM - 200) < 1e-3, "TSP via points");
        require(r.greedyTsp({ 0, 0 }).success, "TSP coincident points");
        require(!r.aStar(0, 99).success && !r.aStar(99, 0).success, "unknown node");
        require(r.nearestNode({ 95, 4 }) == 1, "nearest node");
        auto via = r.run(Algorithm::AStar, { 0, 1, 2 });
        require(via.success && via.nodes.size() == 3, "via route");

        // Рабочий поток возвращает результат.
        RouteWorker wk(pkg.graph);
        wk.requestCompare({ 0, 2 });
        std::vector<Route> res; bool cmp = false;
        for (int i = 0; i < 200 && !wk.poll(res, cmp); ++i) std::this_thread::sleep_for(std::chrono::milliseconds(5));
        require(cmp && res.size() == 3 && res[0].success && res[1].success, "worker compare");

        // Реальный пакет из mapbake: A* и Dijkstra дают одинаковую длину.
        if (argc > 1) {
            CityPackage real;
            require(loadPackage(argv[1], real, err), ("demo package: " + err).c_str());
            Router rr(real.graph);
            size_t n = real.graph.nodeCount(), checked = 0;
            for (size_t i = 0; i < n; i += std::max<size_t>(1, n / 25))
                for (size_t j = 0; j < n; j += std::max<size_t>(1, n / 25)) {
                    auto x = rr.aStar(uint32_t(i), uint32_t(j)), y = rr.dijkstra({ uint32_t(i) }, uint32_t(j));
                    require(x.success == y.success && (!x.success || std::abs(x.lengthM - y.lengthM) < 0.01), "A* vs Dijkstra");
                    ++checked;
                }
            std::cout << "real package pairs checked: " << checked << '\n';
        }
        std::cout << "All package/routing checks passed\n";
        return 0;
    } catch (const std::exception& e) { std::cerr << "FAIL: " << e.what() << '\n'; return 1; }
}
