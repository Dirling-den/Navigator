#include "Package.hpp"
#include <cmath>
#include <cstring>
#include <fstream>

namespace gn {
namespace {

// Читатель с проверкой границ: любой выход за конец буфера помечает ошибку.
class Reader {
public:
    explicit Reader(const std::vector<uint8_t>& b) : m_b(b) {}
    bool ok() const { return m_ok; }
    size_t remaining() const { return m_ok ? m_b.size() - m_pos : 0; }

    template <class T> T get() {
        T v{};
        if (!m_ok || m_b.size() - m_pos < sizeof(T)) { m_ok = false; return v; }
        std::memcpy(&v, m_b.data() + m_pos, sizeof(T));   // little-endian (x86/x64)
        m_pos += sizeof(T);
        return v;
    }
    std::string str(size_t n) {
        if (!m_ok || m_b.size() - m_pos < n) { m_ok = false; return {}; }
        std::string s(reinterpret_cast<const char*>(m_b.data() + m_pos), n);
        m_pos += n;
        return s;
    }
    // Проверка заявленного количества записей: не даём огромному count
    // из битого файла вызвать гигантский reserve.
    bool fits(uint64_t count, uint64_t minBytesEach) {
        if (!m_ok || count * minBytesEach > remaining()) m_ok = false;
        return m_ok;
    }
    Vec2f point() {
        Vec2f p{ get<float>(), get<float>() };
        if (!std::isfinite(p.x) || !std::isfinite(p.z)) m_ok = false;
        return p;
    }
private:
    const std::vector<uint8_t>& m_b;
    size_t m_pos = 0;
    bool m_ok = true;
};

bool fail(std::string& error, const char* what) { error = what; return false; }

} // namespace

bool parsePackage(const std::vector<uint8_t>& bytes, CityPackage& out, std::string& error) {
    out = CityPackage{};
    Reader r(bytes);
    if (r.str(4) != "GNPK") return fail(error, "not a GNPK package");
    if (r.get<uint32_t>() != 1) return fail(error, "unsupported package version");
    out.originLat = r.get<double>();
    out.originLon = r.get<double>();
    uint32_t nNodes = r.get<uint32_t>(), nEdges = r.get<uint32_t>(), nBuild = r.get<uint32_t>(),
             nTrees = r.get<uint32_t>(), nRoads = r.get<uint32_t>(), nCross = r.get<uint32_t>(),
             nNames = r.get<uint32_t>();
    if (!r.ok()) return fail(error, "truncated header");

    auto& g = out.graph;
    if (!r.fits(nNodes, 8)) return fail(error, "truncated nodes");
    g.nodes.resize(nNodes);
    for (auto& p : g.nodes) p = r.point();
    if (!r.fits(uint64_t(nNodes) + 1, 4)) return fail(error, "truncated offsets");
    g.offsets.resize(size_t(nNodes) + 1);
    for (auto& o : g.offsets) o = r.get<uint32_t>();
    // Смещения должны начинаться с 0, не убывать и заканчиваться числом рёбер.
    if (g.offsets.front() != 0 || g.offsets.back() != nEdges) return fail(error, "bad CSR offsets");
    for (size_t i = 1; i < g.offsets.size(); ++i)
        if (g.offsets[i] < g.offsets[i - 1]) return fail(error, "bad CSR offsets");
    if (!r.fits(nEdges, 9)) return fail(error, "truncated edges");
    g.edges.resize(nEdges);
    for (auto& e : g.edges) {
        e.to = r.get<uint32_t>(); e.weight = r.get<float>(); e.type = r.get<uint8_t>();
        if (e.to >= nNodes || !(e.weight >= 0) || !std::isfinite(e.weight))
            return fail(error, "bad edge");
    }

    if (!r.fits(nBuild, 8)) return fail(error, "truncated buildings");
    out.buildings.resize(nBuild);
    for (auto& b : out.buildings) {
        uint32_t n = r.get<uint32_t>(); b.height = r.get<float>();
        if (!r.fits(n, 8) || n < 3 || !(b.height > 0 && b.height < 1000)) return fail(error, "bad building");
        b.outline.resize(n);
        for (auto& p : b.outline) p = r.point();
    }
    if (!r.fits(nTrees, 8)) return fail(error, "truncated trees");
    out.trees.resize(nTrees);
    for (auto& t : out.trees) t = r.point();

    if (!r.fits(nRoads, 9)) return fail(error, "truncated roads");
    out.roads.resize(nRoads);
    for (auto& rd : out.roads) {
        rd.kind = r.get<uint8_t>(); rd.width = r.get<float>();
        uint32_t n = r.get<uint32_t>();
        if (!r.fits(n, 8) || n < 2 || !(rd.width > 0 && rd.width < 100)) return fail(error, "bad road");
        rd.points.resize(n);
        for (auto& p : rd.points) p = r.point();
    }
    if (!r.fits(nCross, 4)) return fail(error, "truncated crossings");
    out.crossings.resize(nCross);
    for (auto& c : out.crossings) {
        c = r.get<uint32_t>();
        if (c >= nNodes) return fail(error, "bad crossing");
    }
    if (!r.fits(nNames, 6)) return fail(error, "truncated names");
    out.names.resize(nNames);
    for (auto& nm : out.names) {
        nm.node = r.get<uint32_t>();
        nm.name = r.str(r.get<uint16_t>());
        if (nm.node >= nNodes) return fail(error, "bad name");
    }
    if (!r.ok()) return fail(error, "truncated or corrupted package");
    if (r.remaining() != 0) return fail(error, "trailing data");
    return true;
}

bool loadPackage(const std::string& path, CityPackage& out, std::string& error) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return fail(error, "cannot open file");
    std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    return parsePackage(bytes, out, error);
}

} // namespace gn
